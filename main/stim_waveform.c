#include "stim_waveform.h"

#include <stddef.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_private/esp_clk_tree_common.h"
#include "esp_private/gdma.h"
#include "esp_private/periph_ctrl.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "hal/dma_types.h"
#include "hal/lcd_hal.h"
#include "hal/lcd_ll.h"
#include "soc/gpio_sig_map.h"
#include "stim_protocol.h"
#include "stim_waveform_builder.h"

static const char *TAG = "STIM_WAVEFORM";

enum {
    STIM_START_DESCRIPTOR_COUNT = STIM_PROTOCOL_START_FRAME_COUNT,
};

typedef struct {
    bool initialized;
    volatile bool fatal;
    volatile bool requested_enabled;
    volatile stim_waveform_state_t state;
    volatile uint32_t completed_start_frames;
    volatile uint32_t descriptor_errors;
    lcd_hal_context_t lcd_hal;
    gdma_channel_handle_t dma_channel;
    stim_waveform_fault_callback_t fault_callback;
    void *fault_user_data;
    portMUX_TYPE lock;
} stim_waveform_context_t;

DMA_ATTR static uint8_t s_stop_loop[STIM_SLOT_SAMPLES];
DMA_ATTR static uint8_t s_start_sequence[STIM_START_SEQUENCE_SAMPLES];
DMA_ATTR static uint8_t s_enabled_idle[STIM_SLOT_SAMPLES];

DMA_ATTR static dma_descriptor_align4_t s_stop_descriptor;
DMA_ATTR static dma_descriptor_align4_t s_stop_entry_descriptor;
DMA_ATTR static dma_descriptor_align4_t
    s_start_descriptors[STIM_START_DESCRIPTOR_COUNT];
DMA_ATTR static dma_descriptor_align4_t s_enabled_descriptor;

static stim_waveform_context_t s_waveform = {
    .lock = portMUX_INITIALIZER_UNLOCKED,
};

static void stim_waveform_init_descriptor(dma_descriptor_align4_t *descriptor,
                                          uint8_t *buffer,
                                          bool signal_eof,
                                          dma_descriptor_align4_t *next)
{
    memset(descriptor, 0, sizeof(*descriptor));
    descriptor->dw0.size = STIM_SLOT_SAMPLES;
    descriptor->dw0.length = STIM_SLOT_SAMPLES;
    descriptor->dw0.suc_eof = signal_eof ? 1U : 0U;
    descriptor->dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
    descriptor->buffer = buffer;
    descriptor->next = next;
}

static void stim_waveform_restore_start_chain(void)
{
    for (size_t index = 0U; index < STIM_START_DESCRIPTOR_COUNT; ++index) {
        s_start_descriptors[index].next =
            index + 1U < STIM_START_DESCRIPTOR_COUNT
                ? &s_start_descriptors[index + 1U]
                : &s_enabled_descriptor;
    }
}

static void stim_waveform_configure_descriptors(void)
{
    stim_waveform_init_descriptor(
        &s_stop_descriptor, s_stop_loop, false, &s_stop_descriptor);
    stim_waveform_init_descriptor(
        &s_stop_entry_descriptor, s_stop_loop, true, &s_stop_descriptor);
    stim_waveform_init_descriptor(
        &s_enabled_descriptor, s_enabled_idle, false, &s_enabled_descriptor);

    for (size_t index = 0U; index < STIM_START_DESCRIPTOR_COUNT; ++index) {
        stim_waveform_init_descriptor(
            &s_start_descriptors[index],
            s_start_sequence + index * STIM_SLOT_SAMPLES,
            true,
            index + 1U < STIM_START_DESCRIPTOR_COUNT
                ? &s_start_descriptors[index + 1U]
                : &s_enabled_descriptor);
    }
}

static bool IRAM_ATTR stim_waveform_on_descriptor_error(
    gdma_channel_handle_t dma_channel,
    gdma_event_data_t *event_data,
    void *user_data)
{
    (void)dma_channel;
    (void)event_data;
    (void)user_data;

    ++s_waveform.descriptor_errors;
    s_waveform.fatal = true;
    s_waveform.state = STIM_WAVEFORM_FAULT;
    if (s_waveform.fault_callback != NULL) {
        return s_waveform.fault_callback(s_waveform.fault_user_data);
    }
    return false;
}

static bool IRAM_ATTR stim_waveform_on_trans_eof(
    gdma_channel_handle_t dma_channel,
    gdma_event_data_t *event_data,
    void *user_data)
{
    (void)dma_channel;
    (void)user_data;

    dma_descriptor_align4_t *completed =
        (dma_descriptor_align4_t *)event_data->tx_eof_desc_addr;

    if (completed == &s_stop_entry_descriptor) {
        s_enabled_descriptor.next = &s_enabled_descriptor;
        stim_waveform_restore_start_chain();
        s_waveform.state = STIM_WAVEFORM_STOP_LOOP;
        s_waveform.completed_start_frames = 0U;
        if (s_waveform.requested_enabled) {
            s_stop_descriptor.next = &s_start_descriptors[0];
        }
        return false;
    }

    for (size_t index = 0U; index < STIM_START_DESCRIPTOR_COUNT; ++index) {
        if (completed != &s_start_descriptors[index]) {
            continue;
        }
        s_waveform.state = STIM_WAVEFORM_START_SEQUENCE;
        s_waveform.completed_start_frames = (uint32_t)index + 1U;
        if (index == 0U) {
            s_stop_descriptor.next = &s_stop_descriptor;
        }
        if (!s_waveform.requested_enabled) {
            s_start_descriptors[index].next = &s_stop_entry_descriptor;
        } else if (index + 1U == STIM_START_DESCRIPTOR_COUNT) {
            s_waveform.state = STIM_WAVEFORM_ENABLED_IDLE;
        }
        return false;
    }
    return false;
}

static esp_err_t stim_waveform_configure_safe_gpio(void)
{
    const gpio_config_t output_config = {
        .pin_bit_mask = (UINT64_C(1) << STIM_MCLK_GPIO) |
                        (UINT64_C(1) << STIM_SCLK_GPIO) |
                        (UINT64_C(1) << STIM_MOSI_GPIO) |
                        (UINT64_C(1) << STIM_CSB_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t result = gpio_config(&output_config);
    if (result != ESP_OK) {
        return result;
    }
    gpio_set_level(STIM_MOSI_GPIO, 1);
    gpio_set_level(STIM_CSB_GPIO, 1);
    gpio_set_level(STIM_SCLK_GPIO, 0);
    gpio_set_level(STIM_MCLK_GPIO, 1);
    return ESP_OK;
}

static void stim_waveform_route_gpio_matrix(void)
{
    esp_rom_gpio_connect_out_signal(STIM_SCLK_GPIO, LCD_PCLK_IDX, false, false);
    esp_rom_gpio_connect_out_signal(STIM_MCLK_GPIO, LCD_PCLK_IDX, true, false);
    esp_rom_gpio_connect_out_signal(STIM_MOSI_GPIO, LCD_DATA_OUT0_IDX, false, false);
    esp_rom_gpio_connect_out_signal(STIM_CSB_GPIO, LCD_DATA_OUT1_IDX, false, false);
}

static esp_err_t stim_waveform_configure_lcd_cam(void)
{
    esp_err_t result = esp_clk_tree_enable_src(
        (soc_module_clk_t)LCD_CLK_SRC_PLL160M, true);
    if (result != ESP_OK) {
        return result;
    }

    periph_module_enable(PERIPH_LCD_CAM_MODULE);
    periph_module_reset(PERIPH_LCD_CAM_MODULE);
    lcd_hal_init(&s_waveform.lcd_hal, 0);
    lcd_cam_dev_t *dev = s_waveform.lcd_hal.dev;

    PERIPH_RCC_ATOMIC() {
        lcd_ll_enable_clock(dev, true);
        lcd_ll_select_clk_src(dev, LCD_CLK_SRC_PLL160M);
        lcd_ll_set_group_clock_coeff(dev, 24, 33, 8);
    }
    lcd_ll_reset(dev);
    lcd_ll_fifo_reset(dev);
    lcd_ll_enable_rgb_mode(dev, false);
    lcd_ll_enable_color_convert(dev, false);
    lcd_ll_set_dma_read_stride(dev, 8);
    lcd_ll_set_data_wire_width(dev, 8);
    lcd_ll_reverse_dma_data_bit_order(dev, false);
    lcd_ll_swap_dma_data_byte_order(dev, false);
    lcd_ll_enable_swizzle(dev, false);
    lcd_ll_set_clock_idle_level(dev, false);
    lcd_ll_set_pixel_clock_edge(dev, true);
    lcd_ll_set_pixel_clock_prescale(dev, 1);
    lcd_ll_set_phase_cycles(dev, 0, 0, 1);
    lcd_ll_set_blank_cycles(dev, 0, 0);
    lcd_ll_enable_output_always_on(dev, true);
    PERIPH_RCC_ATOMIC() {
        lcd_ll_enable_interrupt(dev, UINT32_MAX, false);
    }
    lcd_ll_clear_interrupt_status(dev, UINT32_MAX);
    return ESP_OK;
}

esp_err_t stim_waveform_init(stim_waveform_fault_callback_t fault_callback,
                             void *user_data)
{
    if (s_waveform.initialized) {
        return ESP_OK;
    }

    esp_err_t result = stim_waveform_configure_safe_gpio();
    if (result != ESP_OK) {
        return result;
    }
    if (!stim_protocol_validate_start_frames()) {
        return ESP_ERR_INVALID_CRC;
    }

    stim_waveform_build_buffers(
        s_stop_loop, s_start_sequence, s_enabled_idle);
    stim_waveform_configure_descriptors();
    s_waveform.fault_callback = fault_callback;
    s_waveform.fault_user_data = user_data;

    result = stim_waveform_configure_lcd_cam();
    if (result != ESP_OK) {
        stim_waveform_enter_safe_state();
        return result;
    }
    stim_waveform_route_gpio_matrix();

    const gdma_channel_alloc_config_t channel_config = {
        .flags.isr_cache_safe = false,
    };
    result = gdma_new_ahb_channel(
        &channel_config, &s_waveform.dma_channel, NULL);
    if (result == ESP_OK) {
        result = gdma_connect(
            s_waveform.dma_channel,
            GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));
    }
    if (result == ESP_OK) {
        const gdma_strategy_config_t strategy = {
            .owner_check = false,
            .auto_update_desc = false,
            .eof_till_data_popped = true,
        };
        result = gdma_apply_strategy(s_waveform.dma_channel, &strategy);
    }
    if (result == ESP_OK) {
        const gdma_transfer_config_t transfer = {
            .max_data_burst_size = 0,
            .access_ext_mem = false,
        };
        result = gdma_config_transfer(s_waveform.dma_channel, &transfer);
    }
    if (result == ESP_OK) {
        gdma_tx_event_callbacks_t callbacks = {
            .on_trans_eof = stim_waveform_on_trans_eof,
            .on_descr_err = stim_waveform_on_descriptor_error,
        };
        result = gdma_register_tx_event_callbacks(
            s_waveform.dma_channel, &callbacks, &s_waveform);
    }
    if (result == ESP_OK) {
        result = gdma_start(
            s_waveform.dma_channel, (intptr_t)&s_stop_descriptor);
    }
    if (result != ESP_OK) {
        stim_waveform_enter_safe_state();
        return result;
    }

    esp_rom_delay_us(4);
    lcd_ll_start(s_waveform.lcd_hal.dev);
    s_waveform.requested_enabled = false;
    s_waveform.state = STIM_WAVEFORM_STOP_LOOP;
    s_waveform.initialized = true;
    ESP_LOGI(TAG,
             "LCD_CAM/GDMA running: SCLK=GPIO%d, mclkST=GPIO%d, "
             "MOSI=GPIO%d, CSb=GPIO%d, clock=%u Hz",
             STIM_SCLK_GPIO, STIM_MCLK_GPIO, STIM_MOSI_GPIO, STIM_CSB_GPIO,
             STIM_CLOCK_HZ);
    return ESP_OK;
}

esp_err_t stim_waveform_request_enabled(bool enabled)
{
    if (!s_waveform.initialized || s_waveform.fatal) {
        return ESP_ERR_INVALID_STATE;
    }

    portENTER_CRITICAL(&s_waveform.lock);
    s_waveform.requested_enabled = enabled;
    if (enabled) {
        if (s_waveform.state == STIM_WAVEFORM_STOP_LOOP) {
            stim_waveform_restore_start_chain();
            s_stop_descriptor.next = &s_start_descriptors[0];
        }
    } else {
        if (s_waveform.state == STIM_WAVEFORM_ENABLED_IDLE) {
            s_enabled_descriptor.next = &s_stop_entry_descriptor;
        } else if (s_waveform.state == STIM_WAVEFORM_START_SEQUENCE) {
            for (size_t index = 0U;
                 index < STIM_START_DESCRIPTOR_COUNT;
                 ++index) {
                s_start_descriptors[index].next = &s_stop_entry_descriptor;
            }
        }
    }
    portEXIT_CRITICAL(&s_waveform.lock);
    return ESP_OK;
}

void stim_waveform_enter_safe_state(void)
{
    if (s_waveform.dma_channel != NULL) {
        gdma_stop(s_waveform.dma_channel);
    }
    if (s_waveform.lcd_hal.dev != NULL) {
        lcd_ll_stop(s_waveform.lcd_hal.dev);
    }
    esp_rom_gpio_connect_out_signal(
        STIM_MOSI_GPIO, SIG_GPIO_OUT_IDX, false, false);
    esp_rom_gpio_connect_out_signal(
        STIM_CSB_GPIO, SIG_GPIO_OUT_IDX, false, false);
    gpio_set_level(STIM_MOSI_GPIO, 1);
    gpio_set_level(STIM_CSB_GPIO, 1);
    s_waveform.fatal = true;
    s_waveform.state = STIM_WAVEFORM_FAULT;
}

void stim_waveform_get_status(stim_waveform_status_t *status)
{
    if (status == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_waveform.lock);
    status->state = s_waveform.state;
    status->requested_enabled = s_waveform.requested_enabled;
    status->fatal = s_waveform.fatal;
    status->completed_start_frames = s_waveform.completed_start_frames;
    status->descriptor_errors = s_waveform.descriptor_errors;
    portEXIT_CRITICAL(&s_waveform.lock);
}
