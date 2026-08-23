#include "stim_waveform.h"

#include <stddef.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_private/esp_clk_tree_common.h"
#include "esp_private/gdma.h"
#include "esp_private/periph_ctrl.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "hal/dma_types.h"
#include "hal/gdma_ll.h"
#include "hal/gdma_periph.h"
#include "hal/gpio_ll.h"
#include "hal/lcd_hal.h"
#include "hal/lcd_ll.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_struct.h"
#include "soc/interrupts.h"
#include "stim_protocol.h"
#include "stim_waveform_builder.h"

static const char *TAG = "STIM_WAVEFORM";

enum {
    STIM_START_DESCRIPTOR_COUNT = STIM_PROTOCOL_START_FRAME_COUNT,
    STIM_GDMA_FAULT_MASK = GDMA_LL_EVENT_TX_DESC_ERROR |
                           GDMA_LL_EVENT_TX_TOTAL_EOF |
                           GDMA_LL_EVENT_TX_L1_FIFO_UDF |
                           GDMA_LL_EVENT_TX_L3_FIFO_UDF,
    STIM_GDMA_INTERRUPT_MASK = STIM_GDMA_FAULT_MASK |
                               GDMA_LL_EVENT_TX_EOF,
};

typedef struct {
    bool initialized;
    volatile bool fatal;
    volatile bool requested_enabled;
    volatile stim_waveform_state_t state;
    volatile uint32_t completed_start_frames;
    volatile uint32_t descriptor_errors;
    volatile uint32_t fifo_underflow_errors;
    volatile uint32_t unexpected_stop_errors;
    lcd_hal_context_t lcd_hal;
    gdma_channel_handle_t dma_channel;
    gdma_dev_t *gdma_dev;
    int gdma_group_id;
    int gdma_channel_id;
    intr_handle_t gdma_interrupt;
    intr_handle_t lcd_interrupt;
    stim_waveform_event_callback_t event_callback;
    void *event_user_data;
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

static bool IRAM_ATTR stim_waveform_notify_from_isr(stim_waveform_state_t event)
{
    if (s_waveform.event_callback != NULL) {
        return s_waveform.event_callback(event, s_waveform.event_user_data);
    }
    return false;
}

static void IRAM_ATTR stim_waveform_force_data_high_from_isr(void)
{
    gpio_ll_set_level(&GPIO, STIM_MOSI_GPIO, 1U);
    gpio_ll_set_level(&GPIO, STIM_CSB_GPIO, 1U);
    esp_rom_gpio_connect_out_signal(
        STIM_MOSI_GPIO, SIG_GPIO_OUT_IDX, false, false);
    esp_rom_gpio_connect_out_signal(
        STIM_CSB_GPIO, SIG_GPIO_OUT_IDX, false, false);
}

static bool IRAM_ATTR stim_waveform_quench_from_isr(void)
{
    if (s_waveform.fatal) {
        return false;
    }
    s_waveform.fatal = true;
    s_waveform.state = STIM_WAVEFORM_FAULT;
    if (s_waveform.gdma_dev != NULL) {
        gdma_ll_tx_enable_interrupt(s_waveform.gdma_dev,
                                    s_waveform.gdma_channel_id,
                                    STIM_GDMA_INTERRUPT_MASK,
                                    false);
        gdma_ll_tx_stop(s_waveform.gdma_dev, s_waveform.gdma_channel_id);
    }
    if (s_waveform.lcd_hal.dev != NULL) {
        s_waveform.lcd_hal.dev->lc_dma_int_ena.val &=
            ~LCD_LL_EVENT_TRANS_DONE;
        lcd_ll_stop(s_waveform.lcd_hal.dev);
    }
    stim_waveform_force_data_high_from_isr();
    return stim_waveform_notify_from_isr(STIM_WAVEFORM_FAULT);
}

static bool IRAM_ATTR stim_waveform_process_eof(
    dma_descriptor_align4_t *completed)
{
    bool sequence_complete = false;
    portENTER_CRITICAL_ISR(&s_waveform.lock);

    if (completed == &s_stop_entry_descriptor) {
        s_enabled_descriptor.next = &s_enabled_descriptor;
        stim_waveform_restore_start_chain();
        s_waveform.state = STIM_WAVEFORM_STOP_LOOP;
        s_waveform.completed_start_frames = 0U;
        if (s_waveform.requested_enabled) {
            s_stop_descriptor.next = &s_start_descriptors[0];
        }
        portEXIT_CRITICAL_ISR(&s_waveform.lock);
        return stim_waveform_notify_from_isr(STIM_WAVEFORM_STOP_LOOP);
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
            sequence_complete = true;
        }
        portEXIT_CRITICAL_ISR(&s_waveform.lock);
        return sequence_complete
                   ? stim_waveform_notify_from_isr(
                         STIM_WAVEFORM_ENABLED_IDLE)
                   : false;
    }
    ++s_waveform.unexpected_stop_errors;
    portEXIT_CRITICAL_ISR(&s_waveform.lock);
    return stim_waveform_quench_from_isr();
}

static void IRAM_ATTR stim_waveform_gdma_isr(void *argument)
{
    stim_waveform_context_t *context = (stim_waveform_context_t *)argument;
    const uint32_t status = gdma_ll_tx_get_interrupt_status(
        context->gdma_dev, context->gdma_channel_id, false);
    gdma_ll_tx_clear_interrupt_status(
        context->gdma_dev, context->gdma_channel_id, status);

    bool task_woken = false;
    if ((status & STIM_GDMA_FAULT_MASK) != 0U) {
        if ((status & GDMA_LL_EVENT_TX_DESC_ERROR) != 0U) {
            ++context->descriptor_errors;
        }
        if ((status & (GDMA_LL_EVENT_TX_L1_FIFO_UDF |
                       GDMA_LL_EVENT_TX_L3_FIFO_UDF)) != 0U) {
            ++context->fifo_underflow_errors;
        }
        if ((status & GDMA_LL_EVENT_TX_TOTAL_EOF) != 0U) {
            ++context->unexpected_stop_errors;
        }
        task_woken = stim_waveform_quench_from_isr();
    } else if ((status & GDMA_LL_EVENT_TX_EOF) != 0U) {
        dma_descriptor_align4_t *completed =
            (dma_descriptor_align4_t *)(uintptr_t)
                gdma_ll_tx_get_eof_desc_addr(
                    context->gdma_dev, context->gdma_channel_id);
        task_woken = stim_waveform_process_eof(completed);
    }
    if (task_woken) {
        portYIELD_FROM_ISR();
    }
}

static void IRAM_ATTR stim_waveform_lcd_isr(void *argument)
{
    stim_waveform_context_t *context = (stim_waveform_context_t *)argument;
    const uint32_t status = lcd_ll_get_interrupt_status(context->lcd_hal.dev);
    lcd_ll_clear_interrupt_status(context->lcd_hal.dev, status);
    if ((status & LCD_LL_EVENT_TRANS_DONE) != 0U) {
        ++context->unexpected_stop_errors;
        if (stim_waveform_quench_from_isr()) {
            portYIELD_FROM_ISR();
        }
    }
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
        lcd_ll_set_group_clock_coeff(dev, 12, 33, 4);
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
    lcd_ll_set_pixel_clock_prescale(dev, 2);
    /*
     * ESP32-S3 erratum LCD-239 requires ahead_cycle > 2.  Run the LCD core at
     * exactly 13.2 MHz and divide PCLK by two; two command pixels therefore
     * give four LCD_CLK cycles before DMA data.  Both command bytes are 0x03,
     * keeping D1=CSb and D0=MOSI high during this one-time startup preamble.
     */
    lcd_ll_set_command(dev, 8, 0x0303U);
    lcd_ll_set_phase_cycles(dev, 2, 0, 1);
    lcd_ll_set_blank_cycles(dev, 0, 0);
    lcd_ll_enable_output_always_on(dev, true);
    PERIPH_RCC_ATOMIC() {
        lcd_ll_enable_interrupt(dev, UINT32_MAX, false);
    }
    lcd_ll_clear_interrupt_status(dev, UINT32_MAX);
    return ESP_OK;
}

static void stim_waveform_release_interrupts(void)
{
    if (s_waveform.lcd_hal.dev != NULL) {
        PERIPH_RCC_ATOMIC() {
            lcd_ll_enable_interrupt(s_waveform.lcd_hal.dev,
                                    LCD_LL_EVENT_TRANS_DONE,
                                    false);
        }
        lcd_ll_clear_interrupt_status(
            s_waveform.lcd_hal.dev, LCD_LL_EVENT_TRANS_DONE);
    }
    if (s_waveform.lcd_interrupt != NULL) {
        esp_intr_free(s_waveform.lcd_interrupt);
        s_waveform.lcd_interrupt = NULL;
    }
    if (s_waveform.gdma_dev != NULL) {
        gdma_ll_tx_enable_interrupt(s_waveform.gdma_dev,
                                    s_waveform.gdma_channel_id,
                                    STIM_GDMA_INTERRUPT_MASK,
                                    false);
        gdma_ll_tx_clear_interrupt_status(s_waveform.gdma_dev,
                                          s_waveform.gdma_channel_id,
                                          STIM_GDMA_INTERRUPT_MASK);
    }
    if (s_waveform.gdma_interrupt != NULL) {
        esp_intr_free(s_waveform.gdma_interrupt);
        s_waveform.gdma_interrupt = NULL;
    }
}

static esp_err_t stim_waveform_install_interrupts(void)
{
    esp_err_t result = gdma_get_group_channel_id(
        s_waveform.dma_channel,
        &s_waveform.gdma_group_id,
        &s_waveform.gdma_channel_id);
    if (result != ESP_OK) {
        return result;
    }
    s_waveform.gdma_dev = GDMA_LL_GET_HW(
        s_waveform.gdma_group_id - GDMA_LL_AHB_GROUP_START_ID);
    if (s_waveform.gdma_dev == NULL) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    gdma_ll_tx_enable_interrupt(s_waveform.gdma_dev,
                                s_waveform.gdma_channel_id,
                                UINT32_MAX,
                                false);
    gdma_ll_tx_clear_interrupt_status(s_waveform.gdma_dev,
                                      s_waveform.gdma_channel_id,
                                      UINT32_MAX);
    result = esp_intr_alloc_intrstatus(
        gdma_periph_signals.groups[s_waveform.gdma_group_id]
            .pairs[s_waveform.gdma_channel_id]
            .tx_irq_id,
        ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LOWMED,
        (uint32_t)gdma_ll_tx_get_interrupt_status_reg(
            s_waveform.gdma_dev, s_waveform.gdma_channel_id),
        STIM_GDMA_INTERRUPT_MASK,
        stim_waveform_gdma_isr,
        &s_waveform,
        &s_waveform.gdma_interrupt);
    if (result != ESP_OK) {
        return result;
    }
    gdma_ll_tx_enable_interrupt(s_waveform.gdma_dev,
                                s_waveform.gdma_channel_id,
                                STIM_GDMA_INTERRUPT_MASK,
                                true);

    lcd_ll_clear_interrupt_status(
        s_waveform.lcd_hal.dev, LCD_LL_EVENT_TRANS_DONE);
    result = esp_intr_alloc_intrstatus(
        ETS_LCD_CAM_INTR_SOURCE,
        ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_LOWMED,
        (uint32_t)lcd_ll_get_interrupt_status_reg(s_waveform.lcd_hal.dev),
        LCD_LL_EVENT_TRANS_DONE,
        stim_waveform_lcd_isr,
        &s_waveform,
        &s_waveform.lcd_interrupt);
    if (result != ESP_OK) {
        stim_waveform_release_interrupts();
        return result;
    }
    PERIPH_RCC_ATOMIC() {
        lcd_ll_enable_interrupt(s_waveform.lcd_hal.dev,
                                LCD_LL_EVENT_TRANS_DONE,
                                true);
    }
    return ESP_OK;
}

static void stim_waveform_release_dma(void)
{
    stim_waveform_release_interrupts();
    if (s_waveform.dma_channel != NULL) {
        (void)gdma_stop(s_waveform.dma_channel);
        (void)gdma_disconnect(s_waveform.dma_channel);
        (void)gdma_del_channel(s_waveform.dma_channel);
        s_waveform.dma_channel = NULL;
        s_waveform.gdma_dev = NULL;
    }
}

esp_err_t stim_waveform_init(stim_waveform_event_callback_t event_callback,
                             void *user_data)
{
    if (s_waveform.initialized) {
        return ESP_OK;
    }

    s_waveform.fatal = false;
    esp_err_t result = stim_waveform_configure_safe_gpio();
    if (result != ESP_OK) {
        return result;
    }
    uint8_t start_frames
        [STIM_PROTOCOL_START_FRAME_COUNT][STIM_PROTOCOL_FRAME_BYTES];
    if (!stim_protocol_validate_start_frames() ||
        !stim_protocol_build_start_frames(start_frames)) {
        return ESP_ERR_INVALID_CRC;
    }

    stim_waveform_build_buffers(
        start_frames, s_stop_loop, s_start_sequence, s_enabled_idle);
    if (!stim_waveform_validate_buffers(start_frames,
                                        s_stop_loop,
                                        s_start_sequence,
                                        s_enabled_idle)) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    stim_waveform_configure_descriptors();
    s_waveform.event_callback = event_callback;
    s_waveform.event_user_data = user_data;

    result = stim_waveform_configure_lcd_cam();
    if (result != ESP_OK) {
        stim_waveform_enter_safe_state();
        return result;
    }
    stim_waveform_route_gpio_matrix();

    const gdma_channel_alloc_config_t channel_config = {
        .flags.isr_cache_safe = true,
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
        result = stim_waveform_install_interrupts();
    }
    if (result == ESP_OK) {
        result = gdma_start(
            s_waveform.dma_channel, (intptr_t)&s_stop_descriptor);
    }
    if (result != ESP_OK) {
        stim_waveform_enter_safe_state();
        stim_waveform_release_dma();
        return result;
    }

    esp_rom_delay_us(4);
    s_waveform.requested_enabled = false;
    s_waveform.state = STIM_WAVEFORM_STOP_LOOP;
    s_waveform.initialized = true;
    lcd_ll_start(s_waveform.lcd_hal.dev);
    esp_rom_delay_us(2);
    if (s_waveform.fatal) {
        stim_waveform_deinit();
        return ESP_FAIL;
    }
    ESP_LOGI(TAG,
             "LCD_CAM/GDMA running: SCLK=GPIO%d, mclkST=GPIO%d, "
             "MOSI=GPIO%d, CSb=GPIO%d, clock=%u Hz "
             "(PLL160 / (12 + 4/33) / 2)",
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
            s_enabled_descriptor.next = &s_enabled_descriptor;
            s_stop_descriptor.next = &s_start_descriptors[0];
            s_waveform.state = STIM_WAVEFORM_START_SEQUENCE;
            s_waveform.completed_start_frames = 0U;
        }
    } else {
        s_stop_descriptor.next = &s_stop_descriptor;
        s_enabled_descriptor.next = &s_stop_entry_descriptor;
        for (size_t index = 0U;
             index < STIM_START_DESCRIPTOR_COUNT;
             ++index) {
            s_start_descriptors[index].next = &s_stop_entry_descriptor;
        }
    }
    portEXIT_CRITICAL(&s_waveform.lock);
    return ESP_OK;
}

void stim_waveform_enter_safe_state(void)
{
    if (s_waveform.gdma_dev != NULL) {
        gdma_ll_tx_enable_interrupt(s_waveform.gdma_dev,
                                    s_waveform.gdma_channel_id,
                                    STIM_GDMA_INTERRUPT_MASK,
                                    false);
    }
    if (s_waveform.dma_channel != NULL) {
        (void)gdma_stop(s_waveform.dma_channel);
    }
    if (s_waveform.lcd_hal.dev != NULL) {
        PERIPH_RCC_ATOMIC() {
            lcd_ll_enable_interrupt(s_waveform.lcd_hal.dev,
                                    LCD_LL_EVENT_TRANS_DONE,
                                    false);
        }
        lcd_ll_stop(s_waveform.lcd_hal.dev);
    }
    (void)stim_waveform_configure_safe_gpio();
    esp_rom_gpio_connect_out_signal(
        STIM_MOSI_GPIO, SIG_GPIO_OUT_IDX, false, false);
    esp_rom_gpio_connect_out_signal(
        STIM_CSB_GPIO, SIG_GPIO_OUT_IDX, false, false);
    gpio_set_level(STIM_MOSI_GPIO, 1);
    gpio_set_level(STIM_CSB_GPIO, 1);
    s_waveform.fatal = true;
    s_waveform.state = STIM_WAVEFORM_FAULT;
}

void stim_waveform_deinit(void)
{
    stim_waveform_enter_safe_state();
    stim_waveform_release_dma();
    s_waveform.event_callback = NULL;
    s_waveform.event_user_data = NULL;
    s_waveform.initialized = false;
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
    status->fifo_underflow_errors = s_waveform.fifo_underflow_errors;
    status->unexpected_stop_errors = s_waveform.unexpected_stop_errors;
    portEXIT_CRITICAL(&s_waveform.lock);
}
