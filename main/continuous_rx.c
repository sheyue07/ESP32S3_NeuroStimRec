#include "continuous_rx.h"

#include <string.h>

#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_private/gdma.h"
#include "esp_private/spi_common_internal.h"
#include "esp_rom_gpio.h"
#include "freertos/queue.h"
#include "hal/dma_types.h"
#include "hal/spi_ll.h"
#include "hal/spi_slave_hal.h"
#include "soc/gpio_pins.h"
#include "soc/gpio_sig_map.h"

#define CONTINUOUS_RX_DESC_DATA_SIZE DMA_DESCRIPTOR_BUFFER_MAX_SIZE_4B_ALIGNED
#define CONTINUOUS_RX_DESCS_PER_BLOCK \
    ((CONTINUOUS_RX_BLOCK_SIZE + CONTINUOUS_RX_DESC_DATA_SIZE - 1U) / \
     CONTINUOUS_RX_DESC_DATA_SIZE)
#define CONTINUOUS_RX_DESC_COUNT \
    (CONTINUOUS_RX_BLOCK_COUNT * CONTINUOUS_RX_DESCS_PER_BLOCK)

typedef enum {
    RX_BLOCK_DMA_OWNED,
    RX_BLOCK_CPU_READY,
    RX_BLOCK_CPU_PROCESSING,
} rx_block_state_t;

typedef struct {
    uint8_t block_index;
    uint64_t sequence;
} completed_rx_item_t;

typedef struct {
    bool initialized;
    bool bus_claimed;
    volatile bool running;
    volatile bool fatal;
    volatile continuous_rx_error_t first_error;
    spi_slave_hal_context_t spi_hal;
    gdma_channel_handle_t dma_channel;
    dma_descriptor_align4_t *descriptors;
    uint8_t *blocks[CONTINUOUS_RX_BLOCK_COUNT];
    volatile rx_block_state_t block_states[CONTINUOUS_RX_BLOCK_COUNT];
    QueueHandle_t completed_queue;
    StaticQueue_t completed_queue_storage;
    uint8_t completed_queue_items[
        CONTINUOUS_RX_BLOCK_COUNT * sizeof(completed_rx_item_t)];
    volatile uint32_t descriptor_position;
    volatile uint64_t block_sequence;
    volatile uint64_t completed_descriptors;
    volatile uint64_t completed_blocks;
    volatile uint64_t descriptor_errors;
    volatile uint64_t overruns;
    volatile uint64_t queue_full_errors;
    volatile uint64_t unexpected_eof_errors;
    portMUX_TYPE lock;
} continuous_rx_context_t;

static continuous_rx_context_t s_rx = {
    .lock = portMUX_INITIALIZER_UNLOCKED,
};

static void IRAM_ATTR continuous_rx_fail_isr(continuous_rx_error_t error)
{
    portENTER_CRITICAL_ISR(&s_rx.lock);
    if (!s_rx.fatal) {
        s_rx.first_error = error;
    }
    s_rx.fatal = true;
    s_rx.running = false;
    portEXIT_CRITICAL_ISR(&s_rx.lock);
}

static void continuous_rx_fail(continuous_rx_error_t error)
{
    portENTER_CRITICAL(&s_rx.lock);
    if (!s_rx.fatal) {
        s_rx.first_error = error;
    }
    s_rx.fatal = true;
    s_rx.running = false;
    portEXIT_CRITICAL(&s_rx.lock);
}

static bool IRAM_ATTR continuous_rx_on_descriptor_error(
    gdma_channel_handle_t dma_channel,
    gdma_event_data_t *event_data,
    void *user_data)
{
    (void)dma_channel;
    (void)event_data;
    (void)user_data;
    ++s_rx.descriptor_errors;
    continuous_rx_fail_isr(CONTINUOUS_RX_ERROR_DMA_DESCRIPTOR);
    return false;
}

static bool IRAM_ATTR continuous_rx_on_recv_eof(
    gdma_channel_handle_t dma_channel,
    gdma_event_data_t *event_data,
    void *user_data)
{
    (void)dma_channel;
    (void)event_data;
    (void)user_data;
    if (s_rx.running) {
        ++s_rx.unexpected_eof_errors;
        continuous_rx_fail_isr(CONTINUOUS_RX_ERROR_UNEXPECTED_EOF);
    }
    return false;
}

static bool IRAM_ATTR continuous_rx_on_descriptor_done(
    gdma_channel_handle_t dma_channel,
    gdma_event_data_t *event_data,
    void *user_data)
{
    (void)dma_channel;
    (void)event_data;
    (void)user_data;

    BaseType_t task_woken = pdFALSE;
    const uint32_t descriptor_position = s_rx.descriptor_position + 1U;
    s_rx.descriptor_position = descriptor_position % CONTINUOUS_RX_DESC_COUNT;
    ++s_rx.completed_descriptors;

    if ((descriptor_position % CONTINUOUS_RX_DESCS_PER_BLOCK) != 0U) {
        return false;
    }

    const uint8_t completed_block = (uint8_t)(
        (descriptor_position / CONTINUOUS_RX_DESCS_PER_BLOCK - 1U) %
        CONTINUOUS_RX_BLOCK_COUNT);
    const uint8_t next_block =
        (uint8_t)((completed_block + 1U) % CONTINUOUS_RX_BLOCK_COUNT);

    if (s_rx.block_states[next_block] != RX_BLOCK_DMA_OWNED ||
        s_rx.block_states[completed_block] != RX_BLOCK_DMA_OWNED) {
        ++s_rx.overruns;
        continuous_rx_fail_isr(CONTINUOUS_RX_ERROR_DMA_OVERRUN);
        return false;
    }

    s_rx.block_states[completed_block] = RX_BLOCK_CPU_READY;
    ++s_rx.completed_blocks;
    const completed_rx_item_t completed_item = {
        .block_index = completed_block,
        .sequence = ++s_rx.block_sequence,
    };
    if (xQueueSendFromISR(s_rx.completed_queue, &completed_item, &task_woken) != pdTRUE) {
        ++s_rx.queue_full_errors;
        continuous_rx_fail_isr(CONTINUOUS_RX_ERROR_QUEUE_FULL);
        return false;
    }

    return task_woken == pdTRUE;
}

static void continuous_rx_route_cs(bool active)
{
    esp_rom_gpio_connect_in_signal(
        active ? GPIO_MATRIX_CONST_ZERO_INPUT : GPIO_MATRIX_CONST_ONE_INPUT,
        FSPICS0_IN_IDX,
        false);
}

static void continuous_rx_configure_descriptors(void)
{
    size_t descriptor_index = 0;
    for (size_t block = 0; block < CONTINUOUS_RX_BLOCK_COUNT; ++block) {
        size_t remaining = CONTINUOUS_RX_BLOCK_SIZE;
        size_t offset = 0;
        for (size_t node = 0; node < CONTINUOUS_RX_DESCS_PER_BLOCK; ++node) {
            const size_t node_size = remaining > CONTINUOUS_RX_DESC_DATA_SIZE
                                         ? CONTINUOUS_RX_DESC_DATA_SIZE
                                         : remaining;
            dma_descriptor_align4_t *descriptor =
                &s_rx.descriptors[descriptor_index];
            memset(descriptor, 0, sizeof(*descriptor));
            descriptor->dw0.size = node_size;
            descriptor->dw0.length = 0;
            descriptor->dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
            descriptor->buffer = s_rx.blocks[block] + offset;
            descriptor->next = &s_rx.descriptors[
                (descriptor_index + 1U) % CONTINUOUS_RX_DESC_COUNT];
            remaining -= node_size;
            offset += node_size;
            ++descriptor_index;
        }
    }
}

static void continuous_rx_reset_runtime_state(void)
{
    xQueueReset(s_rx.completed_queue);
    s_rx.descriptor_position = 0;
    s_rx.block_sequence = 0;
    s_rx.completed_descriptors = 0;
    s_rx.completed_blocks = 0;
    s_rx.descriptor_errors = 0;
    s_rx.overruns = 0;
    s_rx.queue_full_errors = 0;
    s_rx.unexpected_eof_errors = 0;
    s_rx.first_error = CONTINUOUS_RX_ERROR_NONE;
    s_rx.fatal = false;
    for (size_t block = 0; block < CONTINUOUS_RX_BLOCK_COUNT; ++block) {
        s_rx.block_states[block] = RX_BLOCK_DMA_OWNED;
    }
    continuous_rx_configure_descriptors();
}

esp_err_t continuous_rx_init(void)
{
    if (s_rx.initialized) {
        return ESP_OK;
    }

    esp_err_t result = spicommon_bus_alloc(SPI2_HOST, "continuous_rx");
    if (result != ESP_OK) {
        return result;
    }
    s_rx.bus_claimed = true;

    const gpio_config_t input_config = {
        .pin_bit_mask = (UINT64_C(1) << CONTINUOUS_RX_CLK_GPIO) |
                        (UINT64_C(1) << CONTINUOUS_RX_DATA_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    result = gpio_config(&input_config);
    if (result != ESP_OK) {
        spicommon_bus_free(SPI2_HOST);
        s_rx.bus_claimed = false;
        return result;
    }

    esp_rom_gpio_connect_in_signal(CONTINUOUS_RX_CLK_GPIO, FSPICLK_IN_IDX, false);
    esp_rom_gpio_connect_in_signal(CONTINUOUS_RX_DATA_GPIO, FSPID_IN_IDX, false);
    continuous_rx_route_cs(false);

    spi_slave_hal_config_t hal_config = {
        .host_id = SPI2_HOST,
    };
    spi_slave_hal_init(&s_rx.spi_hal, &hal_config);
    /* FPGA updates ADC_Data on the falling edge, so sample on rising edge. */
    s_rx.spi_hal.mode = 0;
    s_rx.spi_hal.rx_lsbfirst = 0;
    s_rx.spi_hal.tx_lsbfirst = 0;
    s_rx.spi_hal.use_dma = 1;
    spi_slave_hal_setup_device(&s_rx.spi_hal);
    spi_ll_disable_int(s_rx.spi_hal.hw);
    spi_ll_enable_mosi(s_rx.spi_hal.hw, true);
    spi_ll_enable_miso(s_rx.spi_hal.hw, false);

    gdma_channel_alloc_config_t dma_config = {
        .flags.isr_cache_safe = true,
    };
    result = gdma_new_ahb_channel(&dma_config, NULL, &s_rx.dma_channel);
    if (result != ESP_OK) {
        spicommon_bus_free(SPI2_HOST);
        s_rx.bus_claimed = false;
        return result;
    }
    result = gdma_connect(
        s_rx.dma_channel, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_SPI, 2));
    if (result != ESP_OK) {
        gdma_del_channel(s_rx.dma_channel);
        s_rx.dma_channel = NULL;
        spicommon_bus_free(SPI2_HOST);
        s_rx.bus_claimed = false;
        return result;
    }

    const gdma_strategy_config_t strategy = {
        .owner_check = false,
        .auto_update_desc = false,
        .eof_till_data_popped = true,
    };
    result = gdma_apply_strategy(s_rx.dma_channel, &strategy);
    if (result == ESP_OK) {
        const gdma_transfer_config_t transfer = {
            .max_data_burst_size = 32,
            .access_ext_mem = false,
        };
        result = gdma_config_transfer(s_rx.dma_channel, &transfer);
    }
    if (result != ESP_OK) {
        gdma_disconnect(s_rx.dma_channel);
        gdma_del_channel(s_rx.dma_channel);
        s_rx.dma_channel = NULL;
        spicommon_bus_free(SPI2_HOST);
        s_rx.bus_claimed = false;
        return result;
    }

    s_rx.descriptors = heap_caps_calloc(
        CONTINUOUS_RX_DESC_COUNT,
        sizeof(dma_descriptor_align4_t),
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    for (size_t block = 0; block < CONTINUOUS_RX_BLOCK_COUNT; ++block) {
        s_rx.blocks[block] = heap_caps_malloc(
            CONTINUOUS_RX_BLOCK_SIZE,
            MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    }
    if (s_rx.descriptors == NULL || s_rx.blocks[0] == NULL ||
        s_rx.blocks[1] == NULL || s_rx.blocks[2] == NULL ||
        s_rx.blocks[3] == NULL) {
        continuous_rx_deinit();
        return ESP_ERR_NO_MEM;
    }

    s_rx.completed_queue = xQueueCreateStatic(
        CONTINUOUS_RX_BLOCK_COUNT,
        sizeof(completed_rx_item_t),
        s_rx.completed_queue_items,
        &s_rx.completed_queue_storage);
    if (s_rx.completed_queue == NULL) {
        continuous_rx_deinit();
        return ESP_ERR_NO_MEM;
    }

    gdma_rx_event_callbacks_t callbacks = {
        .on_recv_eof = continuous_rx_on_recv_eof,
        .on_descr_err = continuous_rx_on_descriptor_error,
        .on_recv_done = continuous_rx_on_descriptor_done,
    };
    result = gdma_register_rx_event_callbacks(
        s_rx.dma_channel, &callbacks, &s_rx);
    if (result != ESP_OK) {
        continuous_rx_deinit();
        return result;
    }

    continuous_rx_reset_runtime_state();
    s_rx.initialized = true;
    return ESP_OK;
}

esp_err_t continuous_rx_start(void)
{
    if (!s_rx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_rx.running) {
        return ESP_ERR_INVALID_STATE;
    }

    continuous_rx_route_cs(false);
    continuous_rx_reset_runtime_state();
    spi_slave_hal_hw_reset(&s_rx.spi_hal);
    spi_slave_hal_setup_device(&s_rx.spi_hal);
    spi_ll_disable_int(s_rx.spi_hal.hw);
    spi_ll_enable_mosi(s_rx.spi_hal.hw, true);
    spi_ll_enable_miso(s_rx.spi_hal.hw, false);
    spi_slave_hal_hw_prepare_rx(s_rx.spi_hal.hw);

    esp_err_t result = gdma_reset(s_rx.dma_channel);
    if (result == ESP_OK) {
        result = gdma_start(
            s_rx.dma_channel, (intptr_t)&s_rx.descriptors[0]);
    }
    if (result != ESP_OK) {
        return result;
    }

    s_rx.running = true;
    spi_slave_hal_user_start(&s_rx.spi_hal);
    continuous_rx_route_cs(true);
    return ESP_OK;
}

esp_err_t continuous_rx_stop(void)
{
    if (!s_rx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_rx.running = false;
    continuous_rx_route_cs(false);
    spi_ll_dma_rx_enable(s_rx.spi_hal.hw, false);
    const esp_err_t result = gdma_stop(s_rx.dma_channel);
    /* Keep already completed blocks queued. The capture task drains them
     * before acknowledging stop; only the in-progress partial block is lost. */
    return result;
}

esp_err_t continuous_rx_receive_block(continuous_rx_block_t *block,
                                      TickType_t ticks_to_wait)
{
    if (!s_rx.initialized || block == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    completed_rx_item_t item = {0};
    if (xQueueReceive(s_rx.completed_queue, &item, ticks_to_wait) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    const uint8_t block_index = item.block_index;
    if (block_index >= CONTINUOUS_RX_BLOCK_COUNT ||
        s_rx.block_states[block_index] != RX_BLOCK_CPU_READY) {
        continuous_rx_fail(CONTINUOUS_RX_ERROR_BLOCK_STATE);
        return ESP_ERR_INVALID_STATE;
    }

    s_rx.block_states[block_index] = RX_BLOCK_CPU_PROCESSING;
    block->index = block_index;
    block->data = s_rx.blocks[block_index];
    block->length = CONTINUOUS_RX_BLOCK_SIZE;
    block->sequence = item.sequence;
    return ESP_OK;
}

esp_err_t continuous_rx_release_block(uint8_t block_index)
{
    if (!s_rx.initialized || block_index >= CONTINUOUS_RX_BLOCK_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_rx.block_states[block_index] != RX_BLOCK_CPU_PROCESSING) {
        continuous_rx_fail(CONTINUOUS_RX_ERROR_BLOCK_STATE);
        return ESP_ERR_INVALID_STATE;
    }
    s_rx.block_states[block_index] = RX_BLOCK_DMA_OWNED;
    return ESP_OK;
}

void continuous_rx_get_stats(continuous_rx_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_rx.lock);
    *stats = (continuous_rx_stats_t) {
        .completed_descriptors = s_rx.completed_descriptors,
        .completed_blocks = s_rx.completed_blocks,
        .descriptor_errors = s_rx.descriptor_errors,
        .overruns = s_rx.overruns,
        .queue_full_errors = s_rx.queue_full_errors,
        .unexpected_eof_errors = s_rx.unexpected_eof_errors,
        .first_error = s_rx.first_error,
        .running = s_rx.running,
        .fatal = s_rx.fatal,
    };
    portEXIT_CRITICAL(&s_rx.lock);
}

bool continuous_rx_has_fatal_error(void)
{
    return s_rx.fatal;
}

esp_err_t continuous_rx_deinit(void)
{
    if (s_rx.running) {
        continuous_rx_stop();
    }
    continuous_rx_route_cs(false);

    if (s_rx.dma_channel != NULL) {
        gdma_disconnect(s_rx.dma_channel);
        gdma_del_channel(s_rx.dma_channel);
        s_rx.dma_channel = NULL;
    }
    for (size_t block = 0; block < CONTINUOUS_RX_BLOCK_COUNT; ++block) {
        heap_caps_free(s_rx.blocks[block]);
        s_rx.blocks[block] = NULL;
    }
    heap_caps_free(s_rx.descriptors);
    s_rx.descriptors = NULL;

    if (s_rx.bus_claimed) {
        spicommon_bus_free(SPI2_HOST);
        s_rx.bus_claimed = false;
    }
    s_rx.initialized = false;
    s_rx.completed_queue = NULL;
    return ESP_OK;
}
