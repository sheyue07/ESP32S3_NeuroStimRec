#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONTINUOUS_RX_CLK_GPIO       GPIO_NUM_20
#define CONTINUOUS_RX_DATA_GPIO      GPIO_NUM_16
#define CONTINUOUS_RX_CLOCK_HZ       30000000U
#define CONTINUOUS_RX_BLOCK_COUNT    4U
#define CONTINUOUS_RX_BLOCK_SIZE     32768U

typedef enum {
    CONTINUOUS_RX_ERROR_NONE,
    CONTINUOUS_RX_ERROR_NOT_INITIALIZED,
    CONTINUOUS_RX_ERROR_ALREADY_RUNNING,
    CONTINUOUS_RX_ERROR_DMA_DESCRIPTOR,
    CONTINUOUS_RX_ERROR_DMA_OVERRUN,
    CONTINUOUS_RX_ERROR_QUEUE_FULL,
    CONTINUOUS_RX_ERROR_UNEXPECTED_EOF,
    CONTINUOUS_RX_ERROR_BLOCK_STATE,
} continuous_rx_error_t;

typedef struct {
    uint8_t index;
    const uint8_t *data;
    size_t length;
    uint64_t sequence;
} continuous_rx_block_t;

typedef struct {
    uint64_t completed_descriptors;
    uint64_t completed_blocks;
    uint64_t descriptor_errors;
    uint64_t overruns;
    uint64_t queue_full_errors;
    uint64_t unexpected_eof_errors;
    continuous_rx_error_t first_error;
    bool running;
    bool fatal;
} continuous_rx_stats_t;

esp_err_t continuous_rx_init(void);
esp_err_t continuous_rx_start(void);
esp_err_t continuous_rx_stop(void);
esp_err_t continuous_rx_deinit(void);

esp_err_t continuous_rx_receive_block(continuous_rx_block_t *block,
                                      TickType_t ticks_to_wait);
esp_err_t continuous_rx_release_block(uint8_t block_index);
void continuous_rx_get_stats(continuous_rx_stats_t *stats);
bool continuous_rx_has_fatal_error(void);

#ifdef __cplusplus
}
#endif
