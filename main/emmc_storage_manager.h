#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "sdmmc_cmd.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EMMC_STORAGE_ERR_INTERLOCK ((esp_err_t)0x7101)
#define EMMC_STORAGE_ERR_BUSY      ((esp_err_t)0x7102)
#define EMMC_STORAGE_ERR_CARD      ((esp_err_t)0x7103)

typedef enum {
    EMMC_STATE_IDLE = 0,
    EMMC_STATE_WRITING,
    EMMC_STATE_FINALIZING,
    EMMC_STATE_READING,
    EMMC_STATE_ERROR,
} emmc_state_t;

typedef struct {
    emmc_state_t state;
    bool card_ready;
    bool gpio7_high;
    bool write_armed;
    bool result_available;
    uint64_t physical_bytes;
    uint64_t valid_bytes;
    uint64_t frame_count;
    uint64_t target_frames;
    uint64_t target_bytes;
    uint64_t raw_input_bytes;
    uint64_t dma_blocks;
    uint64_t dma_overruns;
    uint64_t verified_samples;
    uint64_t write_elapsed_us;
    uint64_t wall_elapsed_us;
    uint32_t capture_outcome;
    uint32_t sync_state;
    esp_err_t failure_code;
    uint64_t capacity_sectors;
    uint32_t emmc_clock_khz;
} emmc_status_t;

typedef struct {
    sdmmc_card_t *card;
    uint8_t *dma_buffer;
    size_t dma_buffer_sectors;
    esp_err_t card_error;
} emmc_storage_access_t;

typedef esp_err_t (*emmc_storage_read_operation_t)(
    emmc_storage_access_t *access, void *context);

esp_err_t emmc_storage_manager_init(void);
esp_err_t emmc_storage_get_status(emmc_status_t *status);
esp_err_t emmc_storage_execute_read(emmc_storage_read_operation_t operation,
                                    void *context);
esp_err_t emmc_storage_request_write_start(void);
esp_err_t emmc_storage_request_write_stop(void);
esp_err_t emmc_storage_request_reinit(void);
const char *emmc_storage_state_name(emmc_state_t state);

#ifdef __cplusplus
}
#endif
