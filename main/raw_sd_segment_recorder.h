#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "raw_sd_segment_format.h"
#include "sdmmc_cmd.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RAW_SD_RECORDER_WRITE_BUFFER_BYTES (64U * 1024U)

typedef struct {
    raw_sd_capture_outcome_t outcome;
    uint32_t final_sync_state;
    const raw_sd_sync_event_t *events;
    uint32_t event_count;
    uint32_t event_overflow;
    uint64_t resync_events;
    uint64_t resync_discarded_bytes;
    uint64_t last_resync_start_bit;
    uint64_t last_resync_lock_bit;
    uint64_t dma_last_sequence;
    uint32_t dma_sequence_gaps;
} raw_sd_segment_diagnostics_t;

typedef struct {
    sdmmc_card_t card;
    raw_sd_superblock_t superblock;
    raw_sd_segment_t active_segment;
    uint8_t *write_buffer;
    size_t write_buffer_used;
    uint64_t active_pending_valid_bytes;
    uint64_t next_metadata_bytes;
    uint32_t active_directory_lba;
    uint64_t next_write_lba;
    bool card_initialized;
    bool segment_open;
    bool run_full;
} raw_sd_recorder_t;

esp_err_t raw_sd_recorder_init(raw_sd_recorder_t *recorder);
void raw_sd_recorder_deinit(raw_sd_recorder_t *recorder);
esp_err_t raw_sd_recorder_begin_run(raw_sd_recorder_t *recorder);
esp_err_t raw_sd_recorder_open_segment(raw_sd_recorder_t *recorder);
esp_err_t raw_sd_recorder_append(raw_sd_recorder_t *recorder,
                                 const uint8_t *frames,
                                 size_t length,
                                 size_t *consumed);
esp_err_t raw_sd_recorder_close_segment(raw_sd_recorder_t *recorder,
                                        raw_sd_segment_state_t final_state,
                                        esp_err_t failure_code,
                                        const raw_sd_segment_diagnostics_t *diagnostics);
bool raw_sd_recorder_run_is_full(const raw_sd_recorder_t *recorder);

#ifdef __cplusplus
}
#endif
