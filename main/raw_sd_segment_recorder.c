#include "raw_sd_segment_recorder.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "frame_sync.h"
#include "sdkconfig.h"

static const char *TAG = "RAW_EMMC_REC";

/* ESP32-S3 native SDMMC 4-bit connection to the on-board eMMC. */
#define EMMC_PIN_CLK ((gpio_num_t)CONFIG_EMMC_CTRL_EMMC_CLK_GPIO)
#define EMMC_PIN_CMD ((gpio_num_t)CONFIG_EMMC_CTRL_EMMC_CMD_GPIO)
#define EMMC_PIN_D0  ((gpio_num_t)CONFIG_EMMC_CTRL_EMMC_D0_GPIO)
#define EMMC_PIN_D1  ((gpio_num_t)CONFIG_EMMC_CTRL_EMMC_D1_GPIO)
#define EMMC_PIN_D2  ((gpio_num_t)CONFIG_EMMC_CTRL_EMMC_D2_GPIO)
#define EMMC_PIN_D3  ((gpio_num_t)CONFIG_EMMC_CTRL_EMMC_D3_GPIO)
#define EMMC_MAX_FREQ_KHZ CONFIG_EMMC_CTRL_EMMC_MAX_FREQ_KHZ
#define RAW_SD_METADATA_INTERVAL_BYTES (UINT64_C(64) * 1024U * 1024U)
#define RAW_SD_RECORDER_SECTORS_PER_WRITE \
    (RAW_SD_RECORDER_WRITE_BUFFER_BYTES / RAW_SD_SECTOR_BYTES)

_Static_assert(RAW_SD_RECORDER_WRITE_BUFFER_BYTES % RAW_SD_SECTOR_BYTES == 0U,
               "write buffer must contain whole sectors");
_Static_assert(RAW_SD_FRAME_BYTES == ADC_FRAME_SIZE_BYTES,
               "raw eMMC frame size must match the stream parser");

static esp_err_t write_superblocks(raw_sd_recorder_t *recorder)
{
    recorder->superblock.generation++;
    recorder->superblock.next_write_lba = recorder->next_write_lba;
    recorder->superblock.last_update_time_us = (uint64_t)esp_timer_get_time();
    raw_sd_superblock_finalize(&recorder->superblock);
    esp_err_t result = sdmmc_write_sectors(&recorder->card, &recorder->superblock,
                                           RAW_SD_SUPERBLOCK_LBA_A, 1U);
    if (result != ESP_OK) {
        return result;
    }
    return sdmmc_write_sectors(&recorder->card, &recorder->superblock,
                               RAW_SD_SUPERBLOCK_LBA_B, 1U);
}

static esp_err_t write_active_segment(raw_sd_recorder_t *recorder)
{
    raw_sd_segment_finalize(&recorder->active_segment);
    return sdmmc_write_sectors(&recorder->card, &recorder->active_segment,
                               recorder->active_directory_lba, 1U);
}

static esp_err_t write_data_block(raw_sd_recorder_t *recorder,
                                  const uint8_t *data, size_t bytes)
{
    if (bytes == 0U || bytes % RAW_SD_SECTOR_BYTES != 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint64_t sectors = bytes / RAW_SD_SECTOR_BYTES;
    const uint64_t data_end_lba = RAW_SD_DATA_START_LBA + RAW_SD_DATA_CAPACITY_SECTORS;
    if (recorder->next_write_lba + sectors > data_end_lba) {
        return ESP_ERR_INVALID_SIZE;
    }
    const esp_err_t result = sdmmc_write_sectors(&recorder->card, data,
                                                  (size_t)recorder->next_write_lba,
                                                  (size_t)sectors);
    if (result != ESP_OK) {
        return result;
    }
    recorder->next_write_lba += sectors;
    recorder->superblock.physical_bytes_written += bytes;
    recorder->active_segment.physical_bytes += bytes;
    if (recorder->segment_open && recorder->next_metadata_bytes != 0U &&
        recorder->superblock.physical_bytes_written >=
            recorder->next_metadata_bytes) {
        recorder->active_segment.valid_bytes =
            recorder->active_pending_valid_bytes;
        recorder->active_segment.frame_count =
            recorder->active_segment.valid_bytes / RAW_SD_FRAME_BYTES;
        recorder->active_segment.metadata_generation =
            recorder->superblock.generation + 1U;
        esp_err_t checkpoint_result = write_active_segment(recorder);
        if (checkpoint_result == ESP_OK) {
            checkpoint_result = write_superblocks(recorder);
        }
        if (checkpoint_result != ESP_OK) {
            /* The data write already completed and next_write_lba advanced.
             * Do not let close_segment write the same block a second time. */
            recorder->write_buffer_used = 0U;
            return checkpoint_result;
        }
        do {
            recorder->next_metadata_bytes += RAW_SD_METADATA_INTERVAL_BYTES;
        } while (recorder->next_metadata_bytes <=
                 recorder->superblock.physical_bytes_written);
    }
    return ESP_OK;
}

static esp_err_t flush_full_buffer(raw_sd_recorder_t *recorder)
{
    if (recorder->write_buffer_used != RAW_SD_RECORDER_WRITE_BUFFER_BYTES) {
        return ESP_OK;
    }
    const esp_err_t result = write_data_block(recorder, recorder->write_buffer,
                                               recorder->write_buffer_used);
    if (result == ESP_OK) {
        recorder->write_buffer_used = 0U;
    }
    return result;
}

static esp_err_t flush_tail(raw_sd_recorder_t *recorder)
{
    if (recorder->write_buffer_used == 0U) {
        return ESP_OK;
    }
    const size_t physical_bytes = (recorder->write_buffer_used +
                                   RAW_SD_SECTOR_BYTES - 1U) /
        RAW_SD_SECTOR_BYTES * RAW_SD_SECTOR_BYTES;
    memset(recorder->write_buffer + recorder->write_buffer_used, 0,
           physical_bytes - recorder->write_buffer_used);
    const esp_err_t result = write_data_block(recorder, recorder->write_buffer,
                                               physical_bytes);
    if (result == ESP_OK) {
        recorder->write_buffer_used = 0U;
    }
    return result;
}

esp_err_t raw_sd_recorder_init(raw_sd_recorder_t *recorder)
{
    if (recorder == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(recorder, 0, sizeof(*recorder));
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = EMMC_MAX_FREQ_KHZ;
    host.command_timeout_ms = 3000;
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;
    slot_config.clk = EMMC_PIN_CLK;
    slot_config.cmd = EMMC_PIN_CMD;
    slot_config.d0 = EMMC_PIN_D0;
    slot_config.d1 = EMMC_PIN_D1;
    slot_config.d2 = EMMC_PIN_D2;
    slot_config.d3 = EMMC_PIN_D3;
    slot_config.cd = GPIO_NUM_NC;
    slot_config.wp = GPIO_NUM_NC;
    ESP_LOGI(TAG,
             "eMMC SDMMC pins: CLK=GPIO%d CMD=GPIO%d D0=GPIO%d D1=GPIO%d "
             "D2=GPIO%d D3=GPIO%d, width=4, limit=%d kHz",
             EMMC_PIN_CLK, EMMC_PIN_CMD, EMMC_PIN_D0, EMMC_PIN_D1,
             EMMC_PIN_D2, EMMC_PIN_D3, EMMC_MAX_FREQ_KHZ);
    esp_err_t result = sdmmc_host_init();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "SDMMC host initialization failed: 0x%x (%s)",
                 (unsigned int)result, esp_err_to_name(result));
        return result;
    }
    result = sdmmc_host_init_slot(host.slot, &slot_config);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "SDMMC eMMC slot initialization failed: 0x%x (%s)",
                 (unsigned int)result, esp_err_to_name(result));
        (void)sdmmc_host_deinit();
        return result;
    }
    result = sdmmc_card_init(&host, &recorder->card);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "eMMC initialization failed: 0x%x (%s)",
                 (unsigned int)result, esp_err_to_name(result));
        (void)sdmmc_host_deinit();
        return result;
    }
    if (!recorder->card.is_mmc) {
        ESP_LOGE(TAG,
                 "The detected device is not MMC/eMMC; refusing raw writes");
        (void)sdmmc_host_deinit();
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (recorder->card.csd.sector_size != RAW_SD_SECTOR_BYTES) {
        ESP_LOGE(TAG, "Unsupported eMMC sector size: %d bytes",
                 recorder->card.csd.sector_size);
        (void)sdmmc_host_deinit();
        return ESP_ERR_NOT_SUPPORTED;
    }
    const uint64_t required_sectors = RAW_SD_DATA_START_LBA + RAW_SD_DATA_CAPACITY_SECTORS;
    if ((uint64_t)recorder->card.csd.capacity < required_sectors) {
        ESP_LOGE(TAG, "eMMC user area too small: sectors=%d required=%" PRIu64,
                 recorder->card.csd.capacity, required_sectors);
        (void)sdmmc_host_deinit();
        return ESP_ERR_INVALID_SIZE;
    }
    recorder->write_buffer = heap_caps_malloc(RAW_SD_RECORDER_WRITE_BUFFER_BYTES,
                                               MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA |
                                               MALLOC_CAP_8BIT);
    if (recorder->write_buffer == NULL) {
        (void)sdmmc_host_deinit();
        return ESP_ERR_NO_MEM;
    }
    recorder->card_initialized = true;
    ESP_LOGI(TAG, "eMMC ready: 4-bit SDMMC, actual clock %.2f MHz",
             (double)recorder->card.real_freq_khz / 1000.0);
    return ESP_OK;
}

void raw_sd_recorder_deinit(raw_sd_recorder_t *recorder)
{
    if (recorder == NULL) {
        return;
    }
    heap_caps_free(recorder->write_buffer);
    recorder->write_buffer = NULL;
    if (recorder->card_initialized) {
        (void)sdmmc_host_deinit();
    }
    memset(recorder, 0, sizeof(*recorder));
}

esp_err_t raw_sd_recorder_begin_run(raw_sd_recorder_t *recorder)
{
    if (recorder == NULL || !recorder->card_initialized || recorder->segment_open) {
        return ESP_ERR_INVALID_STATE;
    }
    memset(&recorder->superblock, 0, sizeof(recorder->superblock));
    recorder->superblock.state = RAW_SD_RUN_RUNNING;
    recorder->superblock.run_id = 1U;
    recorder->superblock.data_start_lba = RAW_SD_DATA_START_LBA;
    recorder->superblock.data_capacity_sectors = RAW_SD_DATA_CAPACITY_SECTORS;
    recorder->superblock.directory_capacity =
        RAW_SD_SEGMENT_DIRECTORY_CAPACITY;
    recorder->superblock.event_area_start_lba = RAW_SD_EVENT_AREA_START_LBA;
    recorder->superblock.next_event_lba = RAW_SD_EVENT_AREA_START_LBA;
    recorder->superblock.start_time_us = (uint64_t)esp_timer_get_time();
    recorder->next_write_lba = RAW_SD_DATA_START_LBA;
    recorder->write_buffer_used = 0U;
    recorder->run_full = false;
    ESP_LOGI(TAG,
             "Startup pre-erase disabled; new eMMC run overwrites from LBA%" PRIu32,
             RAW_SD_DATA_START_LBA);
    return write_superblocks(recorder);
}

esp_err_t raw_sd_recorder_open_segment(raw_sd_recorder_t *recorder)
{
    if (recorder == NULL || !recorder->card_initialized || recorder->segment_open ||
        recorder->run_full) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint32_t directory_limit = RAW_SD_SEGMENT_DIRECTORY_CAPACITY;
    if (recorder->superblock.segment_count >= directory_limit) {
        return ESP_ERR_NO_MEM;
    }
    memset(&recorder->active_segment, 0, sizeof(recorder->active_segment));
    recorder->active_segment.state = RAW_SD_SEGMENT_OPEN;
    recorder->active_segment.segment_id = recorder->superblock.segment_count + 1U;
    recorder->active_segment.run_id = recorder->superblock.run_id;
    recorder->active_segment.start_lba = recorder->next_write_lba;
    recorder->active_segment.start_time_us = (uint64_t)esp_timer_get_time();
    recorder->active_segment.metadata_generation =
        recorder->superblock.generation + 1U;
    recorder->active_directory_lba = RAW_SD_SEGMENT_DIRECTORY_START_LBA +
        recorder->superblock.segment_count;
    esp_err_t result = write_active_segment(recorder);
    if (result != ESP_OK) {
        return result;
    }
    recorder->superblock.segment_count++;
    result = write_superblocks(recorder);
    if (result != ESP_OK) {
        /* The directory entry is deliberately not committed without its
         * superblock count. A later start safely overwrites this same LBA. */
        recorder->superblock.segment_count--;
        return result;
    }
    recorder->segment_open = true;
    recorder->next_metadata_bytes =
        recorder->superblock.physical_bytes_written +
        RAW_SD_METADATA_INTERVAL_BYTES;
    return ESP_OK;
}

esp_err_t raw_sd_recorder_append(raw_sd_recorder_t *recorder,
                                 const uint8_t *frames, size_t length,
                                 size_t *consumed)
{
    if (consumed != NULL) {
        *consumed = 0U;
    }
    if (recorder == NULL || frames == NULL || !recorder->segment_open ||
        length % RAW_SD_FRAME_BYTES != 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    /* Capacity must be based on physical bytes. Every closed segment can add
     * up to one sector of tail padding, so counting valid frame bytes alone
     * can overrun the fixed 1 GiB raw eMMC data area after many segments. */
    const uint64_t capacity_bytes =
        (uint64_t)RAW_SD_DATA_CAPACITY_SECTORS * RAW_SD_SECTOR_BYTES;
    if (recorder->superblock.physical_bytes_written > capacity_bytes ||
        recorder->write_buffer_used >
            capacity_bytes - recorder->superblock.physical_bytes_written) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint64_t physically_available =
        capacity_bytes - recorder->superblock.physical_bytes_written -
        recorder->write_buffer_used;
    const uint64_t remaining = physically_available /
        RAW_SD_FRAME_BYTES * RAW_SD_FRAME_BYTES;
    const size_t accepted = length > remaining ? (size_t)remaining : length;
    size_t offset = 0U;
    while (offset < accepted) {
        const size_t available = RAW_SD_RECORDER_WRITE_BUFFER_BYTES - recorder->write_buffer_used;
        const size_t copy_bytes = accepted - offset < available ? accepted - offset : available;
        memcpy(recorder->write_buffer + recorder->write_buffer_used, frames + offset, copy_bytes);
        recorder->write_buffer_used += copy_bytes;
        offset += copy_bytes;
        recorder->active_pending_valid_bytes += copy_bytes;
        if (consumed != NULL) {
            *consumed += copy_bytes;
        }
        const esp_err_t result = flush_full_buffer(recorder);
        if (result != ESP_OK) {
            return result;
        }
    }
    if (accepted != length) {
        recorder->run_full = true;
        return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
}

static void apply_segment_diagnostics(
    raw_sd_recorder_t *recorder,
    const raw_sd_segment_diagnostics_t *diagnostics,
    raw_sd_segment_state_t final_state)
{
    raw_sd_segment_t *const segment = &recorder->active_segment;
    segment->capture_outcome = diagnostics != NULL
                                   ? (uint32_t)diagnostics->outcome
                                   : (uint32_t)(
                                         final_state == RAW_SD_SEGMENT_CLOSED
                                             ? RAW_SD_CAPTURE_CLEAN
                                             : RAW_SD_CAPTURE_FAILED_PIPELINE);
    if (diagnostics == NULL) {
        return;
    }
    segment->final_sync_state = diagnostics->final_sync_state;
    segment->event_overflow = diagnostics->event_overflow;
    segment->resync_events = diagnostics->resync_events;
    segment->resync_discarded_bytes = diagnostics->resync_discarded_bytes;
    segment->last_resync_start_bit = diagnostics->last_resync_start_bit;
    segment->last_resync_lock_bit = diagnostics->last_resync_lock_bit;
    segment->dma_last_sequence = diagnostics->dma_last_sequence;
    segment->dma_sequence_gaps = diagnostics->dma_sequence_gaps;
}

static esp_err_t write_sync_events(
    raw_sd_recorder_t *recorder,
    const raw_sd_segment_diagnostics_t *diagnostics)
{
    if (diagnostics == NULL || diagnostics->events == NULL ||
        diagnostics->event_count == 0U) {
        return ESP_OK;
    }
    const uint32_t available_sectors =
        RAW_SD_EVENT_AREA_END_LBA - recorder->superblock.next_event_lba;
    const uint32_t capacity = available_sectors * RAW_SD_EVENTS_PER_SECTOR;
    const uint32_t stored_events = diagnostics->event_count < capacity
                                       ? diagnostics->event_count
                                       : capacity;
    const uint32_t sectors = (stored_events + RAW_SD_EVENTS_PER_SECTOR - 1U) /
        RAW_SD_EVENTS_PER_SECTOR;
    recorder->active_segment.event_overflow +=
        diagnostics->event_count - stored_events;
    if (stored_events == 0U) {
        recorder->superblock.event_records_overflow +=
            recorder->active_segment.event_overflow;
        return ESP_OK;
    }
    recorder->active_segment.event_start_lba =
        recorder->superblock.next_event_lba;
    recorder->active_segment.event_sector_count = sectors;
    recorder->active_segment.event_count = stored_events;

    uint32_t copied = 0U;
    for (uint32_t sector_index = 0U; sector_index < sectors; ++sector_index) {
        raw_sd_event_sector_t *const sector =
            (raw_sd_event_sector_t *)recorder->write_buffer;
        memset(sector, 0, sizeof(*sector));
        sector->segment_id = recorder->active_segment.segment_id;
        sector->sector_index = sector_index;
        const uint32_t remaining = stored_events - copied;
        sector->record_count = remaining < RAW_SD_EVENTS_PER_SECTOR
                                   ? remaining
                                   : RAW_SD_EVENTS_PER_SECTOR;
        memcpy(sector->events, diagnostics->events + copied,
               sector->record_count * sizeof(raw_sd_sync_event_t));
        raw_sd_event_sector_finalize(sector);
        const esp_err_t result = sdmmc_write_sectors(
            &recorder->card, sector,
            recorder->superblock.next_event_lba + sector_index, 1U);
        if (result != ESP_OK) {
            recorder->active_segment.event_start_lba = 0U;
            recorder->active_segment.event_sector_count = 0U;
            recorder->active_segment.event_count = 0U;
            recorder->active_segment.event_overflow += stored_events;
            recorder->superblock.event_records_overflow +=
                recorder->active_segment.event_overflow;
            return result;
        }
        copied += sector->record_count;
    }
    recorder->superblock.next_event_lba += sectors;
    recorder->superblock.event_records_written += stored_events;
    recorder->superblock.event_records_overflow +=
        recorder->active_segment.event_overflow;
    return ESP_OK;
}

esp_err_t raw_sd_recorder_close_segment(raw_sd_recorder_t *recorder,
                                        raw_sd_segment_state_t final_state,
                                        esp_err_t failure_code,
                                        const raw_sd_segment_diagnostics_t *diagnostics)
{
    if (recorder == NULL || !recorder->segment_open ||
        (final_state != RAW_SD_SEGMENT_CLOSED && final_state != RAW_SD_SEGMENT_FAILED)) {
        return ESP_ERR_INVALID_STATE;
    }
    apply_segment_diagnostics(recorder, diagnostics, final_state);
    esp_err_t result = flush_tail(recorder);
    uint64_t confirmed_valid_bytes = recorder->active_pending_valid_bytes;
    if (result != ESP_OK) {
        const uint64_t unflushed_bytes = recorder->write_buffer_used;
        confirmed_valid_bytes = unflushed_bytes < confirmed_valid_bytes
            ? confirmed_valid_bytes - unflushed_bytes
            : 0U;
        confirmed_valid_bytes -= confirmed_valid_bytes % RAW_SD_FRAME_BYTES;
        final_state = RAW_SD_SEGMENT_FAILED;
        failure_code = result;
        recorder->active_segment.capture_outcome =
            RAW_SD_CAPTURE_FAILED_PIPELINE;
        if (diagnostics != NULL && diagnostics->events != NULL) {
            recorder->active_segment.event_overflow +=
                diagnostics->event_count;
        }
        recorder->superblock.event_records_overflow +=
            recorder->active_segment.event_overflow;
        /* A failed 64 KiB/tail write remains buffered. It must not leak into
         * a later segment if the caller attempts another recording. */
        recorder->write_buffer_used = 0U;
    }
    if (result == ESP_OK) {
        result = write_sync_events(recorder, diagnostics);
        if (result != ESP_OK) {
            final_state = RAW_SD_SEGMENT_FAILED;
            failure_code = result;
            recorder->active_segment.capture_outcome =
                RAW_SD_CAPTURE_FAILED_PIPELINE;
        }
    }
    recorder->active_segment.valid_bytes = confirmed_valid_bytes;
    recorder->active_segment.frame_count = recorder->active_segment.valid_bytes / RAW_SD_FRAME_BYTES;
    recorder->superblock.valid_bytes_written +=
        recorder->active_segment.valid_bytes;
    recorder->active_segment.state = final_state;
    recorder->active_segment.metadata_generation =
        recorder->superblock.generation + 1U;
    recorder->active_segment.failure_code = (uint32_t)failure_code;
    recorder->active_segment.end_time_us = (uint64_t)esp_timer_get_time();
    const esp_err_t directory_result = write_active_segment(recorder);
    if (directory_result != ESP_OK) {
        return directory_result;
    }
    if (final_state == RAW_SD_SEGMENT_CLOSED) {
        recorder->superblock.closed_segment_count++;
    } else {
        recorder->superblock.state = RAW_SD_RUN_FAILED;
        recorder->superblock.failure_code = (uint32_t)failure_code;
    }
    if (final_state == RAW_SD_SEGMENT_CLOSED &&
        recorder->superblock.physical_bytes_written ==
            (uint64_t)RAW_SD_DATA_CAPACITY_SECTORS * RAW_SD_SECTOR_BYTES) {
        recorder->run_full = true;
        recorder->superblock.state = RAW_SD_RUN_COMPLETE;
    }
    const esp_err_t content_result = result;
    const esp_err_t superblock_result = write_superblocks(recorder);
    /* The directory was written before the superblock. If the superblock
     * update fails, the next open can still proceed; readers ignore the
     * uncommitted directory tail using the older superblock counters. */
    recorder->segment_open = false;
    recorder->active_pending_valid_bytes = 0U;
    recorder->next_metadata_bytes = 0U;
    return content_result != ESP_OK ? content_result : superblock_result;
}

bool raw_sd_recorder_run_is_full(const raw_sd_recorder_t *recorder)
{
    return recorder != NULL && recorder->run_full;
}
