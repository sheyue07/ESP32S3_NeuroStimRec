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
#include "adc_frame_pack.h"
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
_Static_assert(RAW_SD_STORED_FRAME_BYTES == ADC_PACKED_FRAME_BYTES,
               "packed metadata must match the packer");

static esp_err_t write_superblocks(raw_sd_recorder_t *recorder)
{
    recorder->superblock.generation++;
    recorder->superblock.next_write_lba = recorder->next_write_lba;
    recorder->superblock.last_update_time_us = (uint64_t)esp_timer_get_time();
    raw_sd_superblock_finalize(&recorder->superblock);
    esp_err_t result = sdmmc_write_sectors(
        &recorder->card, &recorder->superblock,
        RAW_SD_SUPERBLOCK_LBA_A, 1U);
    if (result != ESP_OK) {
        return result;
    }
    return sdmmc_write_sectors(
        &recorder->card, &recorder->superblock,
        RAW_SD_SUPERBLOCK_LBA_B, 1U);
}

static esp_err_t write_active_segment(raw_sd_recorder_t *recorder)
{
    raw_sd_segment_finalize(&recorder->active_segment);
    return sdmmc_write_sectors(
        &recorder->card, &recorder->active_segment,
        recorder->active_directory_lba, 1U);
}

static esp_err_t read_metadata_sector(raw_sd_recorder_t *recorder,
                                      uint32_t lba, void *destination)
{
    const esp_err_t result = sdmmc_read_sectors(
        &recorder->card, recorder->write_buffer, lba, 1U);
    if (result == ESP_OK) {
        memcpy(destination, recorder->write_buffer, RAW_SD_SECTOR_BYTES);
    }
    return result;
}

static esp_err_t load_latest_superblock(raw_sd_recorder_t *recorder,
                                        raw_sd_superblock_t *superblock)
{
    raw_sd_superblock_t copy_a;
    raw_sd_superblock_t copy_b;
    const esp_err_t result_a = read_metadata_sector(
        recorder, RAW_SD_SUPERBLOCK_LBA_A, &copy_a);
    const esp_err_t result_b = read_metadata_sector(
        recorder, RAW_SD_SUPERBLOCK_LBA_B, &copy_b);
    const bool valid_a = result_a == ESP_OK &&
                         raw_sd_superblock_is_valid(&copy_a);
    const bool valid_b = result_b == ESP_OK &&
                         raw_sd_superblock_is_valid(&copy_b);
    if (!valid_a && !valid_b) {
        if (result_a != ESP_OK) {
            return result_a;
        }
        if (result_b != ESP_OK) {
            return result_b;
        }
        return ESP_ERR_INVALID_CRC;
    }
    *superblock = valid_b && (!valid_a || copy_b.generation > copy_a.generation)
                      ? copy_b
                      : copy_a;
    return ESP_OK;
}

static esp_err_t write_data_block(raw_sd_recorder_t *recorder,
                                  const uint8_t *data, size_t bytes)
{
    if (bytes == 0U || bytes % RAW_SD_SECTOR_BYTES != 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint64_t sectors = bytes / RAW_SD_SECTOR_BYTES;
    if (recorder->next_write_lba > recorder->data_end_lba ||
        sectors > recorder->data_end_lba - recorder->next_write_lba) {
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
            recorder->active_pending_valid_bytes / RAW_SD_STORED_FRAME_BYTES * RAW_SD_STORED_FRAME_BYTES;
        recorder->active_segment.frame_count =
            recorder->active_segment.valid_bytes / RAW_SD_STORED_FRAME_BYTES;
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
    const uint64_t card_sectors = (uint64_t)recorder->card.csd.capacity;
    if (card_sectors <= RAW_SD_DATA_START_LBA) {
        ESP_LOGE(TAG, "eMMC user area too small: sectors=%" PRIu64
                      " metadata_end=%" PRIu32,
                 card_sectors, RAW_SD_DATA_START_LBA);
        (void)sdmmc_host_deinit();
        return ESP_ERR_INVALID_SIZE;
    }
    const uint64_t data_sectors = card_sectors - RAW_SD_DATA_START_LBA;
    if (data_sectors > UINT32_MAX) {
        ESP_LOGE(TAG, "eMMC data area exceeds format-v2 limit: sectors=%" PRIu64,
                 data_sectors);
        (void)sdmmc_host_deinit();
        return ESP_ERR_NOT_SUPPORTED;
    }
    recorder->data_capacity_sectors = data_sectors;
    recorder->data_end_lba = card_sectors;
    recorder->write_buffer = heap_caps_malloc(RAW_SD_RECORDER_WRITE_BUFFER_BYTES,
                                               MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA |
                                               MALLOC_CAP_8BIT);
    if (recorder->write_buffer == NULL) {
        heap_caps_free(recorder->write_buffer);
        recorder->write_buffer = NULL;
        (void)sdmmc_host_deinit();
        return ESP_ERR_NO_MEM;
    }
    recorder->card_initialized = true;
    ESP_LOGI(TAG,
             "eMMC ready: 4-bit SDMMC, actual clock %.2f MHz, "
             "raw data area=%" PRIu64 " sectors (%" PRIu64 " bytes)",
             (double)recorder->card.real_freq_khz / 1000.0,
             recorder->data_capacity_sectors,
             recorder->data_capacity_sectors * RAW_SD_SECTOR_BYTES);
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
    recorder->superblock.data_capacity_sectors =
        (uint32_t)recorder->data_capacity_sectors;
    recorder->superblock.directory_capacity =
        RAW_SD_SEGMENT_DIRECTORY_CAPACITY;
    recorder->superblock.event_area_start_lba = RAW_SD_EVENT_AREA_START_LBA;
    recorder->superblock.next_event_lba = RAW_SD_EVENT_AREA_START_LBA;
    recorder->superblock.start_time_us = (uint64_t)esp_timer_get_time();
    recorder->next_write_lba = RAW_SD_DATA_START_LBA;
    recorder->write_buffer_used = 0U;
    recorder->run_full = false;
    /* A power-cycle starts a replacement run. Clear only the small catalog
     * so stale directory slots can never be mistaken for new captures; the
     * multi-GiB data area is overwritten progressively, not pre-erased. */
    memset(recorder->write_buffer, 0, RAW_SD_RECORDER_WRITE_BUFFER_BYTES);
    uint32_t clear_lba = RAW_SD_SEGMENT_DIRECTORY_START_LBA;
    uint32_t remaining = RAW_SD_SEGMENT_DIRECTORY_CAPACITY;
    while (remaining != 0U) {
        const uint32_t chunk = remaining > RAW_SD_RECORDER_SECTORS_PER_WRITE
                                   ? RAW_SD_RECORDER_SECTORS_PER_WRITE
                                   : remaining;
        const esp_err_t clear_result = sdmmc_write_sectors(
            &recorder->card, recorder->write_buffer, clear_lba, chunk);
        if (clear_result != ESP_OK) {
            return clear_result;
        }
        clear_lba += chunk;
        remaining -= chunk;
    }
    ESP_LOGI(TAG,
             "Old catalog cleared; new eMMC run overwrites from LBA%" PRIu32,
             RAW_SD_DATA_START_LBA);
    return write_superblocks(recorder);
}

esp_err_t raw_sd_recorder_resume_run(raw_sd_recorder_t *recorder)
{
    if (recorder == NULL || !recorder->card_initialized ||
        recorder->segment_open) {
        return ESP_ERR_INVALID_STATE;
    }

    raw_sd_superblock_t superblock;
    esp_err_t result = load_latest_superblock(recorder, &superblock);
    if (result != ESP_OK) {
        return result;
    }
    if (superblock.data_start_lba != RAW_SD_DATA_START_LBA ||
        superblock.data_capacity_sectors != recorder->data_capacity_sectors ||
        superblock.directory_capacity != RAW_SD_SEGMENT_DIRECTORY_CAPACITY ||
        superblock.event_area_start_lba != RAW_SD_EVENT_AREA_START_LBA ||
        superblock.segment_count > RAW_SD_SEGMENT_DIRECTORY_CAPACITY) {
        ESP_LOGE(TAG, "Existing eMMC catalog layout is incompatible");
        return ESP_ERR_NOT_SUPPORTED;
    }

    uint64_t expected_lba = RAW_SD_DATA_START_LBA;
    uint64_t physical_bytes = 0U;
    uint64_t valid_bytes = 0U;
    uint32_t closed_segments = 0U;
    uint32_t next_event_lba = RAW_SD_EVENT_AREA_START_LBA;
    bool recovered_open_segment = false;

    for (uint32_t index = 0U; index < superblock.segment_count; ++index) {
        raw_sd_segment_t segment;
        result = read_metadata_sector(
            recorder, RAW_SD_SEGMENT_DIRECTORY_START_LBA + index, &segment);
        if (result != ESP_OK) {
            return result;
        }
        if (!raw_sd_segment_is_valid(&segment) ||
            segment.segment_id != index + 1U ||
            segment.run_id != superblock.run_id ||
            segment.start_lba != expected_lba ||
            (segment.state != RAW_SD_SEGMENT_OPEN &&
             segment.state != RAW_SD_SEGMENT_CLOSED &&
             segment.state != RAW_SD_SEGMENT_FAILED) ||
            segment.physical_bytes % RAW_SD_SECTOR_BYTES != 0U ||
            segment.valid_bytes > segment.physical_bytes ||
            segment.valid_bytes % raw_sd_segment_frame_bytes(&segment) != 0U) {
            ESP_LOGE(TAG, "Invalid eMMC directory entry at index %" PRIu32,
                     index);
            return ESP_ERR_INVALID_CRC;
        }

        const uint64_t segment_sectors =
            segment.physical_bytes / RAW_SD_SECTOR_BYTES;
        if (expected_lba > recorder->data_end_lba ||
            segment_sectors > recorder->data_end_lba - expected_lba) {
            return ESP_ERR_INVALID_SIZE;
        }
        expected_lba += segment_sectors;
        physical_bytes += segment.physical_bytes;
        valid_bytes += segment.valid_bytes;
        if (segment.state == RAW_SD_SEGMENT_CLOSED) {
            closed_segments++;
        }
        if (segment.event_sector_count != 0U) {
            const uint64_t event_end =
                (uint64_t)segment.event_start_lba +
                segment.event_sector_count;
            if (event_end > RAW_SD_EVENT_AREA_END_LBA) {
                return ESP_ERR_INVALID_SIZE;
            }
            if (event_end > next_event_lba) {
                next_event_lba = (uint32_t)event_end;
            }
        }

        if (segment.state == RAW_SD_SEGMENT_OPEN) {
            /* A reset can leave the last checkpoint OPEN. Preserve every
             * committed complete frame, close that slot as failed, then put
             * the next capture after it. At most the uncheckpointed tail is
             * overwritten; earlier captures are never discarded. */
            if (index + 1U != superblock.segment_count) {
                return ESP_ERR_INVALID_STATE;
            }
            segment.state = RAW_SD_SEGMENT_FAILED;
            segment.capture_outcome = RAW_SD_CAPTURE_FAILED_PIPELINE;
            segment.failure_code = (uint32_t)ESP_ERR_INVALID_STATE;
            segment.end_time_us = (uint64_t)esp_timer_get_time();
            segment.metadata_generation = superblock.generation + 1U;
            raw_sd_segment_finalize(&segment);
            memcpy(recorder->write_buffer, &segment, sizeof(segment));
            result = sdmmc_write_sectors(
                &recorder->card, recorder->write_buffer,
                RAW_SD_SEGMENT_DIRECTORY_START_LBA + index, 1U);
            if (result != ESP_OK) {
                return result;
            }
            recovered_open_segment = true;
        }
    }

    recorder->superblock = superblock;
    recorder->superblock.state = RAW_SD_RUN_RUNNING;
    recorder->superblock.failure_code = 0U;
    recorder->superblock.closed_segment_count = closed_segments;
    recorder->superblock.physical_bytes_written = physical_bytes;
    recorder->superblock.valid_bytes_written = valid_bytes;
    recorder->superblock.next_event_lba = next_event_lba;
    recorder->next_write_lba = expected_lba;
    recorder->write_buffer_used = 0U;
    recorder->active_pending_valid_bytes = 0U;
    recorder->next_metadata_bytes = 0U;
    recorder->segment_open = false;
    recorder->run_full = expected_lba >= recorder->data_end_lba ||
        superblock.segment_count >= RAW_SD_SEGMENT_DIRECTORY_CAPACITY;

    result = write_superblocks(recorder);
    if (result != ESP_OK) {
        return result;
    }
    ESP_LOGI(TAG,
             "Resumed eMMC run=%" PRIu32 ": segments=%" PRIu32
             ", next_lba=%" PRIu64 ", remaining=%" PRIu64 " bytes%s",
             recorder->superblock.run_id,
             recorder->superblock.segment_count,
             recorder->next_write_lba,
             raw_sd_recorder_remaining_capacity_bytes(recorder),
             recovered_open_segment ? ", recovered interrupted OPEN segment"
                                    : "");
    return ESP_OK;
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
    /* Capacity is the complete eMMC user area after the fixed metadata LBAs.
     * Count physical bytes because every closed segment can add tail padding. */
    const uint64_t capacity_bytes =
        recorder->data_capacity_sectors * RAW_SD_SECTOR_BYTES;
    if (recorder->superblock.physical_bytes_written > capacity_bytes ||
        recorder->write_buffer_used >
            capacity_bytes - recorder->superblock.physical_bytes_written) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint64_t physically_available =
        capacity_bytes - recorder->superblock.physical_bytes_written -
        recorder->write_buffer_used;
    /* API consumes RAW260; only PACKED132 enters the disk write cache. */
    const uint64_t remaining = physically_available /
        RAW_SD_STORED_FRAME_BYTES * RAW_SD_FRAME_BYTES;
    const size_t accepted = length > remaining ? (size_t)remaining : length;
    size_t offset = 0U;
    while (offset < accepted) {
        uint8_t packed[ADC_PACKED_FRAME_BYTES];
        adc_frame_pack(packed, frames + offset);
        size_t packed_offset = 0U;
        while (packed_offset < sizeof(packed)) {
            const size_t available = RAW_SD_RECORDER_WRITE_BUFFER_BYTES - recorder->write_buffer_used;
            const size_t left = sizeof(packed) - packed_offset;
            const size_t copy_bytes = left < available ? left : available;
            memcpy(recorder->write_buffer + recorder->write_buffer_used,
                   packed + packed_offset, copy_bytes);
            recorder->write_buffer_used += copy_bytes;
            recorder->active_pending_valid_bytes += copy_bytes;
            packed_offset += copy_bytes;
            const esp_err_t result = flush_full_buffer(recorder);
            if (result != ESP_OK) return result;
        }
        offset += RAW_SD_FRAME_BYTES;
        if (consumed != NULL) *consumed += RAW_SD_FRAME_BYTES;
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
        confirmed_valid_bytes -= confirmed_valid_bytes % RAW_SD_STORED_FRAME_BYTES;
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
    /* An I/O failure can leave a partial packed frame even if a later tail
     * flush succeeds. Never publish that fragment as valid data. */
    confirmed_valid_bytes -= confirmed_valid_bytes % RAW_SD_STORED_FRAME_BYTES;
    recorder->active_segment.valid_bytes = confirmed_valid_bytes;
    recorder->active_segment.frame_count = recorder->active_segment.valid_bytes / RAW_SD_STORED_FRAME_BYTES;
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
            recorder->data_capacity_sectors * RAW_SD_SECTOR_BYTES) {
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

uint64_t raw_sd_recorder_data_capacity_sectors(
    const raw_sd_recorder_t *recorder)
{
    return recorder != NULL ? recorder->data_capacity_sectors : 0U;
}

uint64_t raw_sd_recorder_remaining_capacity_bytes(
    const raw_sd_recorder_t *recorder)
{
    if (recorder == NULL) {
        return 0U;
    }
    const uint64_t capacity_bytes =
        recorder->data_capacity_sectors * RAW_SD_SECTOR_BYTES;
    if (recorder->superblock.physical_bytes_written > capacity_bytes ||
        recorder->write_buffer_used >
            capacity_bytes - recorder->superblock.physical_bytes_written) {
        return 0U;
    }
    return capacity_bytes - recorder->superblock.physical_bytes_written -
           recorder->write_buffer_used;
}
