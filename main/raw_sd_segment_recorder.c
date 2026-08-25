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

static const char *TAG = "RAW_SD_REC";

#define SDMMC_PIN_CLK GPIO_NUM_41
#define SDMMC_PIN_CMD GPIO_NUM_42
#define SDMMC_PIN_D0 GPIO_NUM_40
#define SDMMC_PIN_D1 GPIO_NUM_39
#define SDMMC_PIN_D2 GPIO_NUM_1
#define SDMMC_PIN_D3 GPIO_NUM_2
#define SDMMC_MAX_FREQ_KHZ 20000
#define RAW_SD_RECORDER_SECTORS_PER_WRITE \
    (RAW_SD_RECORDER_WRITE_BUFFER_BYTES / RAW_SD_SECTOR_BYTES)

_Static_assert(RAW_SD_RECORDER_WRITE_BUFFER_BYTES % RAW_SD_SECTOR_BYTES == 0U,
               "write buffer must contain whole sectors");
_Static_assert(RAW_SD_FRAME_BYTES == ADC_FRAME_SIZE_BYTES,
               "raw SD frame size must match the stream parser");

static uint64_t data_end_lba(const raw_sd_recorder_t *recorder)
{
    return RAW_SD_DATA_START_LBA + recorder->data_capacity_sectors;
}

static uint64_t max_valid_bytes(const raw_sd_recorder_t *recorder)
{
    return recorder->data_capacity_sectors * RAW_SD_SECTOR_BYTES /
        RAW_SD_FRAME_BYTES * RAW_SD_FRAME_BYTES;
}

static uint64_t round_up_u64(uint64_t value, uint64_t alignment)
{
    if (alignment <= 1U) {
        return value;
    }
    const uint64_t remainder = value % alignment;
    if (remainder == 0U) {
        return value;
    }
    const uint64_t increment = alignment - remainder;
    return value <= UINT64_MAX - increment ? value + increment : UINT64_MAX;
}

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

static bool previous_superblock_usable(
    const raw_sd_recorder_t *recorder,
    const raw_sd_superblock_t *superblock,
    uint64_t *end_lba)
{
    if (!raw_sd_superblock_is_valid(superblock) ||
        superblock->data_start_lba != RAW_SD_DATA_START_LBA ||
        superblock->data_capacity_sectors == 0U) {
        return false;
    }
    const uint64_t previous_capacity_end =
        (uint64_t)superblock->data_start_lba +
        superblock->data_capacity_sectors;
    const uint64_t physical_sectors =
        (superblock->physical_bytes_written + RAW_SD_SECTOR_BYTES - 1U) /
        RAW_SD_SECTOR_BYTES;
    const uint64_t physical_end =
        (uint64_t)superblock->data_start_lba + physical_sectors;
    uint64_t selected_end = superblock->next_write_lba;
    if (physical_end > selected_end) {
        selected_end = physical_end;
    }
    if (selected_end < RAW_SD_DATA_START_LBA ||
        selected_end > previous_capacity_end ||
        selected_end > (uint64_t)recorder->card.csd.capacity) {
        return false;
    }
    *end_lba = selected_end;
    return true;
}

static esp_err_t load_previous_run_end(
    raw_sd_recorder_t *recorder,
    uint64_t *previous_end_lba,
    bool *previous_incomplete,
    bool *found)
{
    raw_sd_superblock_t copy_a = {0};
    raw_sd_superblock_t copy_b = {0};
    esp_err_t read_a = sdmmc_read_sectors(
        &recorder->card, recorder->write_buffer,
        RAW_SD_SUPERBLOCK_LBA_A, 1U);
    if (read_a == ESP_OK) {
        memcpy(&copy_a, recorder->write_buffer, sizeof(copy_a));
    } else {
        ESP_LOGW(TAG, "Previous superblock A read failed: 0x%x (%s)",
                 (unsigned int)read_a, esp_err_to_name(read_a));
    }
    esp_err_t read_b = sdmmc_read_sectors(
        &recorder->card, recorder->write_buffer,
        RAW_SD_SUPERBLOCK_LBA_B, 1U);
    if (read_b == ESP_OK) {
        memcpy(&copy_b, recorder->write_buffer, sizeof(copy_b));
    } else {
        ESP_LOGW(TAG, "Previous superblock B read failed: 0x%x (%s)",
                 (unsigned int)read_b, esp_err_to_name(read_b));
    }
    if (read_a != ESP_OK && read_b != ESP_OK) {
        return read_a;
    }

    uint64_t end_a = 0U;
    uint64_t end_b = 0U;
    const bool valid_a = read_a == ESP_OK &&
        previous_superblock_usable(recorder, &copy_a, &end_a);
    const bool valid_b = read_b == ESP_OK &&
        previous_superblock_usable(recorder, &copy_b, &end_b);
    if (!valid_a && !valid_b) {
        *found = false;
        return ESP_OK;
    }

    const raw_sd_superblock_t *selected = &copy_a;
    uint64_t selected_end = end_a;
    if (!valid_a || (valid_b &&
        (copy_b.generation > copy_a.generation ||
         (copy_b.generation == copy_a.generation && end_b > end_a)))) {
        selected = &copy_b;
        selected_end = end_b;
    }
    *previous_end_lba = selected_end;
    *previous_incomplete = false;
    if (selected->segment_count > RAW_SD_SEGMENT_DIRECTORY_CAPACITY) {
        *previous_incomplete = true;
    } else if (selected->segment_count > 0U) {
        raw_sd_segment_t last_segment = {0};
        const uint32_t last_segment_lba =
            RAW_SD_SEGMENT_DIRECTORY_START_LBA +
            selected->segment_count - 1U;
        const esp_err_t segment_read = sdmmc_read_sectors(
            &recorder->card, recorder->write_buffer,
            last_segment_lba, 1U);
        if (segment_read == ESP_OK) {
            memcpy(&last_segment, recorder->write_buffer,
                   sizeof(last_segment));
        }
        const uint64_t segment_end_lba = last_segment.start_lba +
            (last_segment.physical_bytes + RAW_SD_SECTOR_BYTES - 1U) /
            RAW_SD_SECTOR_BYTES;
        if (segment_read != ESP_OK ||
            !raw_sd_segment_is_valid(&last_segment) ||
            last_segment.run_id != selected->run_id ||
            last_segment.segment_id != selected->segment_count ||
            last_segment.metadata_generation > selected->generation ||
            (last_segment.state != RAW_SD_SEGMENT_CLOSED &&
             last_segment.state != RAW_SD_SEGMENT_FAILED) ||
            segment_end_lba != selected_end) {
            *previous_incomplete = true;
        }
    }
    *found = true;
    ESP_LOGI(TAG,
             "Previous run metadata: generation=%" PRIu32
             ", end_lba=%" PRIu64 ", physical=%" PRIu64 " bytes",
             selected->generation, selected_end,
             selected->physical_bytes_written);
    return ESP_OK;
}

static esp_err_t prepare_previous_run_range(raw_sd_recorder_t *recorder)
{
    uint64_t previous_end_lba = 0U;
    bool previous_incomplete = false;
    bool found = false;
    esp_err_t result = load_previous_run_end(
        recorder, &previous_end_lba, &previous_incomplete, &found);
    if (result != ESP_OK) {
        return result;
    }
    if (!found) {
        ESP_LOGW(TAG,
                 "No valid previous raw-SD metadata; targeted pre-erase skipped");
        return ESP_OK;
    }
    if (previous_incomplete) {
        ESP_LOGW(TAG,
                 "Previous run did not close normally; recorded end LBA may be stale");
    }

    uint64_t allocation_unit_bytes =
        (uint64_t)recorder->card.ssr.alloc_unit_kb * 1024U;
    if (allocation_unit_bytes < RAW_SD_SECTOR_BYTES) {
        allocation_unit_bytes = RAW_SD_ERASE_FALLBACK_AU_BYTES;
        ESP_LOGW(TAG,
                 "Card did not report Allocation Unit; using %" PRIu64
                 " MiB fallback",
                 allocation_unit_bytes / (1024U * 1024U));
    }
    const uint64_t allocation_unit_sectors =
        allocation_unit_bytes / RAW_SD_SECTOR_BYTES;
    const uint64_t erase_margin_sectors =
        RAW_SD_ERASE_MARGIN_BYTES / RAW_SD_SECTOR_BYTES;
    const uint64_t card_end_lba = (uint64_t)recorder->card.csd.capacity;
    uint64_t requested_end_lba = previous_end_lba;
    if (erase_margin_sectors <= card_end_lba &&
        requested_end_lba <= card_end_lba - erase_margin_sectors) {
        requested_end_lba += erase_margin_sectors;
    } else {
        requested_end_lba = card_end_lba;
    }
    requested_end_lba = round_up_u64(
        requested_end_lba, allocation_unit_sectors);
    if (requested_end_lba > card_end_lba) {
        requested_end_lba = card_end_lba;
    }
    if (requested_end_lba > SIZE_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }

    const uint64_t previous_used_sectors =
        previous_end_lba - RAW_SD_DATA_START_LBA;
    ESP_LOGI(TAG,
             "Pre-erasing previous raw range: used=%" PRIu64
             " MiB, AU=%" PRIu64 " KiB, erase=LBA0..%" PRIu64
             " (%" PRIu64 " MiB)",
             previous_used_sectors * RAW_SD_SECTOR_BYTES / (1024U * 1024U),
             allocation_unit_bytes / 1024U,
             requested_end_lba - 1U,
             requested_end_lba * RAW_SD_SECTOR_BYTES / (1024U * 1024U));
    const int64_t erase_start_us = esp_timer_get_time();
    result = sdmmc_erase_sectors(
        &recorder->card, 0U, (size_t)requested_end_lba,
        SDMMC_ERASE_ARG);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Raw range pre-erase failed: 0x%x (%s)",
                 (unsigned int)result, esp_err_to_name(result));
        return result;
    }
    ESP_LOGI(TAG, "Raw range pre-erase completed in %" PRIi64 " ms",
             (esp_timer_get_time() - erase_start_us) / 1000);
    return ESP_OK;
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
    if (recorder->next_write_lba + sectors > data_end_lba(recorder)) {
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
    return ESP_OK;
}

static esp_err_t flush_full_buffer(raw_sd_recorder_t *recorder)
{
    if (recorder->write_buffer_used != RAW_SD_RECORDER_WRITE_BUFFER_BYTES) {
        return ESP_OK;
    }
    esp_err_t result = write_data_block(recorder, recorder->write_buffer,
                                        recorder->write_buffer_used);
    if (result != ESP_OK) {
        return result;
    }
    recorder->write_buffer_used = 0U;
    return ESP_OK;
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
    esp_err_t result = write_data_block(recorder, recorder->write_buffer,
                                        physical_bytes);
    if (result != ESP_OK) {
        return result;
    }
    recorder->write_buffer_used = 0U;
    return ESP_OK;
}

esp_err_t raw_sd_recorder_init(raw_sd_recorder_t *recorder)
{
    if (recorder == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(recorder, 0, sizeof(*recorder));
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_MAX_FREQ_KHZ;
    host.command_timeout_ms = 3000;
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;
    slot_config.clk = SDMMC_PIN_CLK;
    slot_config.cmd = SDMMC_PIN_CMD;
    slot_config.d0 = SDMMC_PIN_D0;
    slot_config.d1 = SDMMC_PIN_D1;
    slot_config.d2 = SDMMC_PIN_D2;
    slot_config.d3 = SDMMC_PIN_D3;
    slot_config.cd = GPIO_NUM_NC;
    slot_config.wp = GPIO_NUM_NC;
    esp_err_t result = sdmmc_host_init();
    if (result != ESP_OK) {
        return result;
    }
    result = sdmmc_host_init_slot(host.slot, &slot_config);
    if (result != ESP_OK) {
        (void)sdmmc_host_deinit();
        return result;
    }
    result = sdmmc_card_init(&host, &recorder->card);
    if (result != ESP_OK) {
        (void)sdmmc_host_deinit();
        return result;
    }
    if (recorder->card.csd.sector_size != RAW_SD_SECTOR_BYTES) {
        ESP_LOGE(TAG, "Unsupported sector size: %d bytes", recorder->card.csd.sector_size);
        (void)sdmmc_host_deinit();
        return ESP_ERR_NOT_SUPPORTED;
    }
    if ((uint64_t)recorder->card.csd.capacity <= RAW_SD_DATA_START_LBA) {
        ESP_LOGE(TAG, "Card too small: sectors=%d metadata_end=%" PRIu32,
                 recorder->card.csd.capacity, RAW_SD_DATA_START_LBA);
        (void)sdmmc_host_deinit();
        return ESP_ERR_INVALID_SIZE;
    }
    recorder->data_capacity_sectors =
        (uint64_t)recorder->card.csd.capacity - RAW_SD_DATA_START_LBA;
    if (recorder->data_capacity_sectors > UINT32_MAX) {
        ESP_LOGE(TAG, "Raw data area exceeds metadata field capacity");
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
    sdmmc_card_print_info(stdout, &recorder->card);
    ESP_LOGI(TAG, "Raw data capacity after LBA2048: %" PRIu64 " MiB",
             recorder->data_capacity_sectors * RAW_SD_SECTOR_BYTES /
                 (1024U * 1024U));
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
    esp_err_t result = prepare_previous_run_range(recorder);
    if (result != ESP_OK) {
        return result;
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
    return content_result != ESP_OK ? content_result : superblock_result;
}

bool raw_sd_recorder_run_is_full(const raw_sd_recorder_t *recorder)
{
    return recorder != NULL && recorder->run_full;
}

uint64_t raw_sd_recorder_max_valid_frames(const raw_sd_recorder_t *recorder)
{
    return recorder == NULL ? 0U :
        max_valid_bytes(recorder) / RAW_SD_FRAME_BYTES;
}
