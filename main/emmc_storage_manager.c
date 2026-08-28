/*
 * Unified ESP32-S3 ADC capture and raw eMMC storage manager.
 *
 * GPIO20 external clock + GPIO16 serial data (rising-edge sample, MSB first)
 * -> SPI2/GDMA cyclic internal buffers -> 2 MiB raw PSRAM ring
 * -> byte-oriented 260-byte frame synchronizer -> 12 MiB valid PSRAM ring
 * -> 64 KiB SDMMC DMA cache -> raw eMMC user-area segmented data region.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "continuous_rx.h"
#include "emmc_storage_manager.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "frame_sync.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "raw_sd_segment_recorder.h"
#include "stim_controller.h"
#include "write_switch.h"

static const char *TAG = "EMMC_MANAGER";

#define RAW_RING_BUFFER_SIZE        (2U * 1024U * 1024U)
#define VALID_RING_BUFFER_SIZE      (12U * 1024U * 1024U)
#define FRAME_BATCH_FRAMES          125U
#define FRAME_BATCH_SIZE            (FRAME_BATCH_FRAMES * ADC_FRAME_SIZE_BYTES)
#define MAX_RECORD_FRAMES           UINT64_C(4129776)
#define RATE_LOG_INTERVAL_US        UINT64_C(1000000)
#define CAPTURE_STOP_TIMEOUT_MS     5000
#define STORAGE_QUEUE_LENGTH        16U
#define STORAGE_READ_DMA_SECTORS    16U
#define SYNC_EVENT_CAPACITY \
    ((RAW_SD_EVENT_AREA_END_LBA - RAW_SD_EVENT_AREA_START_LBA) * \
     RAW_SD_EVENTS_PER_SECTOR)

typedef enum {
    APPEND_OK,
    APPEND_FILE_LIMIT,
    APPEND_IO_ERROR,
} append_result_t;

typedef struct {
    uint64_t written_bytes;
    uint64_t interval_written_bytes;
    uint64_t last_input_bytes;
    uint64_t last_valid_frames;
    uint64_t last_physical_bytes;
    bool io_failed;
} recording_context_t;

typedef struct {
    bool enabled;
    bool failed;
    bool limit_reached;
    uint64_t accepted_frames;
    uint64_t raw_input_bytes;
    uint64_t last_dma_sequence;
    uint32_t dma_sequence_gaps;
    uint64_t raw_overflow_events;
    uint64_t raw_overflow_bytes;
    uint64_t valid_overflow_frames;
    char failure_reason[96];
} capture_status_t;

typedef struct {
    uint8_t *batch;
    size_t batch_pos;
    uint64_t total_frames;
} parser_output_context_t;

typedef enum {
    STORAGE_REQUEST_START_WRITE,
    STORAGE_REQUEST_STOP_WRITE,
    STORAGE_REQUEST_READ,
    STORAGE_REQUEST_REINIT,
} storage_request_type_t;

typedef struct {
    storage_request_type_t type;
    emmc_storage_read_operation_t operation;
    void *context;
    TaskHandle_t waiter;
    esp_err_t *result;
} storage_request_t;

static RingbufHandle_t raw_ringbuf;
static RingbufHandle_t data_ringbuf;
static SemaphoreHandle_t recording_mutex;
static SemaphoreHandle_t dma_stopped_sem;
static SemaphoreHandle_t parser_stopped_sem;
static TaskHandle_t dma_task_handle;
static TaskHandle_t parser_task_handle;
static raw_sd_recorder_t raw_recorder;
static frame_sync_t stream_sync;
static capture_status_t capture_status;
static volatile bool dma_session_done;
static uint8_t *parser_batch_storage;
static void *frame_sync_workspace;
static raw_sd_sync_event_t *sync_event_storage;
static uint32_t sync_event_count;
static uint32_t sync_event_overflow;
static QueueHandle_t request_queue;
static SemaphoreHandle_t status_mutex;
static emmc_status_t current_status;
static uint8_t *read_dma_buffer;
static recording_context_t manager_recording_context;
static bool run_started;
static bool stop_requested;
static int64_t recording_start_us;
static uint32_t last_capture_outcome;
static uint32_t last_sync_state;

static void reset_capture_status(void)
{
    xSemaphoreTake(recording_mutex, portMAX_DELAY);
    memset(&capture_status, 0, sizeof(capture_status));
    capture_status.enabled = true;
    xSemaphoreGive(recording_mutex);
}

static void set_capture_enabled(bool enabled)
{
    xSemaphoreTake(recording_mutex, portMAX_DELAY);
    capture_status.enabled = enabled;
    xSemaphoreGive(recording_mutex);
}

static void get_capture_status(capture_status_t *status)
{
    xSemaphoreTake(recording_mutex, portMAX_DELAY);
    *status = capture_status;
    xSemaphoreGive(recording_mutex);
}

static void signal_capture_failure(const char *operation, esp_err_t error)
{
    xSemaphoreTake(recording_mutex, portMAX_DELAY);
    if (!capture_status.failed) {
        capture_status.failed = true;
        snprintf(capture_status.failure_reason,
                 sizeof(capture_status.failure_reason),
                 "%s: 0x%x (%s)",
                 operation,
                 (unsigned int)error,
                 esp_err_to_name(error));
    }
    xSemaphoreGive(recording_mutex);

    ESP_LOGE(TAG, "%s failed: 0x%x (%s)", operation,
             (unsigned int)error, esp_err_to_name(error));
}

static esp_err_t init_emmc_4bit(void)
{
    return raw_sd_recorder_init(&raw_recorder);
}

static append_result_t append_to_write_cache(recording_context_t *context,
                                              const uint8_t *data,
                                              size_t length,
                                              size_t *consumed)
{
    *consumed = 0;
    if ((length % ADC_FRAME_SIZE_BYTES) != 0U) {
        ESP_LOGE(TAG, "Refusing non-frame-aligned ring item: %zu bytes", length);
        context->io_failed = true;
        return APPEND_IO_ERROR;
    }

    const esp_err_t result = raw_sd_recorder_append(
        &raw_recorder, data, length, consumed);
    context->written_bytes += *consumed;
    context->interval_written_bytes += *consumed;
    if (result == ESP_OK) {
        return APPEND_OK;
    }
    if (result == ESP_ERR_INVALID_SIZE && raw_sd_recorder_run_is_full(&raw_recorder)) {
        return APPEND_FILE_LIMIT;
    }
    context->io_failed = true;
    return APPEND_IO_ERROR;
}

static bool open_raw_segment(recording_context_t *context)
{
    memset(context, 0, sizeof(*context));
    sync_event_count = 0U;
    sync_event_overflow = 0U;
    const esp_err_t result = raw_sd_recorder_open_segment(&raw_recorder);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open raw eMMC segment: 0x%x (%s)",
                 (unsigned int)result, esp_err_to_name(result));
        return false;
    }
    return true;
}

static uint64_t discard_one_ring_buffer(RingbufHandle_t ringbuf)
{
    uint64_t discarded = 0;
    size_t item_size = 0;
    void *item = NULL;
    while ((item = xRingbufferReceive(ringbuf, &item_size, 0)) != NULL) {
        discarded += item_size;
        vRingbufferReturnItem(ringbuf, item);
    }
    return discarded;
}

static uint64_t discard_pipeline_buffers(void)
{
    return discard_one_ring_buffer(raw_ringbuf) +
           discard_one_ring_buffer(data_ringbuf);
}

static bool flush_parser_batch(parser_output_context_t *context)
{
    if (context->batch_pos == 0U) {
        return true;
    }
    if (xRingbufferSend(
            data_ringbuf, context->batch, context->batch_pos, 0) == pdTRUE) {
        context->batch_pos = 0;
        return true;
    }

    xSemaphoreTake(recording_mutex, portMAX_DELAY);
    capture_status.valid_overflow_frames +=
        context->batch_pos / ADC_FRAME_SIZE_BYTES;
    xSemaphoreGive(recording_mutex);
    return false;
}

static void frame_sync_event_callback(const frame_sync_event_t *event,
                                      void *user_ctx)
{
    (void)user_ctx;
    if (event == NULL) {
        return;
    }
    capture_status_t status;
    get_capture_status(&status);
    const bool fast_lock = event->type == FRAME_SYNC_EVENT_FAST_LOCKED;
    if (sync_event_count < SYNC_EVENT_CAPACITY) {
        raw_sd_sync_event_t *const stored =
            &sync_event_storage[sync_event_count++];
        *stored = (raw_sd_sync_event_t){
            .raw_bit_position = event->raw_bit_position,
            .phase_bit_position = event->phase_bit_position,
            .discard_start_bit = event->discard_start_bit,
            .discard_end_bit = event->discard_end_bit,
            .dma_block_sequence = status.last_dma_sequence,
            .verified_frames = fast_lock ? ADC_SYNC_CONFIRM_HEADERS
                                         : event->verified_frames,
            .validation_errors = fast_lock ? event->active_candidates
                                           : event->validation_errors,
            .longest_consecutive_errors =
                event->longest_consecutive_errors,
            .holdover_good_frames = event->holdover_good_frames,
            .holdover_bad_frames = event->holdover_bad_frames,
            .best_candidate_score = event->best_candidate_score,
            .second_candidate_score = event->second_candidate_score,
            .event_type = (uint8_t)event->type,
            .sync_state = (uint8_t)event->state,
            .bit_shift = event->bit_shift,
            .flags = (event->recovery ? 0x01U : 0U) |
                     (fast_lock && event->active_candidates > 1U
                          ? 0x02U
                          : 0U),
        };
    } else {
        ++sync_event_overflow;
    }

    switch (event->type) {
    case FRAME_SYNC_EVENT_FAST_LOCKED:
        if (event->active_candidates > 1U) {
            if (event->recovery) {
                ESP_LOGW(TAG,
                         "SYNC RELOCKED UNCERTAIN: candidates=%u, "
                         "selected=first, phase=%" PRIu64
                         ", shift=%u, raw_bit=%" PRIu64,
                         event->active_candidates,
                         event->phase_bit_position, event->bit_shift,
                         event->raw_bit_position);
            } else {
                ESP_LOGW(TAG,
                         "SYNC LOCKED UNCERTAIN: candidates=%u, "
                         "selected=first, phase=%" PRIu64
                         ", shift=%u, raw_bit=%" PRIu64,
                         event->active_candidates,
                         event->phase_bit_position, event->bit_shift,
                         event->raw_bit_position);
            }
        } else {
            ESP_LOGI(TAG,
                     "%s: candidates=1, phase=%" PRIu64
                     ", shift=%u, raw_bit=%" PRIu64,
                     event->recovery ? "SYNC RELOCKED" : "SYNC LOCKED",
                     event->phase_bit_position, event->bit_shift,
                     event->raw_bit_position);
        }
        break;
    case FRAME_SYNC_EVENT_HOLDOVER_STARTED:
        ESP_LOGW(TAG,
                 "HOLDOVER STARTED: raw_bit=%" PRIu64
                 ", preserving 2080-bit phase",
                 event->raw_bit_position);
        break;
    case FRAME_SYNC_EVENT_HOLDOVER_RECOVERED:
        ESP_LOGW(TAG,
                 "HOLDOVER RECOVERED: good=%u, bad=%u, gap_bits=%" PRIu64,
                 event->holdover_good_frames, event->holdover_bad_frames,
                 event->discard_end_bit - event->discard_start_bit);
        break;
    case FRAME_SYNC_EVENT_HOLDOVER_FAILED:
        ESP_LOGW(TAG,
                 "GLOBAL SYNC SEARCH: holdover checked=%u, bad=%u",
                 event->holdover_good_frames +
                     event->holdover_bad_frames - 1U,
                 event->holdover_bad_frames);
        break;
    case FRAME_SYNC_EVENT_UNRESOLVED:
        ESP_LOGW(TAG,
                 "SYNC UNRESOLVED: no fast lock for 10 seconds; "
                 "DMA remains active, candidates=%u",
                 event->active_candidates);
        break;
    default:
        break;
    }
}

static bool validated_frame_callback(
    const uint8_t frame[ADC_FRAME_SIZE_BYTES], void *user_ctx)
{
    parser_output_context_t *context = user_ctx;
    if (context == NULL) {
        return false;
    }
    if (context->batch_pos == FRAME_BATCH_SIZE &&
        !flush_parser_batch(context)) {
        return false;
    }
    if (context->total_frames >= MAX_RECORD_FRAMES) {
        /* Reaching the complete-frame file limit is a normal stop condition. */
        return true;
    }

    const bool announce_recording = context->total_frames == 0U;
    memcpy(context->batch + context->batch_pos,
           frame,
           ADC_FRAME_SIZE_BYTES);
    context->batch_pos += ADC_FRAME_SIZE_BYTES;
    ++context->total_frames;

    if (announce_recording) {
        ESP_LOGI(TAG,
                 "RECORDING STARTED: first complete %u-byte frame accepted",
                 ADC_FRAME_SIZE_BYTES);
    }
    return true;
}

static void publish_parser_progress(const parser_output_context_t *context)
{
    xSemaphoreTake(recording_mutex, portMAX_DELAY);
    capture_status.accepted_frames = context->total_frames;
    if (context->total_frames >= MAX_RECORD_FRAMES) {
        capture_status.limit_reached = true;
    }
    xSemaphoreGive(recording_mutex);
}

static void adc_dma_task(void *parameter)
{
    (void)parameter;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (true) {
            continuous_rx_block_t block;
            const esp_err_t receive_result = continuous_rx_receive_block(
                &block, pdMS_TO_TICKS(20));
            if (receive_result == ESP_OK) {
                xSemaphoreTake(recording_mutex, portMAX_DELAY);
                const uint64_t expected_sequence =
                    capture_status.last_dma_sequence == 0U
                        ? 1U
                        : capture_status.last_dma_sequence + 1U;
                const bool sequence_ok = block.sequence == expected_sequence;
                if (!sequence_ok) {
                    ++capture_status.dma_sequence_gaps;
                }
                capture_status.last_dma_sequence = block.sequence;
                capture_status.raw_input_bytes += block.length;
                xSemaphoreGive(recording_mutex);
                if (!sequence_ok) {
                    (void)continuous_rx_release_block(block.index);
                    signal_capture_failure("DMA block sequence discontinuity",
                                           ESP_ERR_INVALID_RESPONSE);
                    (void)continuous_rx_stop();
                    continue;
                }
                const bool copied = xRingbufferSend(raw_ringbuf,
                                                    block.data,
                                                    block.length,
                                                    0) == pdTRUE;
                const esp_err_t release_result =
                    continuous_rx_release_block(block.index);
                if (release_result != ESP_OK) {
                    signal_capture_failure("DMA block release", release_result);
                    (void)continuous_rx_stop();
                }
                if (!copied) {
                    xSemaphoreTake(recording_mutex, portMAX_DELAY);
                    ++capture_status.raw_overflow_events;
                    capture_status.raw_overflow_bytes += block.length;
                    xSemaphoreGive(recording_mutex);
                    signal_capture_failure("raw PSRAM ring overflow",
                                           ESP_ERR_NO_MEM);
                    (void)continuous_rx_stop();
                }

                capture_status_t status;
                get_capture_status(&status);
                if (status.limit_reached || status.failed) {
                    (void)continuous_rx_stop();
                }
                continue;
            }

            continuous_rx_stats_t rx_stats;
            continuous_rx_get_stats(&rx_stats);
            if (rx_stats.fatal) {
                signal_capture_failure("Continuous SPI2/GDMA receive",
                                       ESP_ERR_INVALID_STATE);
                (void)continuous_rx_stop();
            }
            if (!rx_stats.running) {
                break;
            }
            if (receive_result != ESP_ERR_TIMEOUT) {
                signal_capture_failure("DMA completion queue receive",
                                       receive_result);
                (void)continuous_rx_stop();
            }
        }

        dma_session_done = true;
        xSemaphoreGive(dma_stopped_sem);
    }
}

static const char *frame_sync_error_name(frame_sync_error_t error)
{
    switch (error) {
    case FRAME_SYNC_ERROR_HEADER:
        return "locked frame header discontinuity";
    case FRAME_SYNC_ERROR_PADDING:
        return "locked frame zero-padding mismatch";
    case FRAME_SYNC_ERROR_OUTPUT:
        return "valid frame output overflow";
    default:
        return "frame synchronization error";
    }
}

static void frame_parser_task(void *parameter)
{
    (void)parameter;
    parser_output_context_t output = {
        .batch = parser_batch_storage,
        .batch_pos = 0,
        .total_frames = 0,
    };

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        output.batch_pos = 0;
        output.total_frames = 0;
        bool parser_failed = false;
        uint32_t processed_blocks = 0;

        while (true) {
            size_t item_size = 0;
            void *item = xRingbufferReceiveUpTo(
                raw_ringbuf,
                &item_size,
                pdMS_TO_TICKS(20),
                CONTINUOUS_RX_BLOCK_SIZE);
            if (item != NULL) {
                if (!parser_failed) {
                    const frame_sync_error_t sync_result = frame_sync_feed(
                        &stream_sync,
                        item,
                        item_size,
                        validated_frame_callback,
                        &output);
                    if (sync_result == FRAME_SYNC_ERROR_OUTPUT) {
                        parser_failed = true;
                        signal_capture_failure(
                            frame_sync_error_name(sync_result),
                            ESP_ERR_INVALID_RESPONSE);
                        (void)continuous_rx_stop();
                    }
                    if (!parser_failed && !flush_parser_batch(&output)) {
                        parser_failed = true;
                        signal_capture_failure("valid frame ring overflow",
                                               ESP_ERR_NO_MEM);
                        (void)continuous_rx_stop();
                    }
                }
                vRingbufferReturnItem(raw_ringbuf, item);
                publish_parser_progress(&output);
                ++processed_blocks;
                /* One 1 ms idle window per 256 KiB avoids starving IDLE0
                 * without imposing a delay on every 32 KiB DMA block. */
                if ((processed_blocks & 7U) == 0U) {
                    vTaskDelay(1);
                }
                continue;
            }

            if (dma_session_done) {
                break;
            }
        }

        if (!parser_failed && !flush_parser_batch(&output)) {
            signal_capture_failure("valid frame ring final flush",
                                   ESP_ERR_NO_MEM);
        }
        publish_parser_progress(&output);
        xSemaphoreGive(parser_stopped_sem);
    }
}

static bool finish_recording(recording_context_t *context, const char *reason);

static append_result_t drain_valid_ring(recording_context_t *context,
                                        TickType_t ticks_to_wait)
{
    size_t item_size = 0;
    void *item = xRingbufferReceive(
        data_ringbuf, &item_size, ticks_to_wait);
    if (item == NULL) {
        return APPEND_OK;
    }

    if (context->io_failed) {
        vRingbufferReturnItem(data_ringbuf, item);
        return APPEND_OK;
    }

    size_t consumed = 0;
    const append_result_t result = append_to_write_cache(
        context, item, item_size, &consumed);
    vRingbufferReturnItem(data_ringbuf, item);
    if (result == APPEND_OK && consumed != item_size) {
        signal_capture_failure("valid ring short consume", ESP_ERR_INVALID_SIZE);
        return APPEND_IO_ERROR;
    }
    return result;
}

static bool stop_pipeline_drain_and_finish(recording_context_t *context,
                                           const char *reason)
{
    bool wait_failed = false;
    bool parser_done = false;

    continuous_rx_stats_t rx_stats;
    continuous_rx_get_stats(&rx_stats);
    if (rx_stats.running || rx_stats.fatal) {
        const esp_err_t stop_result = continuous_rx_stop();
        if (stop_result != ESP_OK && stop_result != ESP_ERR_INVALID_STATE) {
            signal_capture_failure("Continuous receiver stop", stop_result);
            wait_failed = true;
        }
    }

    if (xSemaphoreTake(dma_stopped_sem,
                       pdMS_TO_TICKS(CAPTURE_STOP_TIMEOUT_MS)) != pdTRUE) {
        signal_capture_failure("DMA task stop timeout", ESP_ERR_TIMEOUT);
        wait_failed = true;
    }

    const int64_t deadline_us = esp_timer_get_time() +
        (int64_t)CAPTURE_STOP_TIMEOUT_MS * 1000;
    while (!parser_done && esp_timer_get_time() < deadline_us) {
        if (xSemaphoreTake(parser_stopped_sem, 0) == pdTRUE) {
            parser_done = true;
        }
        const append_result_t drain_result =
            drain_valid_ring(context, pdMS_TO_TICKS(10));
        if (drain_result == APPEND_IO_ERROR) {
            signal_capture_failure("eMMC write while draining", ESP_FAIL);
            wait_failed = true;
        }
    }
    if (!parser_done) {
        signal_capture_failure("Parser task drain timeout", ESP_ERR_TIMEOUT);
        wait_failed = true;
    }

    (void)wait_failed;
    return finish_recording(context, reason);
}

static const char *capture_outcome_name(raw_sd_capture_outcome_t outcome)
{
    switch (outcome) {
    case RAW_SD_CAPTURE_CLEAN:
        return "CLEAN";
    case RAW_SD_CAPTURE_CLOSED_WITH_GAPS:
        return "CLOSED_WITH_GAPS";
    case RAW_SD_CAPTURE_CLOSED_UNCERTAIN_SYNC:
        return "CLOSED_UNCERTAIN_SYNC";
    case RAW_SD_CAPTURE_FAILED_UNRESOLVED_SYNC:
        return "FAILED_UNRESOLVED_SYNC";
    case RAW_SD_CAPTURE_FAILED_AMBIGUOUS_SYNC:
        return "FAILED_AMBIGUOUS_SYNC";
    case RAW_SD_CAPTURE_FAILED_PIPELINE:
    default:
        return "FAILED_PIPELINE";
    }
}

static bool finish_recording(recording_context_t *context, const char *reason)
{
    set_capture_enabled(false);
    bool drain_ok = !context->io_failed;
    size_t item_size = 0;
    void *item = NULL;
    while ((item = xRingbufferReceive(data_ringbuf, &item_size, 0)) != NULL) {
        if (drain_ok) {
            size_t consumed = 0;
            const append_result_t result = append_to_write_cache(
                context, (const uint8_t *)item, item_size, &consumed);
            if (result == APPEND_IO_ERROR ||
                (result == APPEND_OK && consumed != item_size)) {
                drain_ok = false;
            }
        }
        vRingbufferReturnItem(data_ringbuf, item);
    }

    capture_status_t status;
    get_capture_status(&status);
    continuous_rx_stats_t rx_stats;
    continuous_rx_get_stats(&rx_stats);
    const frame_sync_stats_t *sync_stats = frame_sync_get_stats(&stream_sync);
    const frame_sync_status_t sync_status =
        frame_sync_get_status(&stream_sync);
    const bool frame_aligned =
        (context->written_bytes % ADC_FRAME_SIZE_BYTES) == 0U;

    raw_sd_capture_outcome_t outcome;
    if (!drain_ok || status.failed || rx_stats.fatal || context->io_failed ||
        !frame_aligned || status.dma_sequence_gaps != 0U) {
        outcome = RAW_SD_CAPTURE_FAILED_PIPELINE;
    } else if (sync_status.state != FRAME_SYNC_LOCKED ||
               !sync_status.ever_locked) {
        outcome = RAW_SD_CAPTURE_FAILED_UNRESOLVED_SYNC;
    } else if (sync_status.uncertain_lock_seen) {
        outcome = RAW_SD_CAPTURE_CLOSED_UNCERTAIN_SYNC;
    } else if (sync_status.had_gaps) {
        outcome = RAW_SD_CAPTURE_CLOSED_WITH_GAPS;
    } else {
        outcome = RAW_SD_CAPTURE_CLEAN;
    }
    bool success = outcome == RAW_SD_CAPTURE_CLEAN ||
                   outcome == RAW_SD_CAPTURE_CLOSED_WITH_GAPS ||
                   outcome == RAW_SD_CAPTURE_CLOSED_UNCERTAIN_SYNC;
    const uint64_t valid_bytes = context->written_bytes -
        (context->written_bytes % ADC_FRAME_SIZE_BYTES);
    const raw_sd_segment_diagnostics_t diagnostics = {
        .outcome = outcome,
        .final_sync_state = (uint32_t)sync_status.state,
        .events = sync_event_storage,
        .event_count = sync_event_count,
        .event_overflow = sync_event_overflow,
        .resync_events = sync_stats != NULL ? sync_stats->resync_events : 0U,
        .resync_discarded_bytes = sync_stats != NULL
                                      ? sync_stats->resync_discarded_bytes
                                      : 0U,
        .last_resync_start_bit = sync_stats != NULL
                                     ? sync_stats->last_resync_start_bit
                                     : 0U,
        .last_resync_lock_bit = sync_stats != NULL
                                    ? sync_stats->last_resync_lock_bit
                                    : 0U,
        .dma_last_sequence = status.last_dma_sequence,
        .dma_sequence_gaps = status.dma_sequence_gaps,
    };
    const esp_err_t metadata_result = raw_sd_recorder_close_segment(
        &raw_recorder,
        success ? RAW_SD_SEGMENT_CLOSED : RAW_SD_SEGMENT_FAILED,
        success ? ESP_OK : ESP_FAIL,
        &diagnostics);
    if (metadata_result != ESP_OK) {
        success = false;
        outcome = RAW_SD_CAPTURE_FAILED_PIPELINE;
        ESP_LOGE(TAG, "Raw eMMC segment metadata update failed: 0x%x (%s)",
                 (unsigned int)metadata_result,
                 esp_err_to_name(metadata_result));
    }

    const uint64_t confirmed_valid_bytes =
        raw_recorder.active_segment.valid_bytes;
    ESP_LOGI(TAG, "Recording stopped: %s", reason);
    ESP_LOGI(TAG,
             "Final=%s | Sync=%s | CompleteFrames=%" PRIu64
             " | ValidBytes=%" PRIu64,
             capture_outcome_name(outcome),
             frame_sync_state_name(sync_status.state),
             confirmed_valid_bytes / ADC_FRAME_SIZE_BYTES,
             confirmed_valid_bytes);
    if (confirmed_valid_bytes != valid_bytes) {
        ESP_LOGW(TAG,
                 "Accepted=%" PRIu64 ", but only %" PRIu64
                 " complete-frame bytes were confirmed on eMMC",
                 valid_bytes, confirmed_valid_bytes);
    }
    ESP_LOGI(TAG,
             "DMA blocks=%" PRIu64 ", last_sequence=%" PRIu64
             ", sequence_gaps=%" PRIu32 ", overrun=%" PRIu64
             ", descriptor_errors=%" PRIu64 ", unexpected_eof=%" PRIu64,
             rx_stats.completed_blocks, status.last_dma_sequence,
             status.dma_sequence_gaps, rx_stats.overruns,
             rx_stats.descriptor_errors, rx_stats.unexpected_eof_errors);
    ESP_LOGI(TAG,
             "SyncFrames=%" PRIu64 ", header_errors=%" PRIu64
             ", padding_errors=%" PRIu64 ", resync_events=%" PRIu64
             ", discarded=%" PRIu64 " bytes, metadata_events=%" PRIu32
             ", event_overflow=%" PRIu32,
             sync_stats != NULL ? sync_stats->frames_output : 0U,
             sync_stats != NULL ? sync_stats->header_errors : 0U,
             sync_stats != NULL ? sync_stats->padding_errors : 0U,
             sync_stats != NULL ? sync_stats->resync_events : 0U,
             sync_stats != NULL ? sync_stats->resync_discarded_bytes : 0U,
             sync_event_count, sync_event_overflow);
    if (!success && status.failure_reason[0] != '\0') {
        ESP_LOGE(TAG, "Failure detail: %s", status.failure_reason);
    }
    last_capture_outcome = (uint32_t)outcome;
    last_sync_state = (uint32_t)sync_status.state;
    return success;
}

static void log_write_rate(recording_context_t *context,
                           int64_t *last_log_time_us)
{
    const int64_t now_us = esp_timer_get_time();
    const int64_t elapsed_us = now_us - *last_log_time_us;
    if (elapsed_us < (int64_t)RATE_LOG_INTERVAL_US) {
        return;
    }

    const double seconds = (double)elapsed_us / 1000000.0;
    capture_status_t status;
    get_capture_status(&status);
    continuous_rx_stats_t rx_stats;
    continuous_rx_get_stats(&rx_stats);
    const frame_sync_status_t sync_status =
        frame_sync_get_status(&stream_sync);
    const uint64_t physical_bytes = raw_recorder.active_segment.physical_bytes;
    const uint64_t input_delta = status.raw_input_bytes -
        context->last_input_bytes;
    const uint64_t valid_delta_frames = status.accepted_frames -
        context->last_valid_frames;
    const uint64_t physical_delta = physical_bytes -
        context->last_physical_bytes;
    const double adc_rate =
        ((double)input_delta / (1024.0 * 1024.0)) / seconds;
    const double valid_rate =
        ((double)(valid_delta_frames * ADC_FRAME_SIZE_BYTES) /
         (1024.0 * 1024.0)) / seconds;
    const double emmc_rate =
        ((double)physical_delta / (1024.0 * 1024.0)) / seconds;

    ESP_LOGI(TAG,
             "ADC=%.2f MiB/s | Valid=%.2f MiB/s | eMMC=%.2f MiB/s"
             " | Sync=%s | Candidates=%u | Uncertain=%s"
             " | frames=%" PRIu64 " | valid=%" PRIu64
             " | raw_free=%zu | valid_free=%zu | dma_overrun=%" PRIu64
             " | DMA_seq=%" PRIu64 " | raw_overflow=%" PRIu64
             ", valid_overflow=%" PRIu64,
             adc_rate, valid_rate, emmc_rate,
             frame_sync_state_name(sync_status.state),
             sync_status.state == FRAME_SYNC_LOCKED
                 ? sync_status.last_lock_candidates
                 : sync_status.active_candidates,
             sync_status.uncertain_lock_seen ? "yes" : "no",
             status.accepted_frames,
             context->written_bytes,
             xRingbufferGetCurFreeSize(raw_ringbuf),
             xRingbufferGetCurFreeSize(data_ringbuf),
             rx_stats.overruns,
             status.last_dma_sequence,
             status.raw_overflow_bytes,
             status.valid_overflow_frames);

    context->interval_written_bytes = 0;
    context->last_input_bytes = status.raw_input_bytes;
    context->last_valid_frames = status.accepted_frames;
    context->last_physical_bytes = physical_bytes;
    *last_log_time_us = now_us;
}

static void close_failed_start(recording_context_t *context)
{
    if (raw_recorder.segment_open) {
        (void)raw_sd_recorder_close_segment(&raw_recorder,
                                            RAW_SD_SEGMENT_FAILED,
                                            ESP_FAIL,
                                            NULL);
    }
    set_capture_enabled(false);
}

static void set_state(emmc_state_t state, esp_err_t failure)
{
    xSemaphoreTake(status_mutex, portMAX_DELAY);
    current_status.state = state;
    current_status.failure_code = failure;
    current_status.card_ready = raw_recorder.card_initialized &&
                                state != EMMC_STATE_ERROR;
    xSemaphoreGive(status_mutex);
}

static void update_status_progress(void)
{
    capture_status_t capture;
    get_capture_status(&capture);
    continuous_rx_stats_t rx_stats;
    continuous_rx_get_stats(&rx_stats);
    const uint64_t now_us = (uint64_t)esp_timer_get_time();

    xSemaphoreTake(status_mutex, portMAX_DELAY);
    current_status.physical_bytes =
        raw_recorder.active_segment.physical_bytes;
    current_status.valid_bytes = raw_recorder.segment_open
        ? manager_recording_context.written_bytes
        : raw_recorder.active_segment.valid_bytes;
    current_status.frame_count = current_status.valid_bytes /
                                 ADC_FRAME_SIZE_BYTES;
    current_status.raw_input_bytes = capture.raw_input_bytes;
    current_status.dma_blocks = rx_stats.completed_blocks;
    current_status.dma_overruns = rx_stats.overruns;
    if (recording_start_us > 0) {
        current_status.wall_elapsed_us = now_us -
            (uint64_t)recording_start_us;
        current_status.write_elapsed_us = current_status.wall_elapsed_us;
    }
    current_status.capture_outcome = last_capture_outcome;
    current_status.sync_state = last_sync_state;
    xSemaphoreGive(status_mutex);
}

static esp_err_t submit_request(storage_request_t *request)
{
    if (request_queue == NULL || request == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t result = ESP_FAIL;
    request->waiter = xTaskGetCurrentTaskHandle();
    request->result = &result;
    (void)ulTaskNotifyTake(pdTRUE, 0);
    if (xQueueSend(request_queue, request, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    return result;
}

static void finish_request(const storage_request_t *request, esp_err_t result)
{
    *request->result = result;
    xTaskNotifyGive(request->waiter);
}

static void release_card(void)
{
    raw_sd_recorder_deinit(&raw_recorder);
    run_started = false;
    xSemaphoreTake(status_mutex, portMAX_DELAY);
    current_status.card_ready = false;
    current_status.capacity_sectors = 0U;
    current_status.emmc_clock_khz = 0U;
    xSemaphoreGive(status_mutex);
}

static esp_err_t prepare_card(void)
{
    const esp_err_t result = init_emmc_4bit();
    if (result != ESP_OK) {
        return result;
    }
    xSemaphoreTake(status_mutex, portMAX_DELAY);
    current_status.capacity_sectors =
        (uint64_t)raw_recorder.card.csd.capacity;
    current_status.emmc_clock_khz =
        (uint32_t)raw_recorder.card.real_freq_khz;
    current_status.card_ready = true;
    xSemaphoreGive(status_mutex);
    return ESP_OK;
}

static esp_err_t start_capture(void)
{
    if (!run_started) {
        const esp_err_t begin_result =
            raw_sd_recorder_begin_run(&raw_recorder);
        if (begin_result != ESP_OK) {
            return begin_result;
        }
        run_started = true;
    }
    if (raw_sd_recorder_run_is_full(&raw_recorder)) {
        return ESP_ERR_INVALID_SIZE;
    }

    const uint64_t stale_bytes = discard_pipeline_buffers();
    if (stale_bytes > 0U) {
        ESP_LOGW(TAG, "Discarded %" PRIu64 " stale pipeline bytes",
                 stale_bytes);
    }
    if (!open_raw_segment(&manager_recording_context)) {
        return ESP_FAIL;
    }

    while (xSemaphoreTake(dma_stopped_sem, 0) == pdTRUE) {
    }
    while (xSemaphoreTake(parser_stopped_sem, 0) == pdTRUE) {
    }
    frame_sync_reset(&stream_sync);
    reset_capture_status();
    dma_session_done = false;
    stop_requested = false;
    last_capture_outcome = 0U;
    last_sync_state = (uint32_t)FRAME_SYNC_ACQUIRE;

    const esp_err_t result = continuous_rx_start();
    if (result != ESP_OK) {
        signal_capture_failure("Continuous receiver start", result);
        close_failed_start(&manager_recording_context);
        return result;
    }
    xTaskNotifyGive(parser_task_handle);
    xTaskNotifyGive(dma_task_handle);
    recording_start_us = esp_timer_get_time();

    xSemaphoreTake(status_mutex, portMAX_DELAY);
    current_status.result_available = false;
    current_status.physical_bytes = 0U;
    current_status.valid_bytes = 0U;
    current_status.frame_count = 0U;
    current_status.raw_input_bytes = 0U;
    current_status.dma_blocks = 0U;
    current_status.dma_overruns = 0U;
    current_status.write_elapsed_us = 0U;
    current_status.wall_elapsed_us = 0U;
    current_status.capture_outcome = 0U;
    current_status.sync_state = (uint32_t)FRAME_SYNC_ACQUIRE;
    xSemaphoreGive(status_mutex);
    set_state(EMMC_STATE_WRITING, ESP_OK);
    ESP_LOGI(TAG,
             "ADC CAPTURE STARTED: header=FFFF0000, frame=%u bytes",
             ADC_FRAME_SIZE_BYTES);
    return ESP_OK;
}

static esp_err_t handle_request(const storage_request_t *request)
{
    emmc_status_t snapshot;
    (void)emmc_storage_get_status(&snapshot);

    switch (request->type) {
    case STORAGE_REQUEST_START_WRITE: {
        if (!snapshot.card_ready || snapshot.state == EMMC_STATE_ERROR) {
            return EMMC_STORAGE_ERR_CARD;
        }
        if (snapshot.state != EMMC_STATE_IDLE) {
            return EMMC_STORAGE_ERR_BUSY;
        }
        if (!write_switch_is_high()) {
            return EMMC_STORAGE_ERR_INTERLOCK;
        }
        const esp_err_t result = start_capture();
        if (result != ESP_OK) {
            set_state(EMMC_STATE_ERROR, result);
        }
        return result;
    }
    case STORAGE_REQUEST_STOP_WRITE:
        if (snapshot.state == EMMC_STATE_WRITING) {
            stop_requested = true;
        }
        return ESP_OK;
    case STORAGE_REQUEST_READ: {
        if (!snapshot.card_ready || snapshot.state == EMMC_STATE_ERROR) {
            return EMMC_STORAGE_ERR_CARD;
        }
        if (snapshot.state != EMMC_STATE_IDLE) {
            return EMMC_STORAGE_ERR_BUSY;
        }
        if (write_switch_is_high()) {
            return EMMC_STORAGE_ERR_INTERLOCK;
        }
        set_state(EMMC_STATE_READING, ESP_OK);
        emmc_storage_access_t access = {
            .card = &raw_recorder.card,
            .dma_buffer = read_dma_buffer,
            .dma_buffer_sectors = STORAGE_READ_DMA_SECTORS,
            .card_error = ESP_OK,
        };
        const esp_err_t result = request->operation(&access, request->context);
        if (access.card_error != ESP_OK) {
            set_state(EMMC_STATE_ERROR, access.card_error);
        } else {
            set_state(EMMC_STATE_IDLE, ESP_OK);
        }
        return result;
    }
    case STORAGE_REQUEST_REINIT: {
        if (snapshot.state == EMMC_STATE_WRITING ||
            snapshot.state == EMMC_STATE_FINALIZING ||
            snapshot.state == EMMC_STATE_READING) {
            return EMMC_STORAGE_ERR_BUSY;
        }
        if (write_switch_is_high()) {
            return EMMC_STORAGE_ERR_INTERLOCK;
        }
        release_card();
        const esp_err_t result = prepare_card();
        if (result == ESP_OK) {
            set_state(EMMC_STATE_IDLE, ESP_OK);
        } else {
            set_state(EMMC_STATE_ERROR, result);
        }
        return result;
    }
    default:
        return ESP_ERR_INVALID_ARG;
    }
}

static void storage_manager_task(void *parameter)
{
    (void)parameter;
    storage_request_t request;
    int64_t last_log_time_us = 0;

    for (;;) {
        emmc_status_t snapshot;
        (void)emmc_storage_get_status(&snapshot);

        if (snapshot.state == EMMC_STATE_WRITING) {
            if (xQueueReceive(request_queue, &request, 0) == pdTRUE) {
                const esp_err_t result = handle_request(&request);
                finish_request(&request, result);
            }

            capture_status_t capture;
            get_capture_status(&capture);
            if (stop_requested || capture.failed || capture.limit_reached ||
                raw_sd_recorder_run_is_full(&raw_recorder)) {
                set_state(EMMC_STATE_FINALIZING,
                          capture.failed ? ESP_FAIL : ESP_OK);
                continue;
            }

            size_t item_size = 0U;
            void *item = xRingbufferReceive(
                data_ringbuf, &item_size, pdMS_TO_TICKS(50));
            if (item != NULL) {
                size_t consumed = 0U;
                const append_result_t append_result = append_to_write_cache(
                    &manager_recording_context, item, item_size, &consumed);
                vRingbufferReturnItem(data_ringbuf, item);
                if (append_result == APPEND_FILE_LIMIT) {
                    set_state(EMMC_STATE_FINALIZING, ESP_OK);
                    continue;
                }
                if (append_result == APPEND_IO_ERROR ||
                    consumed != item_size) {
                    signal_capture_failure("raw eMMC write/cache", ESP_FAIL);
                    set_state(EMMC_STATE_FINALIZING, ESP_FAIL);
                    continue;
                }
            }
            log_write_rate(&manager_recording_context, &last_log_time_us);
            update_status_progress();
            continue;
        }

        if (snapshot.state == EMMC_STATE_FINALIZING) {
            capture_status_t capture;
            get_capture_status(&capture);
            const char *reason = capture.failed
                ? "capture failure"
                : stop_requested ? "GPIO7 became low"
                                 : "complete-frame 1 GiB limit reached";
            const bool success = stop_pipeline_drain_and_finish(
                &manager_recording_context, reason);
            update_status_progress();
            xSemaphoreTake(status_mutex, portMAX_DELAY);
            current_status.result_available = true;
            xSemaphoreGive(status_mutex);
            stop_requested = false;
            set_state(success ? EMMC_STATE_IDLE : EMMC_STATE_ERROR,
                      success ? ESP_OK : ESP_FAIL);
            continue;
        }

        if (xQueueReceive(request_queue, &request, portMAX_DELAY) == pdTRUE) {
            const esp_err_t result = handle_request(&request);
            finish_request(&request, result);
            if (result == ESP_OK && request.type == STORAGE_REQUEST_START_WRITE) {
                last_log_time_us = recording_start_us;
            }
        }
    }
}

esp_err_t emmc_storage_manager_init(void)
{
    if (request_queue != NULL || status_mutex != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(TAG, "ESP32-S3 continuous external-clock ADC logger starting");
    ESP_LOGI(TAG,
             "Capture input: CLK=GPIO%d, DATA=GPIO%d, %u Hz",
             CONTINUOUS_RX_CLK_GPIO,
             CONTINUOUS_RX_DATA_GPIO,
             CONTINUOUS_RX_CLOCK_HZ);
    ESP_LOGI(TAG,
             "Capture mode: external clock, rising-edge sample, MSB first, no external CS");
    ESP_LOGI(TAG,
             "Frame: %u bytes/%u bits, header=FFFF0000, "
             "64 x (16-bit sample + 0000 padding), no counter",
             ADC_FRAME_SIZE_BYTES,
             ADC_FRAME_SIZE_BITS);
    ESP_LOGW(TAG,
             "DEDICATED RAW eMMC MODE: eMMC user-area LBA0/LBA1 and the raw "
             "data area will be overwritten");
    ESP_LOGI(TAG,
             "Raw eMMC user-area layout: metadata LBA0..2047, data starts "
             "LBA2048, capacity=1 GiB");

    const esp_err_t stimulus_result = stim_controller_init();
    if (stimulus_result != ESP_OK) {
        ESP_LOGE(TAG,
                 "Stimulus port initialization failed: 0x%x (%s); "
                 "continuing ADC/eMMC capture",
                 (unsigned int)stimulus_result,
                 esp_err_to_name(stimulus_result));
    }

    status_mutex = xSemaphoreCreateMutex();
    request_queue = xQueueCreate(STORAGE_QUEUE_LENGTH,
                                 sizeof(storage_request_t));
    recording_mutex = xSemaphoreCreateMutex();
    dma_stopped_sem = xSemaphoreCreateBinary();
    parser_stopped_sem = xSemaphoreCreateBinary();
    if (status_mutex == NULL || request_queue == NULL ||
        recording_mutex == NULL || dma_stopped_sem == NULL ||
        parser_stopped_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create synchronization objects");
        return ESP_ERR_NO_MEM;
    }
    memset(&current_status, 0, sizeof(current_status));
    current_status.state = EMMC_STATE_ERROR;
    current_status.target_frames = MAX_RECORD_FRAMES;
    current_status.target_bytes =
        (uint64_t)RAW_SD_DATA_CAPACITY_SECTORS * RAW_SD_SECTOR_BYTES;

    StaticRingbuffer_t *raw_ring_structure = heap_caps_calloc(
        1, sizeof(StaticRingbuffer_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    StaticRingbuffer_t *valid_ring_structure = heap_caps_calloc(
        1, sizeof(StaticRingbuffer_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    uint8_t *raw_ring_storage = heap_caps_malloc(
        RAW_RING_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint8_t *valid_ring_storage = heap_caps_malloc(
        VALID_RING_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (raw_ring_structure == NULL || valid_ring_structure == NULL ||
        raw_ring_storage == NULL || valid_ring_storage == NULL) {
        ESP_LOGE(TAG, "Failed to allocate 2 MiB raw + 12 MiB valid PSRAM rings");
        heap_caps_free(raw_ring_structure);
        heap_caps_free(valid_ring_structure);
        heap_caps_free(raw_ring_storage);
        heap_caps_free(valid_ring_storage);
        return ESP_ERR_NO_MEM;
    }

    raw_ringbuf = xRingbufferCreateStatic(
        RAW_RING_BUFFER_SIZE,
        RINGBUF_TYPE_BYTEBUF,
        raw_ring_storage,
        raw_ring_structure);
    data_ringbuf = xRingbufferCreateStatic(
        VALID_RING_BUFFER_SIZE,
        RINGBUF_TYPE_NOSPLIT,
        valid_ring_storage,
        valid_ring_structure);
    if (raw_ringbuf == NULL || data_ringbuf == NULL) {
        ESP_LOGE(TAG, "Failed to create PSRAM pipeline rings");
        return ESP_ERR_NO_MEM;
    }

    const size_t sync_workspace_bytes = frame_sync_workspace_size();
    frame_sync_workspace = heap_caps_calloc(
        1U, sync_workspace_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    sync_event_storage = heap_caps_calloc(
        SYNC_EVENT_CAPACITY, sizeof(raw_sd_sync_event_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (frame_sync_workspace == NULL || sync_event_storage == NULL ||
        !frame_sync_init(&stream_sync, frame_sync_workspace,
                         sync_workspace_bytes)) {
        ESP_LOGE(TAG,
                 "Failed to allocate synchronizer/event PSRAM: workspace=%zu, "
                 "events=%u",
                 sync_workspace_bytes, (unsigned int)SYNC_EVENT_CAPACITY);
        heap_caps_free(frame_sync_workspace);
        heap_caps_free(sync_event_storage);
        return ESP_ERR_NO_MEM;
    }
    frame_sync_set_event_callback(
        &stream_sync, frame_sync_event_callback, NULL);

    esp_err_t result = continuous_rx_init();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Continuous receiver initialization failed: 0x%x (%s)",
                 (unsigned int)result, esp_err_to_name(result));
        return result;
    }
    ESP_LOGI(TAG, "Continuous GDMA ring ready: %u x %u bytes",
             CONTINUOUS_RX_BLOCK_COUNT, CONTINUOUS_RX_BLOCK_SIZE);
    ESP_LOGI(TAG,
             "FreeRTOS pipeline: DMA(CPU0/P20) -> raw 2 MiB -> "
             "parser(CPU0/P10) -> valid 12 MiB -> eMMC(CPU1/P6)");

    parser_batch_storage = heap_caps_malloc(
        FRAME_BATCH_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (parser_batch_storage == NULL) {
        ESP_LOGE(TAG, "Failed to allocate %u-byte parser batch",
                 (unsigned int)FRAME_BATCH_SIZE);
        continuous_rx_deinit();
        return ESP_ERR_NO_MEM;
    }

    read_dma_buffer = heap_caps_malloc(
        STORAGE_READ_DMA_SECTORS * RAW_SD_SECTOR_BYTES,
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (read_dma_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate UART read DMA buffer");
        continuous_rx_deinit();
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreatePinnedToCore(
            adc_dma_task,
            "ADC_DMA_TASK",
            6144,
            NULL,
            20,
            &dma_task_handle,
            0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ADC DMA task");
        continuous_rx_deinit();
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreatePinnedToCore(
            frame_parser_task,
            "FRAME_PARSER_TASK",
            8192,
            NULL,
            10,
            &parser_task_handle,
            0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create frame parser task");
        vTaskDelete(dma_task_handle);
        dma_task_handle = NULL;
        continuous_rx_deinit();
        return ESP_ERR_NO_MEM;
    }

    const esp_err_t card_result = prepare_card();
    if (card_result == ESP_OK) {
        set_state(EMMC_STATE_IDLE, ESP_OK);
    } else {
        set_state(EMMC_STATE_ERROR, card_result);
        ESP_LOGE(TAG, "eMMC initialization failed: 0x%x (%s)",
                 (unsigned int)card_result, esp_err_to_name(card_result));
    }

    if (xTaskCreatePinnedToCore(
            storage_manager_task,
            "EMMC_STORAGE",
            8192,
            NULL,
            6,
            NULL,
            1) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create eMMC storage manager task");
        release_card();
        vTaskDelete(parser_task_handle);
        parser_task_handle = NULL;
        vTaskDelete(dma_task_handle);
        dma_task_handle = NULL;
        continuous_rx_deinit();
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG,
             "Unified ownership ready: IDLE/WRITING/FINALIZING/READING/ERROR");
    return ESP_OK;
}

esp_err_t emmc_storage_get_status(emmc_status_t *status)
{
    if (status == NULL || status_mutex == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(status_mutex, portMAX_DELAY);
    *status = current_status;
    xSemaphoreGive(status_mutex);
    status->gpio7_high = write_switch_is_high();
    status->write_armed = write_switch_is_armed();
    return ESP_OK;
}

esp_err_t emmc_storage_execute_read(emmc_storage_read_operation_t operation,
                                    void *context)
{
    if (operation == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    emmc_status_t status;
    (void)emmc_storage_get_status(&status);
    if (!status.card_ready || status.state == EMMC_STATE_ERROR) {
        return EMMC_STORAGE_ERR_CARD;
    }
    if (status.state != EMMC_STATE_IDLE) {
        return EMMC_STORAGE_ERR_BUSY;
    }
    if (write_switch_is_high()) {
        return EMMC_STORAGE_ERR_INTERLOCK;
    }
    storage_request_t request = {
        .type = STORAGE_REQUEST_READ,
        .operation = operation,
        .context = context,
    };
    return submit_request(&request);
}

esp_err_t emmc_storage_request_write_start(void)
{
    emmc_status_t status;
    (void)emmc_storage_get_status(&status);
    if (!status.card_ready || status.state == EMMC_STATE_ERROR) {
        return EMMC_STORAGE_ERR_CARD;
    }
    if (status.state != EMMC_STATE_IDLE) {
        return EMMC_STORAGE_ERR_BUSY;
    }
    if (!write_switch_is_high()) {
        return EMMC_STORAGE_ERR_INTERLOCK;
    }
    storage_request_t request = {.type = STORAGE_REQUEST_START_WRITE};
    return submit_request(&request);
}

esp_err_t emmc_storage_request_write_stop(void)
{
    emmc_status_t status;
    (void)emmc_storage_get_status(&status);
    if (status.state != EMMC_STATE_WRITING) {
        return ESP_OK;
    }
    storage_request_t request = {.type = STORAGE_REQUEST_STOP_WRITE};
    return submit_request(&request);
}

esp_err_t emmc_storage_request_reinit(void)
{
    emmc_status_t status;
    (void)emmc_storage_get_status(&status);
    if (status.state == EMMC_STATE_WRITING ||
        status.state == EMMC_STATE_FINALIZING ||
        status.state == EMMC_STATE_READING) {
        return EMMC_STORAGE_ERR_BUSY;
    }
    if (write_switch_is_high()) {
        return EMMC_STORAGE_ERR_INTERLOCK;
    }
    storage_request_t request = {.type = STORAGE_REQUEST_REINIT};
    return submit_request(&request);
}

const char *emmc_storage_state_name(emmc_state_t state)
{
    switch (state) {
    case EMMC_STATE_IDLE: return "IDLE";
    case EMMC_STATE_WRITING: return "WRITING";
    case EMMC_STATE_FINALIZING: return "FINALIZING";
    case EMMC_STATE_READING: return "READING";
    case EMMC_STATE_ERROR: return "ERROR";
    default: return "UNKNOWN";
    }
}
