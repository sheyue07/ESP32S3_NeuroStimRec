#include "adc_preview.h"

#include <stdbool.h>
#include <string.h>

#include "esp_timer.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define ADC_PREVIEW_INPUT_FRAME_HZ 14423U
#define ADC_PREVIEW_FRAME_BYTES 260U

typedef struct {
    portMUX_TYPE lock;
    QueueHandle_t queue;
    StaticQueue_t queue_storage;
    uint8_t queue_bytes[ADC_PREVIEW_QUEUE_DEPTH * sizeof(adc_preview_record_t)];
    bool enabled;
    uint8_t channel_count;
    uint8_t channels[ADC_PREVIEW_MAX_CHANNELS];
    uint16_t target_hz;
    uint16_t actual_hz;
    uint32_t decimation;
    uint32_t generation;
    /* The fields below are used only by the storage task. Keeping a local
     * snapshot avoids a cross-core critical section for every frame. */
    uint32_t ingest_generation;
    uint32_t ingest_phase;
    uint32_t ingest_decimation;
    bool ingest_enabled;
    uint8_t ingest_channel_count;
    uint8_t ingest_channels[ADC_PREVIEW_MAX_CHANNELS];
    uint64_t produced;
    uint64_t dropped;
} adc_preview_context_t;

static adc_preview_context_t s_preview = {
    .lock = portMUX_INITIALIZER_UNLOCKED,
};

esp_err_t adc_preview_init(void)
{
    if (s_preview.queue != NULL) {
        return ESP_OK;
    }
    s_preview.queue = xQueueCreateStatic(
        ADC_PREVIEW_QUEUE_DEPTH,
        sizeof(adc_preview_record_t),
        s_preview.queue_bytes,
        &s_preview.queue_storage);
    return s_preview.queue != NULL ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t adc_preview_configure(const uint8_t *channels,
                                size_t channel_count,
                                uint16_t target_hz,
                                uint16_t *actual_hz)
{
    if (s_preview.queue == NULL || channel_count > ADC_PREVIEW_MAX_CHANNELS ||
        (channel_count != 0U && channels == NULL) || target_hz == 0U ||
        target_hz > 200U) {
        return ESP_ERR_INVALID_ARG;
    }
    uint64_t selected = 0U;
    for (size_t index = 0U; index < channel_count; ++index) {
        if (channels[index] > 63U ||
            (selected & (UINT64_C(1) << channels[index])) != 0U) {
            return ESP_ERR_INVALID_ARG;
        }
        selected |= UINT64_C(1) << channels[index];
    }
    uint32_t decimation = (ADC_PREVIEW_INPUT_FRAME_HZ + target_hz / 2U) /
                          target_hz;
    if (decimation == 0U) {
        decimation = 1U;
    }
    const uint16_t configured_hz =
        (uint16_t)(ADC_PREVIEW_INPUT_FRAME_HZ / decimation);

    portENTER_CRITICAL(&s_preview.lock);
    s_preview.channel_count = (uint8_t)channel_count;
    if (channel_count != 0U) {
        memcpy(s_preview.channels, channels, channel_count);
    }
    s_preview.target_hz = target_hz;
    s_preview.actual_hz = configured_hz;
    s_preview.decimation = decimation;
    s_preview.enabled = channel_count != 0U;
    (void)__atomic_add_fetch(&s_preview.generation, 1U, __ATOMIC_RELEASE);
    portEXIT_CRITICAL(&s_preview.lock);
    xQueueReset(s_preview.queue);
    if (actual_hz != NULL) {
        *actual_hz = configured_hz;
    }
    return ESP_OK;
}

void adc_preview_disable(void)
{
    portENTER_CRITICAL(&s_preview.lock);
    s_preview.enabled = false;
    s_preview.channel_count = 0U;
    (void)__atomic_add_fetch(&s_preview.generation, 1U, __ATOMIC_RELEASE);
    portEXIT_CRITICAL(&s_preview.lock);
    if (s_preview.queue != NULL) {
        xQueueReset(s_preview.queue);
    }
}

void adc_preview_ingest_batch(const uint8_t *frames,
                              size_t frame_count,
                              uint64_t first_frame_index)
{
    if (frames == NULL || frame_count == 0U || s_preview.queue == NULL) {
        return;
    }
    const uint32_t generation = __atomic_load_n(
        &s_preview.generation, __ATOMIC_ACQUIRE);
    if (generation != s_preview.ingest_generation) {
        portENTER_CRITICAL(&s_preview.lock);
        s_preview.ingest_generation = s_preview.generation;
        s_preview.ingest_enabled = s_preview.enabled;
        s_preview.ingest_decimation = s_preview.decimation;
        s_preview.ingest_channel_count = s_preview.channel_count;
        memcpy(s_preview.ingest_channels, s_preview.channels,
               s_preview.channel_count);
        s_preview.ingest_phase = 0U;
        portEXIT_CRITICAL(&s_preview.lock);
    }
    if (!s_preview.ingest_enabled || s_preview.ingest_decimation == 0U) {
        return;
    }

    const uint32_t decimation = s_preview.ingest_decimation;
    const uint32_t initial_phase = s_preview.ingest_phase;
    size_t selected_offset = (size_t)(decimation - initial_phase - 1U);
    while (selected_offset < frame_count) {
        const uint8_t *const frame =
            frames + selected_offset * ADC_PREVIEW_FRAME_BYTES;
        adc_preview_record_t record = {
            .frame_index = first_frame_index + selected_offset,
            .timestamp_us = (uint64_t)esp_timer_get_time(),
            .channel_count = s_preview.ingest_channel_count,
        };
        for (size_t index = 0U;
             index < s_preview.ingest_channel_count; ++index) {
            const uint8_t channel = s_preview.ingest_channels[index];
            const size_t offset = 4U + (size_t)channel * 4U;
            record.samples[index].channel = channel;
            record.samples[index].value =
                ((uint16_t)frame[offset] << 8U) | frame[offset + 1U];
        }
        const bool queued =
            xQueueSend(s_preview.queue, &record, 0) == pdTRUE;
        portENTER_CRITICAL(&s_preview.lock);
        if (queued) {
            ++s_preview.produced;
        } else {
            ++s_preview.dropped;
        }
        portEXIT_CRITICAL(&s_preview.lock);
        selected_offset += decimation;
    }
    s_preview.ingest_phase =
        (uint32_t)(((uint64_t)initial_phase + frame_count) % decimation);
}

bool adc_preview_receive(adc_preview_record_t *record, TickType_t wait_ticks)
{
    return record != NULL && s_preview.queue != NULL &&
           xQueueReceive(s_preview.queue, record, wait_ticks) == pdTRUE;
}

void adc_preview_get_status(adc_preview_status_t *status)
{
    if (status == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_preview.lock);
    *status = (adc_preview_status_t){
        .enabled = s_preview.enabled,
        .channel_count = s_preview.channel_count,
        .target_hz = s_preview.target_hz,
        .actual_hz = s_preview.actual_hz,
        .produced = s_preview.produced,
        .dropped = s_preview.dropped,
    };
    memcpy(status->channels, s_preview.channels, s_preview.channel_count);
    portEXIT_CRITICAL(&s_preview.lock);
}
