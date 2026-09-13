#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ADC_PREVIEW_MAX_CHANNELS 8U
#define ADC_PREVIEW_QUEUE_DEPTH 8U
#define ADC_PREVIEW_BATCH_MAX_RECORDS 50U

typedef struct {
    uint8_t channel;
    uint16_t value;
} adc_preview_sample_t;

typedef struct {
    uint64_t frame_index;
    uint64_t timestamp_us;
    uint16_t sample_rate_hz;
    uint16_t frame_step;
    uint8_t channel_count;
    adc_preview_sample_t samples[ADC_PREVIEW_MAX_CHANNELS];
} adc_preview_record_t;

typedef struct {
    bool enabled;
    uint8_t channel_count;
    uint8_t channels[ADC_PREVIEW_MAX_CHANNELS];
    uint16_t target_hz;
    uint16_t actual_hz;
    uint64_t produced;
    uint64_t dropped;
} adc_preview_status_t;

esp_err_t adc_preview_init(void);
esp_err_t adc_preview_configure(const uint8_t *channels,
                                size_t channel_count,
                                uint16_t target_hz,
                                uint16_t *actual_hz);
void adc_preview_disable(void);
/* Called by the storage task with RAW260 frames, not PACKED132 bytes.
 * frame_count counts complete 64-channel frames; first_frame_index is 1-based.
 * A full preview queue drops preview records rather than blocking storage. */
void adc_preview_ingest_batch(const uint8_t *frames,
                              size_t frame_count,
                              uint64_t first_frame_index);
bool adc_preview_receive(adc_preview_record_t *record, TickType_t wait_ticks);
void adc_preview_get_status(adc_preview_status_t *status);

#ifdef __cplusplus
}
#endif
