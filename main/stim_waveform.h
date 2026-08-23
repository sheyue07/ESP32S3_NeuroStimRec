#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STIM_MCLK_GPIO GPIO_NUM_19
#define STIM_SCLK_GPIO GPIO_NUM_8
#define STIM_MOSI_GPIO GPIO_NUM_17
#define STIM_CSB_GPIO GPIO_NUM_15
#define STIM_CLOCK_HZ 6600000U

typedef enum {
    STIM_WAVEFORM_STOP_LOOP = 0,
    STIM_WAVEFORM_START_SEQUENCE,
    STIM_WAVEFORM_ENABLED_IDLE,
    STIM_WAVEFORM_FAULT,
} stim_waveform_state_t;

typedef bool (*stim_waveform_event_callback_t)(stim_waveform_state_t event,
                                               void *user_data);

typedef struct {
    stim_waveform_state_t state;
    bool requested_enabled;
    bool fatal;
    uint32_t completed_start_frames;
    uint32_t descriptor_errors;
    uint32_t fifo_underflow_errors;
    uint32_t unexpected_stop_errors;
} stim_waveform_status_t;

esp_err_t stim_waveform_init(stim_waveform_event_callback_t event_callback,
                             void *user_data);
esp_err_t stim_waveform_request_enabled(bool enabled);
void stim_waveform_enter_safe_state(void);
void stim_waveform_deinit(void);
void stim_waveform_get_status(stim_waveform_status_t *status);

#ifdef __cplusplus
}
#endif
