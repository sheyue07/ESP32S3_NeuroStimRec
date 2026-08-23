#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "stim_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STIM_ACTIVE_SAMPLES 40U
#define STIM_IDLE_SAMPLES 61U
#define STIM_SLOT_SAMPLES 101U
#define STIM_START_SEQUENCE_SAMPLES 1010U

#define STIM_SAMPLE_MOSI (1U << 0)
#define STIM_SAMPLE_CSB (1U << 1)
#define STIM_SAMPLE_IDLE (STIM_SAMPLE_MOSI | STIM_SAMPLE_CSB)

void stim_waveform_build_slot(
    const uint8_t frame[STIM_PROTOCOL_FRAME_BYTES],
    uint8_t output[STIM_SLOT_SAMPLES]);
void stim_waveform_build_buffers(
    const uint8_t start_frames
        [STIM_PROTOCOL_START_FRAME_COUNT][STIM_PROTOCOL_FRAME_BYTES],
    uint8_t stop_loop[STIM_SLOT_SAMPLES],
    uint8_t start_sequence[STIM_START_SEQUENCE_SAMPLES],
    uint8_t enabled_idle[STIM_SLOT_SAMPLES]);
bool stim_waveform_validate_buffers(
    const uint8_t start_frames
        [STIM_PROTOCOL_START_FRAME_COUNT][STIM_PROTOCOL_FRAME_BYTES],
    const uint8_t stop_loop[STIM_SLOT_SAMPLES],
    const uint8_t start_sequence[STIM_START_SEQUENCE_SAMPLES],
    const uint8_t enabled_idle[STIM_SLOT_SAMPLES]);

#ifdef __cplusplus
}
#endif
