#include "stim_waveform_builder.h"

#include <stddef.h>
#include <string.h>

_Static_assert(STIM_ACTIVE_SAMPLES + STIM_IDLE_SAMPLES == STIM_SLOT_SAMPLES,
               "stimulus slot length mismatch");
_Static_assert(STIM_PROTOCOL_START_FRAME_COUNT * STIM_SLOT_SAMPLES ==
                   STIM_START_SEQUENCE_SAMPLES,
               "stimulus start sequence length mismatch");

void stim_waveform_build_slot(
    const uint8_t frame[STIM_PROTOCOL_FRAME_BYTES],
    uint8_t output[STIM_SLOT_SAMPLES])
{
    size_t sample = 0U;
    for (size_t byte = 0U; byte < STIM_PROTOCOL_FRAME_BYTES; ++byte) {
        for (int bit = 7; bit >= 0; --bit) {
            output[sample++] =
                (frame[byte] & (uint8_t)(1U << bit)) != 0U
                    ? STIM_SAMPLE_MOSI
                    : 0U;
        }
    }
    memset(output + STIM_ACTIVE_SAMPLES, STIM_SAMPLE_IDLE, STIM_IDLE_SAMPLES);
}

void stim_waveform_build_buffers(
    const uint8_t start_frames
        [STIM_PROTOCOL_START_FRAME_COUNT][STIM_PROTOCOL_FRAME_BYTES],
    uint8_t stop_loop[STIM_SLOT_SAMPLES],
    uint8_t start_sequence[STIM_START_SEQUENCE_SAMPLES],
    uint8_t enabled_idle[STIM_SLOT_SAMPLES])
{
    stim_waveform_build_slot(start_frames[0], stop_loop);
    memset(enabled_idle, STIM_SAMPLE_IDLE, STIM_SLOT_SAMPLES);
    for (size_t frame = 0U; frame < STIM_PROTOCOL_START_FRAME_COUNT; ++frame) {
        stim_waveform_build_slot(
            start_frames[frame],
            start_sequence + frame * STIM_SLOT_SAMPLES);
    }
}

bool stim_waveform_validate_buffers(
    const uint8_t start_frames
        [STIM_PROTOCOL_START_FRAME_COUNT][STIM_PROTOCOL_FRAME_BYTES],
    const uint8_t stop_loop[STIM_SLOT_SAMPLES],
    const uint8_t start_sequence[STIM_START_SEQUENCE_SAMPLES],
    const uint8_t enabled_idle[STIM_SLOT_SAMPLES])
{
    uint8_t expected[STIM_SLOT_SAMPLES];
    stim_waveform_build_slot(start_frames[0], expected);
    if (memcmp(stop_loop, expected, sizeof(expected)) != 0) {
        return false;
    }

    for (size_t sample = 0U; sample < STIM_SLOT_SAMPLES; ++sample) {
        if (enabled_idle[sample] != STIM_SAMPLE_IDLE) {
            return false;
        }
    }

    for (size_t frame = 0U; frame < STIM_PROTOCOL_START_FRAME_COUNT; ++frame) {
        stim_waveform_build_slot(start_frames[frame], expected);
        if (memcmp(start_sequence + frame * STIM_SLOT_SAMPLES,
                   expected,
                   sizeof(expected)) != 0) {
            return false;
        }
    }
    return true;
}
