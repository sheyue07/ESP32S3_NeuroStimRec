#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define STIM_PROTOCOL_FRAME_BYTES 5U
#define STIM_PROTOCOL_START_FRAME_COUNT 10U

extern const uint8_t stim_start_frames
    [STIM_PROTOCOL_START_FRAME_COUNT][STIM_PROTOCOL_FRAME_BYTES];

uint8_t stim_protocol_checksum(const uint8_t frame[STIM_PROTOCOL_FRAME_BYTES]);
bool stim_protocol_validate_start_frames(void);

#ifdef __cplusplus
}
#endif

