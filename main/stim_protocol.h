#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define STIM_PROTOCOL_FRAME_BYTES 5U
#define STIM_PROTOCOL_START_FRAME_COUNT 10U

typedef struct {
    uint8_t Ch;
    uint16_t ChipID;
    uint8_t STclk_Sel;
    uint16_t mode;
    uint16_t Freq;
    uint16_t PulseNum;
    uint16_t PulseWA;
    uint16_t PulseGap;
    uint16_t PulseWC;
    uint16_t PulseAMP;
    uint16_t Stim;
} stim_protocol_parameters_t;

typedef struct {
    bool read;
    bool global;
    bool enable;
    uint8_t stclk_select;
    uint8_t reserve;
    uint8_t register_address;
    uint8_t channel_address;
    uint16_t data;
} stim_protocol_command_t;

extern const uint8_t stim_start_golden_frames
    [STIM_PROTOCOL_START_FRAME_COUNT][STIM_PROTOCOL_FRAME_BYTES];
extern const stim_protocol_parameters_t stim_protocol_default_parameters;

uint8_t stim_protocol_checksum(const uint8_t frame[STIM_PROTOCOL_FRAME_BYTES]);
bool stim_protocol_build_frame(
    const stim_protocol_command_t *command,
    uint8_t output[STIM_PROTOCOL_FRAME_BYTES]);
bool stim_protocol_build_start_frames(
    uint8_t output[STIM_PROTOCOL_START_FRAME_COUNT][STIM_PROTOCOL_FRAME_BYTES]);
bool stim_protocol_build_parameter_frames(
    const stim_protocol_parameters_t *parameters,
    uint8_t output[STIM_PROTOCOL_START_FRAME_COUNT][STIM_PROTOCOL_FRAME_BYTES]);
bool stim_protocol_validate_start_frames(void);

#ifdef __cplusplus
}
#endif
