#include "stim_protocol.h"

const uint8_t stim_start_frames
    [STIM_PROTOCOL_START_FRAME_COUNT][STIM_PROTOCOL_FRAME_BYTES] = {
        {0x40, 0x23, 0x2A, 0x0D, 0x9A},
        {0x80, 0x23, 0x2A, 0x0D, 0xDA},
        {0x00, 0x63, 0x00, 0x06, 0x69},
        {0x00, 0xA3, 0xC3, 0x23, 0x89},
        {0x00, 0xE3, 0x00, 0x64, 0x47},
        {0x01, 0x23, 0x00, 0x14, 0x38},
        {0x01, 0x63, 0x00, 0x05, 0x69},
        {0x01, 0xA3, 0x00, 0x14, 0xB8},
        {0x01, 0xE3, 0xFF, 0xFF, 0xE2},
        {0x20, 0x23, 0xFF, 0xFF, 0x41},
};

uint8_t stim_protocol_checksum(const uint8_t frame[STIM_PROTOCOL_FRAME_BYTES])
{
    uint32_t sum = 0U;
    for (size_t index = 0; index < STIM_PROTOCOL_FRAME_BYTES - 1U; ++index) {
        sum += frame[index];
    }
    return (uint8_t)sum;
}

bool stim_protocol_validate_start_frames(void)
{
    for (size_t frame = 0; frame < STIM_PROTOCOL_START_FRAME_COUNT; ++frame) {
        if (stim_protocol_checksum(stim_start_frames[frame]) !=
            stim_start_frames[frame][STIM_PROTOCOL_FRAME_BYTES - 1U]) {
            return false;
        }
    }
    return true;
}

