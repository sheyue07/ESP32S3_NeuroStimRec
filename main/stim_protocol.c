#include "stim_protocol.h"

#include <string.h>

#define STIM_PROTOCOL_DEFAULT_CHANNEL 0x23U

static const stim_protocol_command_t
    s_start_commands[STIM_PROTOCOL_START_FRAME_COUNT] = {
        {.global = true, .data = 0x2A0D},
        {.read = true, .data = 0x2A0D},
        {.register_address = 1U, .data = 0x0006},
        {.register_address = 2U, .data = 0xC323},
        {.register_address = 3U, .data = 0x0064},
        {.register_address = 4U, .data = 0x0014},
        {.register_address = 5U, .data = 0x0005},
        {.register_address = 6U, .data = 0x0014},
        {.register_address = 7U, .data = 0xFFFF},
        {.enable = true, .register_address = 0U, .data = 0xFFFF},
};

const uint8_t stim_start_golden_frames
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

bool stim_protocol_build_frame(
    const stim_protocol_command_t *command,
    uint8_t output[STIM_PROTOCOL_FRAME_BYTES])
{
    if (command == NULL || output == NULL || command->stclk_select > 3U ||
        command->reserve > 3U || command->register_address > 7U ||
        command->channel_address > 0x3FU) {
        return false;
    }

    output[0] = (command->read ? 0x80U : 0U) |
                (command->global ? 0x40U : 0U) |
                (command->enable ? 0x20U : 0U) |
                (uint8_t)(command->stclk_select << 3U) |
                (uint8_t)(command->reserve << 1U) |
                (uint8_t)(command->register_address >> 2U);
    output[1] = (uint8_t)((command->register_address & 0x03U) << 6U) |
                (command->channel_address & 0x3FU);
    output[2] = (uint8_t)(command->data >> 8U);
    output[3] = (uint8_t)command->data;
    output[4] = stim_protocol_checksum(output);
    return true;
}

bool stim_protocol_build_start_frames(
    uint8_t output[STIM_PROTOCOL_START_FRAME_COUNT][STIM_PROTOCOL_FRAME_BYTES])
{
    if (output == NULL) {
        return false;
    }
    for (size_t index = 0U;
         index < STIM_PROTOCOL_START_FRAME_COUNT;
         ++index) {
        stim_protocol_command_t command = s_start_commands[index];
        command.channel_address = STIM_PROTOCOL_DEFAULT_CHANNEL;
        if (!stim_protocol_build_frame(&command, output[index])) {
            return false;
        }
    }
    return true;
}

bool stim_protocol_validate_start_frames(void)
{
    uint8_t generated
        [STIM_PROTOCOL_START_FRAME_COUNT][STIM_PROTOCOL_FRAME_BYTES];
    return stim_protocol_build_start_frames(generated) &&
           memcmp(generated, stim_start_golden_frames, sizeof(generated)) == 0;
}
