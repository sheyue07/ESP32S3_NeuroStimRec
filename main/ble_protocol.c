#include "ble_protocol.h"

#include <string.h>

#define BLE_PROTOCOL_MAGIC_0 0x4EU
#define BLE_PROTOCOL_MAGIC_1 0x53U
#define BLE_PROTOCOL_REASSEMBLY_TIMEOUT_MS UINT64_C(2000)

uint16_t ble_protocol_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = UINT16_C(0xFFFF);
    if (data == NULL && length != 0U) {
        return 0U;
    }
    for (size_t index = 0U; index < length; ++index) {
        crc ^= (uint16_t)data[index] << 8U;
        for (unsigned int bit = 0U; bit < 8U; ++bit) {
            crc = (crc & UINT16_C(0x8000)) != 0U
                      ? (uint16_t)((crc << 1U) ^ UINT16_C(0x1021))
                      : (uint16_t)(crc << 1U);
        }
    }
    return crc;
}

void ble_protocol_write_u16(uint8_t *output, uint16_t value)
{
    output[0] = (uint8_t)value;
    output[1] = (uint8_t)(value >> 8U);
}

void ble_protocol_write_u32(uint8_t *output, uint32_t value)
{
    for (size_t byte = 0U; byte < 4U; ++byte) {
        output[byte] = (uint8_t)(value >> (byte * 8U));
    }
}

void ble_protocol_write_u64(uint8_t *output, uint64_t value)
{
    for (size_t byte = 0U; byte < 8U; ++byte) {
        output[byte] = (uint8_t)(value >> (byte * 8U));
    }
}

uint16_t ble_protocol_read_u16(const uint8_t *input)
{
    return (uint16_t)input[0] | ((uint16_t)input[1] << 8U);
}

uint32_t ble_protocol_read_u32(const uint8_t *input)
{
    uint32_t value = 0U;
    for (size_t byte = 0U; byte < 4U; ++byte) {
        value |= (uint32_t)input[byte] << (byte * 8U);
    }
    return value;
}

uint64_t ble_protocol_read_u64(const uint8_t *input)
{
    uint64_t value = 0U;
    for (size_t byte = 0U; byte < 8U; ++byte) {
        value |= (uint64_t)input[byte] << (byte * 8U);
    }
    return value;
}

void ble_protocol_reassembler_reset(ble_protocol_reassembler_t *reassembler)
{
    if (reassembler != NULL) {
        memset(reassembler, 0, sizeof(*reassembler));
    }
}

static bool fragment_header_is_valid(const uint8_t *packet,
                                     size_t packet_length)
{
    return packet != NULL && packet_length >= BLE_PROTOCOL_HEADER_BYTES &&
           packet[0] == BLE_PROTOCOL_MAGIC_0 &&
           packet[1] == BLE_PROTOCOL_MAGIC_1 &&
           packet[2] == BLE_PROTOCOL_VERSION && packet[7] != 0U &&
           packet[7] <= BLE_PROTOCOL_MAX_FRAGMENTS && packet[6] < packet[7] &&
           packet_length - BLE_PROTOCOL_HEADER_BYTES <=
               BLE_PROTOCOL_MAX_PAYLOAD_BYTES;
}

bool ble_protocol_reassembler_push(ble_protocol_reassembler_t *reassembler,
                                   const uint8_t *packet,
                                   size_t packet_length,
                                   uint64_t now_ms,
                                   ble_protocol_message_t *message)
{
    if (reassembler == NULL || message == NULL ||
        !fragment_header_is_valid(packet, packet_length)) {
        return false;
    }
    message->payload = NULL;
    message->payload_length = 0U;

    const uint8_t type = packet[3];
    const uint16_t sequence = ble_protocol_read_u16(packet + 4U);
    const uint8_t fragment_index = packet[6];
    const uint8_t fragment_count = packet[7];
    const uint16_t crc16 = ble_protocol_read_u16(packet + 8U);
    const size_t fragment_length = packet_length - BLE_PROTOCOL_HEADER_BYTES;

    if (reassembler->active &&
        now_ms - reassembler->last_update_ms >
            BLE_PROTOCOL_REASSEMBLY_TIMEOUT_MS) {
        ble_protocol_reassembler_reset(reassembler);
    }
    if (!reassembler->active) {
        /*
         * A completed message leaves its assembled payload available to the
         * caller until the next packet arrives.  Clear per-message fragment
         * bookkeeping here before accepting that next packet; otherwise a
         * same-length command with different contents is mistaken for a
         * conflicting duplicate fragment and rejected with ATT error 0x13.
         */
        reassembler->received_count = 0U;
        memset(reassembler->fragment_lengths, 0,
               sizeof(reassembler->fragment_lengths));
        reassembler->active = true;
        reassembler->type = type;
        reassembler->sequence = sequence;
        reassembler->crc16 = crc16;
        reassembler->fragment_count = fragment_count;
    } else if (reassembler->type != type ||
               reassembler->sequence != sequence ||
               reassembler->crc16 != crc16 ||
               reassembler->fragment_count != fragment_count) {
        ble_protocol_reassembler_reset(reassembler);
        return false;
    }

    const uint8_t previous_length =
        reassembler->fragment_lengths[fragment_index];
    const uint8_t *const fragment_data =
        packet + BLE_PROTOCOL_HEADER_BYTES;
    if (previous_length != 0U) {
        if (previous_length != fragment_length ||
            memcmp(reassembler->fragments[fragment_index],
                   fragment_data, fragment_length) != 0) {
            ble_protocol_reassembler_reset(reassembler);
            return false;
        }
    } else {
        memcpy(reassembler->fragments[fragment_index],
               fragment_data, fragment_length);
        reassembler->fragment_lengths[fragment_index] =
            (uint8_t)fragment_length;
        ++reassembler->received_count;
    }
    reassembler->last_update_ms = now_ms;
    if (reassembler->received_count != reassembler->fragment_count) {
        return true;
    }

    size_t assembled_length = 0U;
    for (size_t index = 0U; index < reassembler->fragment_count; ++index) {
        const size_t part_length = reassembler->fragment_lengths[index];
        if (assembled_length + part_length > sizeof(reassembler->assembled)) {
            ble_protocol_reassembler_reset(reassembler);
            return false;
        }
        memcpy(reassembler->assembled + assembled_length,
               reassembler->fragments[index], part_length);
        assembled_length += part_length;
    }
    if (ble_protocol_crc16(reassembler->assembled, assembled_length) !=
        reassembler->crc16) {
        ble_protocol_reassembler_reset(reassembler);
        return false;
    }

    message->type = reassembler->type;
    message->sequence = reassembler->sequence;
    message->payload = reassembler->assembled;
    message->payload_length = assembled_length;
    reassembler->active = false;
    return true;
}

bool ble_protocol_fragment_message(uint8_t type,
                                   uint16_t sequence,
                                   const uint8_t *payload,
                                   size_t payload_length,
                                   size_t packet_byte_limit,
                                   ble_protocol_packet_callback_t callback,
                                   void *user_context)
{
    if ((payload == NULL && payload_length != 0U) || callback == NULL ||
        packet_byte_limit <= BLE_PROTOCOL_HEADER_BYTES ||
        packet_byte_limit > BLE_PROTOCOL_HEADER_BYTES +
                                BLE_PROTOCOL_MAX_PAYLOAD_BYTES) {
        return false;
    }
    const size_t fragment_payload =
        packet_byte_limit - BLE_PROTOCOL_HEADER_BYTES;
    const size_t fragment_count = payload_length == 0U
                                      ? 1U
                                      : (payload_length + fragment_payload - 1U) /
                                            fragment_payload;
    if (fragment_count > UINT8_MAX) {
        return false;
    }
    const uint16_t crc16 = ble_protocol_crc16(payload, payload_length);
    uint8_t packet[BLE_PROTOCOL_HEADER_BYTES + BLE_PROTOCOL_MAX_PAYLOAD_BYTES];
    for (size_t index = 0U; index < fragment_count; ++index) {
        const size_t offset = index * fragment_payload;
        const size_t remaining = payload_length - offset;
        const size_t part_length = remaining < fragment_payload
                                       ? remaining
                                       : fragment_payload;
        packet[0] = BLE_PROTOCOL_MAGIC_0;
        packet[1] = BLE_PROTOCOL_MAGIC_1;
        packet[2] = BLE_PROTOCOL_VERSION;
        packet[3] = type;
        ble_protocol_write_u16(packet + 4U, sequence);
        packet[6] = (uint8_t)index;
        packet[7] = (uint8_t)fragment_count;
        ble_protocol_write_u16(packet + 8U, crc16);
        if (part_length != 0U) {
            memcpy(packet + BLE_PROTOCOL_HEADER_BYTES,
                   payload + offset, part_length);
        }
        if (!callback(packet,
                      BLE_PROTOCOL_HEADER_BYTES + part_length,
                      user_context)) {
            return false;
        }
    }
    return true;
}

static bool parse_preview_config(const ble_protocol_message_t *message,
                                 ble_protocol_command_t *command)
{
    if (message->payload_length < 3U) {
        return false;
    }
    const uint8_t count = message->payload[0];
    if (count > 8U || message->payload_length != 3U + count) {
        return false;
    }
    uint64_t selected = 0U;
    for (size_t index = 0U; index < count; ++index) {
        const uint8_t channel = message->payload[1U + index];
        if (channel > 63U ||
            (selected & (UINT64_C(1) << channel)) != 0U) {
            return false;
        }
        selected |= UINT64_C(1) << channel;
        command->data.preview.channels[index] = channel;
    }
    const uint16_t target_hz =
        ble_protocol_read_u16(message->payload + 1U + count);
    if (target_hz == 0U || target_hz > 200U) {
        return false;
    }
    command->data.preview.channel_count = count;
    command->data.preview.target_hz = target_hz;
    return true;
}

bool ble_protocol_parse_command(const ble_protocol_message_t *message,
                                ble_protocol_command_t *command)
{
    if (message == NULL || command == NULL ||
        (message->payload == NULL && message->payload_length != 0U)) {
        return false;
    }
    memset(command, 0, sizeof(*command));
    command->sequence = message->sequence;
    switch (message->type) {
    case BLE_MSG_HELLO_REQUEST:
        command->type = BLE_COMMAND_HELLO;
        return message->payload_length == 0U;
    case BLE_MSG_STATUS_REQUEST:
        command->type = BLE_COMMAND_STATUS;
        return message->payload_length == 0U;
    case BLE_MSG_KEEPALIVE:
        command->type = BLE_COMMAND_KEEPALIVE;
        return message->payload_length == 0U;
    case BLE_MSG_PREVIEW_CONFIG:
        command->type = BLE_COMMAND_PREVIEW_CONFIG;
        return parse_preview_config(message, command);
    case BLE_MSG_CAPTURE_CONTROL:
        if (message->payload_length != 5U || message->payload[0] > 1U) {
            return false;
        }
        command->type = BLE_COMMAND_CAPTURE_CONTROL;
        command->data.capture.start = message->payload[0] != 0U;
        command->data.capture.request_id =
            ble_protocol_read_u32(message->payload + 1U);
        return true;
    case BLE_MSG_STIMULATION_CONFIG:
        if (message->payload_length != 24U || message->payload[4] > 63U ||
            message->payload[5] > 3U) {
            return false;
        }
        command->type = BLE_COMMAND_STIMULATION_CONFIG;
        command->data.stimulation_config.revision =
            ble_protocol_read_u32(message->payload);
        command->data.stimulation_config.Ch = message->payload[4];
        command->data.stimulation_config.STclk_Sel = message->payload[5];
        command->data.stimulation_config.ChipID =
            ble_protocol_read_u16(message->payload + 6U);
        command->data.stimulation_config.mode =
            ble_protocol_read_u16(message->payload + 8U);
        command->data.stimulation_config.Freq =
            ble_protocol_read_u16(message->payload + 10U);
        command->data.stimulation_config.PulseNum =
            ble_protocol_read_u16(message->payload + 12U);
        command->data.stimulation_config.PulseWA =
            ble_protocol_read_u16(message->payload + 14U);
        command->data.stimulation_config.PulseGap =
            ble_protocol_read_u16(message->payload + 16U);
        command->data.stimulation_config.PulseWC =
            ble_protocol_read_u16(message->payload + 18U);
        command->data.stimulation_config.PulseAMP =
            ble_protocol_read_u16(message->payload + 20U);
        command->data.stimulation_config.Stim =
            ble_protocol_read_u16(message->payload + 22U);
        return true;
    case BLE_MSG_STIMULATION_CONTROL:
        if (message->payload_length != 9U || message->payload[0] > 1U) {
            return false;
        }
        command->type = BLE_COMMAND_STIMULATION_CONTROL;
        command->data.stimulation_control.start = message->payload[0] != 0U;
        command->data.stimulation_control.request_id =
            ble_protocol_read_u32(message->payload + 1U);
        command->data.stimulation_control.revision =
            ble_protocol_read_u32(message->payload + 5U);
        return true;
    default:
        return false;
    }
}
