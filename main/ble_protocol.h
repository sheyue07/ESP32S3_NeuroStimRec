#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_PROTOCOL_VERSION 1U
#define BLE_PROTOCOL_HEADER_BYTES 10U
#define BLE_PROTOCOL_DEFAULT_PACKET_BYTES 20U
#define BLE_PROTOCOL_MAX_PAYLOAD_BYTES 172U
#define BLE_PROTOCOL_MAX_FRAGMENTS 8U

typedef enum {
    BLE_MSG_HELLO_REQUEST = 0x01,
    BLE_MSG_STATUS_REQUEST = 0x02,
    BLE_MSG_PREVIEW_CONFIG = 0x03,
    BLE_MSG_CAPTURE_CONTROL = 0x04,
    BLE_MSG_STIMULATION_CONFIG = 0x05,
    BLE_MSG_STIMULATION_CONTROL = 0x06,
    BLE_MSG_KEEPALIVE = 0x07,
    BLE_MSG_HELLO_RESPONSE = 0x81,
    BLE_MSG_STATUS = 0x82,
    BLE_MSG_PREVIEW_CONFIG_ACK = 0x83,
    BLE_MSG_CAPTURE_ACK = 0x84,
    BLE_MSG_STIMULATION_CONFIG_ACK = 0x85,
    BLE_MSG_STIMULATION_ACK = 0x86,
    BLE_MSG_ERROR = 0x87,
    BLE_MSG_PREVIEW_DATA = 0x90,
    BLE_MSG_PREVIEW_BATCH = 0x91,
} ble_protocol_message_type_t;

typedef struct {
    uint8_t type;
    uint16_t sequence;
    const uint8_t *payload;
    size_t payload_length;
} ble_protocol_message_t;

typedef struct {
    bool active;
    uint8_t type;
    uint16_t sequence;
    uint16_t crc16;
    uint8_t fragment_count;
    uint8_t received_count;
    uint64_t last_update_ms;
    uint8_t fragment_lengths[BLE_PROTOCOL_MAX_FRAGMENTS];
    uint8_t fragments[BLE_PROTOCOL_MAX_FRAGMENTS][BLE_PROTOCOL_MAX_PAYLOAD_BYTES];
    uint8_t assembled[BLE_PROTOCOL_MAX_PAYLOAD_BYTES];
} ble_protocol_reassembler_t;

typedef bool (*ble_protocol_packet_callback_t)(const uint8_t *packet,
                                                size_t packet_length,
                                                void *user_context);

typedef enum {
    BLE_COMMAND_HELLO,
    BLE_COMMAND_STATUS,
    BLE_COMMAND_PREVIEW_CONFIG,
    BLE_COMMAND_CAPTURE_CONTROL,
    BLE_COMMAND_STIMULATION_CONFIG,
    BLE_COMMAND_STIMULATION_CONTROL,
    BLE_COMMAND_KEEPALIVE,
} ble_protocol_command_type_t;

typedef struct {
    uint8_t channel_count;
    uint8_t channels[8];
    uint16_t target_hz;
} ble_preview_config_command_t;

typedef struct {
    bool start;
    uint32_t request_id;
} ble_capture_control_command_t;

typedef struct {
    uint32_t revision;
    uint8_t Ch;
    uint8_t STclk_Sel;
    uint16_t ChipID;
    uint16_t mode;
    uint16_t Freq;
    uint16_t PulseNum;
    uint16_t PulseWA;
    uint16_t PulseGap;
    uint16_t PulseWC;
    uint16_t PulseAMP;
    uint16_t Stim;
} ble_stimulation_config_command_t;

typedef struct {
    bool start;
    uint32_t request_id;
    uint32_t revision;
} ble_stimulation_control_command_t;

typedef struct {
    ble_protocol_command_type_t type;
    uint16_t sequence;
    union {
        ble_preview_config_command_t preview;
        ble_capture_control_command_t capture;
        ble_stimulation_config_command_t stimulation_config;
        ble_stimulation_control_command_t stimulation_control;
    } data;
} ble_protocol_command_t;

uint16_t ble_protocol_crc16(const uint8_t *data, size_t length);
void ble_protocol_reassembler_reset(ble_protocol_reassembler_t *reassembler);
bool ble_protocol_reassembler_push(ble_protocol_reassembler_t *reassembler,
                                   const uint8_t *packet,
                                   size_t packet_length,
                                   uint64_t now_ms,
                                   ble_protocol_message_t *message);
bool ble_protocol_fragment_message(uint8_t type,
                                   uint16_t sequence,
                                   const uint8_t *payload,
                                   size_t payload_length,
                                   size_t packet_byte_limit,
                                   ble_protocol_packet_callback_t callback,
                                   void *user_context);
bool ble_protocol_parse_command(const ble_protocol_message_t *message,
                                ble_protocol_command_t *command);

void ble_protocol_write_u16(uint8_t *output, uint16_t value);
void ble_protocol_write_u32(uint8_t *output, uint32_t value);
void ble_protocol_write_u64(uint8_t *output, uint64_t value);
uint16_t ble_protocol_read_u16(const uint8_t *input);
uint32_t ble_protocol_read_u32(const uint8_t *input);
uint64_t ble_protocol_read_u64(const uint8_t *input);

#ifdef __cplusplus
}
#endif
