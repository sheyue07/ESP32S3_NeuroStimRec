#include "ble_service.h"
#include "uart_bridge.h"
#include <inttypes.h>

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "adc_preview.h"
#include "ble_protocol.h"
#include "emmc_storage_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "os/os_mbuf.h"
#include "raw_sd_segment_format.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "stim_controller.h"
#include "stim_protocol.h"
#include "stim_waveform.h"

#define BLE_DEVICE_NAME "NeuroStimRec"
#define BLE_COMMAND_QUEUE_DEPTH 8U
#define BLE_CONTROL_TASK_STACK 5120U
#define BLE_CONTROL_TASK_PRIORITY 5U
#define BLE_STATUS_INTERVAL_MS 500U
#define BLE_PREVIEW_BATCH_FORMAT 1U
#define BLE_PREVIEW_BATCH_HEADER_BYTES 24U
#define BLE_PREVIEW_BATCH_MAX_LATENCY_MS 500U
#define BLE_NOTIFY_FLAG (1U << 0)
#define BLE_DISCONNECT_FLAG (1U << 1)

#define BLE_RESULT_OK 0U
#define BLE_RESULT_INVALID_COMMAND 1U
#define BLE_RESULT_BUSY 2U
#define BLE_RESULT_STORAGE 3U
#define BLE_RESULT_INTERNAL 5U

/* BLE_UUID128_INIT takes UUID bytes in little-endian order. */
static const ble_uuid128_t s_service_uuid = BLE_UUID128_INIT(
    0x01, 0x50, 0x47, 0x70, 0xe9, 0xaf, 0x18, 0x9b,
    0xb2, 0x4d, 0x80, 0x73, 0x44, 0x8b, 0xe2, 0x11);
static const ble_uuid128_t s_write_uuid = BLE_UUID128_INIT(
    0x02, 0x50, 0x47, 0x70, 0xe9, 0xaf, 0x18, 0x9b,
    0xb2, 0x4d, 0x80, 0x73, 0x44, 0x8b, 0xe2, 0x11);
static const ble_uuid128_t s_notify_uuid = BLE_UUID128_INIT(
    0x03, 0x50, 0x47, 0x70, 0xe9, 0xaf, 0x18, 0x9b,
    0xb2, 0x4d, 0x80, 0x73, 0x44, 0x8b, 0xe2, 0x11);

typedef struct {
    portMUX_TYPE lock;
    QueueHandle_t command_queue;
    StaticQueue_t command_queue_storage;
    uint8_t command_queue_bytes[
        BLE_COMMAND_QUEUE_DEPTH * sizeof(ble_protocol_command_t)];
    TaskHandle_t control_task;
    ble_protocol_reassembler_t reassembler;
    uint16_t connection_handle;
    uint16_t notify_value_handle;
    uint16_t tx_sequence;
    uint32_t config_revision;
    uint8_t own_address_type;
    bool connected;
    bool notify_enabled;
    bool stimulation_configured;
} ble_service_context_t;

static const char *TAG = "BLE_SERVICE";
static ble_service_context_t s_ble = {
    .lock = portMUX_INITIALIZER_UNLOCKED,
    .connection_handle = BLE_HS_CONN_HANDLE_NONE,
};

void ble_store_config_init(void);

static void start_advertising(void);

static uint16_t result_from_esp_err(esp_err_t error)
{
    if (error == ESP_OK) {
        return BLE_RESULT_OK;
    }
    if (error == EMMC_STORAGE_ERR_BUSY || error == EMMC_STORAGE_ERR_INTERLOCK) {
        return BLE_RESULT_BUSY;
    }
    if (error == EMMC_STORAGE_ERR_CARD) {
        return BLE_RESULT_STORAGE;
    }
    if (error == ESP_ERR_INVALID_ARG) {
        return BLE_RESULT_INVALID_COMMAND;
    }
    if (error == ESP_ERR_INVALID_STATE) {
        return BLE_RESULT_BUSY;
    }
    return BLE_RESULT_INTERNAL;
}

static bool connection_snapshot(uint16_t *connection_handle,
                                uint16_t *notify_handle)
{
    bool ready;
    portENTER_CRITICAL(&s_ble.lock);
    ready = s_ble.connected && s_ble.notify_enabled &&
            s_ble.connection_handle != BLE_HS_CONN_HANDLE_NONE &&
            s_ble.notify_value_handle != 0U;
    if (connection_handle != NULL) {
        *connection_handle = s_ble.connection_handle;
    }
    if (notify_handle != NULL) {
        *notify_handle = s_ble.notify_value_handle;
    }
    portEXIT_CRITICAL(&s_ble.lock);
    return ready;
}

typedef struct {
    uint16_t connection_handle;
    uint16_t notify_handle;
    bool sent;
} notify_packet_context_t;

typedef struct {
    bool active;
    TickType_t started_at;
    uint64_t first_frame_index;
    uint64_t first_timestamp_us;
    uint16_t sample_rate_hz;
    uint16_t frame_step;
    uint8_t channel_count;
    uint8_t sample_count;
    uint8_t channels[ADC_PREVIEW_MAX_CHANNELS];
    uint16_t values[ADC_PREVIEW_BATCH_MAX_RECORDS]
                   [ADC_PREVIEW_MAX_CHANNELS];
} preview_batch_context_t;

static bool notify_packet(const uint8_t *packet,
                          size_t packet_length,
                          void *user_context)
{
    notify_packet_context_t *const context = user_context;
    struct os_mbuf *const om = os_msys_get_pkthdr(packet_length, 0);
    if (om == NULL || os_mbuf_append(om, packet, packet_length) != 0) {
        if (om != NULL) {
            os_mbuf_free_chain(om);
        }
        return false;
    }
    const int result = ble_gatts_notify_custom(
        context->connection_handle, context->notify_handle, om);
    if (result != 0) {
        return false;
    }
    context->sent = true;
    return true;
}

static bool send_message(uint8_t type,
                         const uint8_t *payload,
                         size_t payload_length)
{
    uint16_t connection_handle;
    uint16_t notify_handle;
    if (!connection_snapshot(&connection_handle, &notify_handle)) {
        return false;
    }
    size_t packet_limit = BLE_PROTOCOL_DEFAULT_PACKET_BYTES;
    const uint16_t mtu = ble_att_mtu(connection_handle);
    if (mtu > 3U) {
        packet_limit = mtu - 3U;
        const size_t maximum = BLE_PROTOCOL_HEADER_BYTES +
                               BLE_PROTOCOL_MAX_PAYLOAD_BYTES;
        if (packet_limit > maximum) {
            packet_limit = maximum;
        }
    }
    uint16_t sequence;
    portENTER_CRITICAL(&s_ble.lock);
    sequence = s_ble.tx_sequence++;
    portEXIT_CRITICAL(&s_ble.lock);
    notify_packet_context_t context = {
        .connection_handle = connection_handle,
        .notify_handle = notify_handle,
    };
    return ble_protocol_fragment_message(type, sequence,
                                         payload, payload_length,
                                         packet_limit,
                                         notify_packet, &context) &&
           context.sent;
}

static void send_status(void)
{
    emmc_status_t storage;
    if (emmc_storage_get_status(&storage) != ESP_OK) {
        return;
    }
    stim_waveform_status_t stimulation;
    stim_waveform_get_status(&stimulation);
    adc_preview_status_t preview;
    adc_preview_get_status(&preview);

    uint32_t flags = 0U;
    if (storage.card_ready) flags |= 1U << 0;
    if (storage.state == EMMC_STATE_WRITING) flags |= 1U << 1;
    if (stimulation.state == STIM_WAVEFORM_START_SEQUENCE ||
        stimulation.state == STIM_WAVEFORM_ENABLED_IDLE) flags |= 1U << 2;
    if (storage.sync_state == 2U) flags |= 1U << 3;
    if (preview.enabled) flags |= 1U << 4;

    uint8_t payload[56] = {0};
    ble_protocol_write_u32(payload, (uint32_t)(esp_timer_get_time() / 1000));
    ble_protocol_write_u32(payload + 4U, flags);
    payload[8] = (uint8_t)storage.state;
    payload[9] = (uint8_t)stimulation.state;
    payload[10] = (uint8_t)storage.sync_state;
    payload[11] = storage.capture_source_mask;
    payload[12] = stimulation.requested_enabled ? 2U : 0U;
    ble_protocol_write_u16(payload + 14U,
                           (uint16_t)((uint32_t)storage.failure_code & 0xFFFFU));
    ble_protocol_write_u64(payload + 16U, storage.frame_count);
    ble_protocol_write_u64(payload + 24U, storage.valid_bytes);
    ble_protocol_write_u32(payload + 32U,
                           preview.dropped > UINT32_MAX
                               ? UINT32_MAX
                               : (uint32_t)preview.dropped);
    ble_protocol_write_u32(payload + 36U, s_ble.config_revision);
    ble_protocol_write_u64(payload + 40U, storage.target_bytes);
    ble_protocol_write_u64(payload + 48U,
                           storage.capacity_sectors * RAW_SD_SECTOR_BYTES);
    (void)send_message(BLE_MSG_STATUS, payload, sizeof(payload));
}

static void preview_batch_reset(preview_batch_context_t *batch)
{
    if (batch != NULL) {
        memset(batch, 0, sizeof(*batch));
    }
}

static bool preview_batch_matches(const preview_batch_context_t *batch,
                                  const adc_preview_record_t *record)
{
    if (!batch->active || batch->channel_count != record->channel_count ||
        batch->sample_rate_hz != record->sample_rate_hz ||
        batch->frame_step != record->frame_step) {
        return false;
    }
    for (size_t index = 0U; index < record->channel_count; ++index) {
        if (batch->channels[index] != record->samples[index].channel) {
            return false;
        }
    }
    return true;
}

static size_t preview_message_payload_limit(void)
{
    uint16_t connection_handle;
    if (!connection_snapshot(&connection_handle, NULL)) {
        return 0U;
    }
    size_t packet_limit = BLE_PROTOCOL_DEFAULT_PACKET_BYTES;
    const uint16_t mtu = ble_att_mtu(connection_handle);
    if (mtu > 3U) {
        packet_limit = mtu - 3U;
    }
    const size_t maximum = BLE_PROTOCOL_HEADER_BYTES +
                           BLE_PROTOCOL_MAX_PAYLOAD_BYTES;
    if (packet_limit > maximum) {
        packet_limit = maximum;
    }
    return packet_limit > BLE_PROTOCOL_HEADER_BYTES
               ? packet_limit - BLE_PROTOCOL_HEADER_BYTES
               : 0U;
}

static size_t preview_batch_capacity(uint8_t channel_count,
                                     uint16_t sample_rate_hz)
{
    if (channel_count == 0U) {
        return 0U;
    }
    const size_t fixed_bytes = BLE_PREVIEW_BATCH_HEADER_BYTES + channel_count;
    const size_t per_sample_bytes = (size_t)channel_count * 2U;
    const size_t payload_limit = preview_message_payload_limit();
    size_t capacity = payload_limit > fixed_bytes
                          ? (payload_limit - fixed_bytes) / per_sample_bytes
                          : 1U;
    if (capacity == 0U) {
        capacity = 1U;
    }
    if (capacity > ADC_PREVIEW_BATCH_MAX_RECORDS) {
        capacity = ADC_PREVIEW_BATCH_MAX_RECORDS;
    }
    /* Bound display latency to roughly 500 ms even when one channel would
     * allow a much larger ATT notification. */
    size_t latency_capacity = (sample_rate_hz + 1U) / 2U;
    if (latency_capacity == 0U) {
        latency_capacity = 1U;
    }
    if (capacity > latency_capacity) {
        capacity = latency_capacity;
    }
    return capacity;
}

static void preview_batch_start(preview_batch_context_t *batch,
                                const adc_preview_record_t *record,
                                TickType_t now)
{
    preview_batch_reset(batch);
    batch->active = true;
    batch->started_at = now;
    batch->first_frame_index = record->frame_index;
    batch->first_timestamp_us = record->timestamp_us;
    batch->sample_rate_hz = record->sample_rate_hz;
    batch->frame_step = record->frame_step;
    batch->channel_count = record->channel_count;
    for (size_t index = 0U; index < record->channel_count; ++index) {
        batch->channels[index] = record->samples[index].channel;
    }
}

static void preview_batch_append(preview_batch_context_t *batch,
                                 const adc_preview_record_t *record)
{
    if (batch->sample_count >= ADC_PREVIEW_BATCH_MAX_RECORDS) {
        return;
    }
    const size_t sample_index = batch->sample_count;
    for (size_t channel = 0U; channel < record->channel_count; ++channel) {
        batch->values[sample_index][channel] =
            record->samples[channel].value;
    }
    ++batch->sample_count;
}

static void send_preview_batch(preview_batch_context_t *batch)
{
    if (!batch->active || batch->sample_count == 0U) {
        preview_batch_reset(batch);
        return;
    }
    uint8_t payload[BLE_PROTOCOL_MAX_PAYLOAD_BYTES] = {0};
    payload[0] = BLE_PREVIEW_BATCH_FORMAT;
    payload[1] = batch->channel_count;
    payload[2] = batch->sample_count;
    ble_protocol_write_u16(payload + 4U, batch->sample_rate_hz);
    ble_protocol_write_u16(payload + 6U, batch->frame_step);
    ble_protocol_write_u64(payload + 8U, batch->first_frame_index);
    ble_protocol_write_u64(payload + 16U, batch->first_timestamp_us);
    memcpy(payload + BLE_PREVIEW_BATCH_HEADER_BYTES,
           batch->channels, batch->channel_count);

    size_t offset = BLE_PREVIEW_BATCH_HEADER_BYTES + batch->channel_count;
    for (size_t sample = 0U; sample < batch->sample_count; ++sample) {
        for (size_t channel = 0U; channel < batch->channel_count; ++channel) {
            if (offset + 2U > sizeof(payload)) {
                preview_batch_reset(batch);
                return;
            }
            ble_protocol_write_u16(payload + offset,
                                   batch->values[sample][channel]);
            offset += 2U;
        }
    }
    (void)send_message(BLE_MSG_PREVIEW_BATCH, payload, offset);
    preview_batch_reset(batch);
}

static void send_preview_ack(const ble_preview_config_command_t *preview,
                             esp_err_t result,
                             uint16_t actual_hz)
{
    uint8_t payload[5U + ADC_PREVIEW_MAX_CHANNELS] = {0};
    ble_protocol_write_u16(payload, result_from_esp_err(result));
    ble_protocol_write_u16(payload + 2U, actual_hz);
    payload[4] = result == ESP_OK ? preview->channel_count : 0U;
    if (result == ESP_OK) {
        memcpy(payload + 5U, preview->channels, preview->channel_count);
    }
    (void)send_message(BLE_MSG_PREVIEW_CONFIG_ACK,
                       payload, 5U + payload[4]);
}

static void send_capture_ack(const ble_capture_control_command_t *capture,
                             esp_err_t result)
{
    emmc_status_t status = {0};
    (void)emmc_storage_get_status(&status);
    uint8_t payload[8];
    ble_protocol_write_u32(payload, capture->request_id);
    ble_protocol_write_u16(payload + 4U, result_from_esp_err(result));
    payload[6] = status.state == EMMC_STATE_WRITING ? 1U : 0U;
    payload[7] = status.capture_source_mask;
    (void)send_message(BLE_MSG_CAPTURE_ACK, payload, sizeof(payload));
}

static void send_stimulation_config_ack(uint32_t revision,
                                        esp_err_t result)
{
    uint8_t payload[6];
    ble_protocol_write_u32(payload, revision);
    ble_protocol_write_u16(payload + 4U, result_from_esp_err(result));
    (void)send_message(BLE_MSG_STIMULATION_CONFIG_ACK,
                       payload, sizeof(payload));
}

static void send_stimulation_ack(
    const ble_stimulation_control_command_t *control,
    esp_err_t result)
{
    stim_waveform_status_t status;
    stim_waveform_get_status(&status);
    uint8_t payload[11];
    ble_protocol_write_u32(payload, control->request_id);
    ble_protocol_write_u16(payload + 4U, result_from_esp_err(result));
    payload[6] = (uint8_t)status.state;
    ble_protocol_write_u32(payload + 7U, s_ble.config_revision);
    (void)send_message(BLE_MSG_STIMULATION_ACK,
                       payload, sizeof(payload));
}

static void send_error(uint32_t request_id, uint16_t result, uint16_t detail)
{
    uint8_t payload[8];
    ble_protocol_write_u32(payload, request_id);
    ble_protocol_write_u16(payload + 4U, result);
    ble_protocol_write_u16(payload + 6U, detail);
    (void)send_message(BLE_MSG_ERROR, payload, sizeof(payload));
}

static void handle_command(const ble_protocol_command_t *command)
{
    switch (command->type) {
    case BLE_COMMAND_STATUS:
    case BLE_COMMAND_HELLO:
        send_status();
        break;
    case BLE_COMMAND_PREVIEW_CONFIG: {
        uint16_t actual_hz = 0U;
        const esp_err_t result = adc_preview_configure(
            command->data.preview.channels,
            command->data.preview.channel_count,
            command->data.preview.target_hz,
            &actual_hz);
        send_preview_ack(&command->data.preview, result, actual_hz);
        break;
    }
    case BLE_COMMAND_CAPTURE_CONTROL: {
        const esp_err_t result = emmc_storage_set_capture_request(
            EMMC_CAPTURE_SOURCE_BLE, command->data.capture.start);
        send_capture_ack(&command->data.capture, result);
        send_status();
        break;
    }
    case BLE_COMMAND_STIMULATION_CONFIG: {
        const ble_stimulation_config_command_t *const config =
            &command->data.stimulation_config;
        const stim_protocol_parameters_t parameters = {
            .Ch = config->Ch,
            .ChipID = config->ChipID,
            .STclk_Sel = config->STclk_Sel,
            .mode = config->mode,
            .Freq = config->Freq,
            .PulseNum = config->PulseNum,
            .PulseWA = config->PulseWA,
            .PulseGap = config->PulseGap,
            .PulseWC = config->PulseWC,
            .PulseAMP = config->PulseAMP,
            .Stim = config->Stim,
        };
        const esp_err_t result = stim_controller_configure(&parameters);
        if (result == ESP_OK) {
            s_ble.config_revision = config->revision;
            s_ble.stimulation_configured = true;
        }
        send_stimulation_config_ack(config->revision, result);
        uart_bridge_stim_log("CONFIG seq=%u rev=%" PRIu32 " result=%s "
            "Ch=%u ChipID=0x%04X STclk_Sel=%u mode=0x%04X Freq=%u "
            "PulseNum=%u PulseWA=%u PulseGap=%u PulseWC=%u PulseAMP=%u Stim=0x%04X",
            command->sequence, config->revision, esp_err_to_name(result),
            config->Ch, config->ChipID, config->STclk_Sel, config->mode,
            config->Freq, config->PulseNum, config->PulseWA, config->PulseGap,
            config->PulseWC, config->PulseAMP, config->Stim);
        send_status();
        break;
    }
    case BLE_COMMAND_STIMULATION_CONTROL: {
        const ble_stimulation_control_command_t *const control =
            &command->data.stimulation_control;
        esp_err_t result;
        if (control->start &&
            (!s_ble.stimulation_configured ||
             control->revision != s_ble.config_revision)) {
            result = ESP_ERR_INVALID_STATE;
        } else {
            result = stim_controller_request_enabled(control->start);
        }
        send_stimulation_ack(control, result);
        uart_bridge_stim_log("%s seq=%u request=%" PRIu32 " rev=%" PRIu32
            " active_rev=%" PRIu32 " result=%s",
            control->start ? "START" : "STOP", command->sequence,
            control->request_id, control->revision, s_ble.config_revision,
            esp_err_to_name(result));
        send_status();
        break;
    }
    case BLE_COMMAND_KEEPALIVE:
        break;
    default:
        send_error(0U, BLE_RESULT_INVALID_COMMAND, 0U);
        break;
    }
}

static void ble_control_task(void *parameter)
{
    (void)parameter;
    uint32_t notifications = 0U;
    TickType_t last_status = xTaskGetTickCount();
    preview_batch_context_t preview_batch = {0};
    for (;;) {
        (void)xTaskNotifyWait(0U, UINT32_MAX, &notifications, 0);
        if ((notifications & BLE_DISCONNECT_FLAG) != 0U) {
            preview_batch_reset(&preview_batch);
            adc_preview_disable();
            (void)emmc_storage_set_capture_request(
                EMMC_CAPTURE_SOURCE_BLE, false);
            (void)stim_controller_request_enabled(false);
        }
        ble_protocol_command_t command;
        while (xQueueReceive(s_ble.command_queue, &command, 0) == pdTRUE) {
            handle_command(&command);
        }

        adc_preview_record_t preview;
        if (connection_snapshot(NULL, NULL) &&
            adc_preview_receive(&preview, pdMS_TO_TICKS(10))) {
            const TickType_t now = xTaskGetTickCount();
            if (preview_batch.active &&
                !preview_batch_matches(&preview_batch, &preview)) {
                /* Configuration changed. Do not deliver stale channels after
                 * the new configuration ACK. */
                preview_batch_reset(&preview_batch);
            }
            if (!preview_batch.active) {
                preview_batch_start(&preview_batch, &preview, now);
            }
            preview_batch_append(&preview_batch, &preview);
            if (preview_batch.sample_count >=
                preview_batch_capacity(preview_batch.channel_count,
                                       preview_batch.sample_rate_hz)) {
                send_preview_batch(&preview_batch);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        const TickType_t now = xTaskGetTickCount();
        adc_preview_status_t preview_status;
        adc_preview_get_status(&preview_status);
        if (preview_batch.active && !preview_status.enabled) {
            preview_batch_reset(&preview_batch);
        } else if (preview_batch.active &&
                   now - preview_batch.started_at >=
                       pdMS_TO_TICKS(BLE_PREVIEW_BATCH_MAX_LATENCY_MS)) {
            send_preview_batch(&preview_batch);
        }
        if (connection_snapshot(NULL, NULL) &&
            now - last_status >= pdMS_TO_TICKS(BLE_STATUS_INTERVAL_MS)) {
            send_status();
            last_status = now;
        }
    }
}

static int write_characteristic_access(uint16_t connection_handle,
                                       uint16_t attribute_handle,
                                       struct ble_gatt_access_ctxt *context,
                                       void *argument)
{
    (void)connection_handle;
    (void)attribute_handle;
    (void)argument;
    if (context->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }
    const uint16_t length = OS_MBUF_PKTLEN(context->om);
    if (length < BLE_PROTOCOL_HEADER_BYTES ||
        length > BLE_PROTOCOL_HEADER_BYTES + BLE_PROTOCOL_MAX_PAYLOAD_BYTES) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    uint8_t packet[BLE_PROTOCOL_HEADER_BYTES + BLE_PROTOCOL_MAX_PAYLOAD_BYTES];
    uint16_t copied = 0U;
    if (ble_hs_mbuf_to_flat(context->om, packet, sizeof(packet), &copied) != 0 ||
        copied != length) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    ble_protocol_message_t message;
    const uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000);
    if (!ble_protocol_reassembler_push(&s_ble.reassembler,
                                       packet, copied, now_ms, &message)) {
        return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
    }
    if (message.payload == NULL) {
        return 0;
    }
    ble_protocol_command_t command;
    if (!ble_protocol_parse_command(&message, &command)) {
        return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
    }
    if (xQueueSend(s_ble.command_queue, &command, 0) != pdTRUE) {
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    xTaskNotify(s_ble.control_task, BLE_NOTIFY_FLAG, eSetBits);
    return 0;
}

/*
 * ESP-IDF 6.x validates that every characteristic has an access callback,
 * including notify-only characteristics.  The phone never reads or writes
 * this value directly; notifications are sent with ble_gatts_notify_custom().
 */
static int notify_characteristic_access(uint16_t connection_handle,
                                        uint16_t attribute_handle,
                                        struct ble_gatt_access_ctxt *context,
                                        void *argument)
{
    (void)connection_handle;
    (void)attribute_handle;
    (void)context;
    (void)argument;
    return BLE_ATT_ERR_READ_NOT_PERMITTED;
}

static const struct ble_gatt_svc_def s_gatt_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &s_write_uuid.u,
                .access_cb = write_characteristic_access,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid = &s_notify_uuid.u,
                .access_cb = notify_characteristic_access,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_ble.notify_value_handle,
            },
            {0},
        },
    },
    {0},
};

static int gap_event(struct ble_gap_event *event, void *argument)
{
    (void)argument;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            portENTER_CRITICAL(&s_ble.lock);
            s_ble.connected = true;
            s_ble.connection_handle = event->connect.conn_handle;
            s_ble.notify_enabled = false;
            portEXIT_CRITICAL(&s_ble.lock);
            ble_protocol_reassembler_reset(&s_ble.reassembler);
        } else {
            start_advertising();
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        portENTER_CRITICAL(&s_ble.lock);
        s_ble.connected = false;
        s_ble.notify_enabled = false;
        s_ble.connection_handle = BLE_HS_CONN_HANDLE_NONE;
        portEXIT_CRITICAL(&s_ble.lock);
        ble_protocol_reassembler_reset(&s_ble.reassembler);
        xTaskNotify(s_ble.control_task, BLE_DISCONNECT_FLAG, eSetBits);
        start_advertising();
        return 0;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_ble.notify_value_handle) {
            portENTER_CRITICAL(&s_ble.lock);
            s_ble.notify_enabled = event->subscribe.cur_notify != 0U;
            portEXIT_CRITICAL(&s_ble.lock);
            if (event->subscribe.cur_notify != 0U) {
                xTaskNotify(s_ble.control_task, BLE_NOTIFY_FLAG, eSetBits);
            }
        }
        return 0;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertising();
        return 0;
    default:
        return 0;
    }
}

static void start_advertising(void)
{
    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = (ble_uuid128_t *)&s_service_uuid;
    fields.num_uuids128 = 1U;
    fields.uuids128_is_complete = 1U;

    const int fields_result = ble_gap_adv_set_fields(&fields);
    if (fields_result != 0) {
        ESP_LOGE(TAG, "Failed to set BLE advertising fields: %d", fields_result);
        return;
    }

    struct ble_hs_adv_fields response_fields = {0};
    const char *const name = ble_svc_gap_device_name();
    response_fields.name = (uint8_t *)name;
    response_fields.name_len = strlen(name);
    response_fields.name_is_complete = 1U;
    const int response_result = ble_gap_adv_rsp_set_fields(&response_fields);
    if (response_result != 0) {
        ESP_LOGE(TAG, "Failed to set BLE scan response: %d", response_result);
        return;
    }
    struct ble_gap_adv_params parameters = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
        .itvl_min = BLE_GAP_ADV_ITVL_MS(100),
        .itvl_max = BLE_GAP_ADV_ITVL_MS(120),
    };
    const int start_result = ble_gap_adv_start(
        s_ble.own_address_type, NULL, BLE_HS_FOREVER,
        &parameters, gap_event, NULL);
    if (start_result != 0) {
        ESP_LOGE(TAG, "Failed to start BLE advertising: %d", start_result);
    } else {
        ESP_LOGI(TAG, "Advertising as %s with NeuroStimRec service UUID", name);
    }
}

static void stack_reset(int reason)
{
    ESP_LOGE(TAG, "NimBLE stack reset: %d", reason);
}

static void stack_sync(void)
{
    if (ble_hs_util_ensure_addr(0) != 0 ||
        ble_hs_id_infer_auto(0, &s_ble.own_address_type) != 0) {
        ESP_LOGE(TAG, "Unable to select BLE identity address");
        return;
    }
    start_advertising();
}

static void gatt_register(struct ble_gatt_register_ctxt *context,
                          void *argument)
{
    (void)context;
    (void)argument;
}

static void nimble_host_task(void *parameter)
{
    (void)parameter;
    nimble_port_run();
    vTaskDelete(NULL);
}

esp_err_t ble_service_init(void)
{
    s_ble.command_queue = xQueueCreateStatic(
        BLE_COMMAND_QUEUE_DEPTH,
        sizeof(ble_protocol_command_t),
        s_ble.command_queue_bytes,
        &s_ble.command_queue_storage);
    if (s_ble.command_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCore(ble_control_task, "BLE_CONTROL",
                                BLE_CONTROL_TASK_STACK, NULL,
                                BLE_CONTROL_TASK_PRIORITY,
                                &s_ble.control_task, 1) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES ||
        result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        result = nvs_flash_erase();
        if (result == ESP_OK) {
            result = nvs_flash_init();
        }
    }
    if (result != ESP_OK) {
        return result;
    }
    result = nimble_port_init();
    if (result != ESP_OK) {
        return result;
    }
    ble_svc_gap_init();
    ble_svc_gatt_init();

    int host_result = ble_svc_gap_device_name_set(BLE_DEVICE_NAME);
    if (host_result != 0) {
        ESP_LOGE(TAG, "Failed to set BLE device name: rc=%d", host_result);
        return ESP_FAIL;
    }
    host_result = ble_gatts_count_cfg(s_gatt_services);
    if (host_result != 0) {
        ESP_LOGE(TAG, "Invalid BLE GATT service definition: rc=%d",
                 host_result);
        return ESP_FAIL;
    }
    host_result = ble_gatts_add_svcs(s_gatt_services);
    if (host_result != 0) {
        ESP_LOGE(TAG, "Failed to register BLE GATT service: rc=%d",
                 host_result);
        return ESP_FAIL;
    }
    ble_hs_cfg.reset_cb = stack_reset;
    ble_hs_cfg.sync_cb = stack_sync;
    ble_hs_cfg.gatts_register_cb = gatt_register;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_store_config_init();
    nimble_port_freertos_init(nimble_host_task);
    ESP_LOGI(TAG, "NimBLE GATT service ready as %s", BLE_DEVICE_NAME);
    return ESP_OK;
}
