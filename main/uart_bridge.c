#include "uart_bridge.h"

#include <ctype.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "driver/uart.h"
#include "emmc_storage_manager.h"
#include "esp_log.h"
#include "raw_sd_segment_format.h"
#include "sdkconfig.h"

#define COMMAND_BYTES 160U
#define PACKET_PAYLOAD_BYTES 4096U
#define PACKET_MAGIC "EMB1"

typedef struct __attribute__((packed)) {
    uint8_t magic[4];
    uint32_t sequence;
    uint64_t stream_offset;
    uint32_t payload_bytes;
    uint32_t payload_crc32;
} packet_header_t;

typedef enum {
    READ_LIST,
    READ_DATA,
    READ_LBA,
    READ_EVENTS,
} read_command_t;

typedef struct {
    read_command_t command;
    uint64_t first;
    uint64_t offset;
    uint64_t length;
    bool has_offset;
    bool has_length;
    bool executed;
} read_context_t;

_Static_assert(sizeof(packet_header_t) == 24U,
               "EMB1 packet header must be 24 bytes");

static const uart_port_t bridge_uart =
    (uart_port_t)CONFIG_EMMC_CTRL_UART_PORT;

static bool uart_write_all(const void *data, size_t bytes)
{
    const uint8_t *cursor = data;
    while (bytes > 0U) {
        const int written = uart_write_bytes(bridge_uart, cursor, bytes);
        if (written <= 0) {
            return false;
        }
        cursor += (size_t)written;
        bytes -= (size_t)written;
    }
    return true;
}

static bool uart_line(const char *format, ...)
{
    char line[448];
    va_list args;
    va_start(args, format);
    const int length = vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    return length >= 0 && (size_t)length < sizeof(line) &&
           uart_write_all(line, (size_t)length) &&
           uart_write_all("\r\n", 2U);
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t bytes)
{
    crc = ~crc;
    for (size_t index = 0U; index < bytes; ++index) {
        crc ^= data[index];
        for (unsigned int bit = 0U; bit < 8U; ++bit) {
            const uint32_t mask = (uint32_t)-(int32_t)(crc & 1U);
            crc = (crc >> 1U) ^ (UINT32_C(0xEDB88320) & mask);
        }
    }
    return ~crc;
}

static bool parse_u64(const char *text, uint64_t *value)
{
    if (text == NULL || value == NULL || *text == '\0' || *text == '-') {
        return false;
    }
    char *end = NULL;
    const unsigned long long parsed = strtoull(text, &end, 0);
    if (end == text || *end != '\0') {
        return false;
    }
    *value = (uint64_t)parsed;
    return true;
}

static esp_err_t read_sectors(emmc_storage_access_t *access, uint64_t lba,
                              size_t count, const uint8_t **data)
{
    const uint64_t capacity = (uint64_t)access->card->csd.capacity;
    if (count == 0U || count > access->dma_buffer_sectors ||
        lba >= capacity || count > capacity - lba || lba > SIZE_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }
    const esp_err_t result = sdmmc_read_sectors(
        access->card, access->dma_buffer, (size_t)lba, count);
    if (result == ESP_OK) {
        *data = access->dma_buffer;
    } else {
        access->card_error = result;
    }
    return result;
}

static esp_err_t read_sector_copy(emmc_storage_access_t *access,
                                  uint64_t lba, void *destination)
{
    const uint8_t *data = NULL;
    const esp_err_t result = read_sectors(access, lba, 1U, &data);
    if (result == ESP_OK) {
        memcpy(destination, data, RAW_SD_SECTOR_BYTES);
    }
    return result;
}

static esp_err_t load_superblock(emmc_storage_access_t *access,
                                 raw_sd_superblock_t *superblock)
{
    raw_sd_superblock_t copy_a;
    raw_sd_superblock_t copy_b;
    const esp_err_t result_a = read_sector_copy(
        access, RAW_SD_SUPERBLOCK_LBA_A, &copy_a);
    const esp_err_t error_a = access->card_error;
    access->card_error = ESP_OK;
    const esp_err_t result_b = read_sector_copy(
        access, RAW_SD_SUPERBLOCK_LBA_B, &copy_b);
    const bool valid_a = result_a == ESP_OK &&
                         raw_sd_superblock_is_valid(&copy_a);
    const bool valid_b = result_b == ESP_OK &&
                         raw_sd_superblock_is_valid(&copy_b);
    if (!valid_a && !valid_b) {
        if (result_a != ESP_OK) {
            access->card_error = error_a;
            return result_a;
        }
        if (result_b != ESP_OK) {
            return result_b;
        }
        access->card_error = ESP_OK;
        return ESP_ERR_INVALID_CRC;
    }
    access->card_error = ESP_OK;
    *superblock = valid_b && (!valid_a || copy_b.generation > copy_a.generation)
        ? copy_b : copy_a;
    return ESP_OK;
}

static esp_err_t load_segment(emmc_storage_access_t *access, uint32_t index,
                              raw_sd_segment_t *segment)
{
    if (index >= RAW_SD_SEGMENT_DIRECTORY_CAPACITY) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t result = read_sector_copy(
        access, RAW_SD_SEGMENT_DIRECTORY_START_LBA + index, segment);
    if (result == ESP_OK && !raw_sd_segment_is_valid(segment)) {
        result = ESP_ERR_INVALID_CRC;
    }
    return result;
}

static const char *segment_state_name(uint32_t state)
{
    switch (state) {
    case RAW_SD_SEGMENT_OPEN: return "OPEN";
    case RAW_SD_SEGMENT_CLOSED: return "CLOSED";
    case RAW_SD_SEGMENT_FAILED: return "FAILED";
    default: return "UNKNOWN";
    }
}

static esp_err_t stream_bytes(emmc_storage_access_t *access,
                              uint64_t start_lba, uint64_t byte_offset,
                              uint64_t total_bytes)
{
    const uint64_t sectors = (uint64_t)access->card->csd.capacity;
    if (start_lba >= sectors || start_lba > UINT64_MAX / RAW_SD_SECTOR_BYTES) {
        uart_line("ERR RANGE invalid_start_lba=%" PRIu64, start_lba);
        return ESP_ERR_INVALID_ARG;
    }
    const uint64_t capacity_bytes = sectors * RAW_SD_SECTOR_BYTES;
    const uint64_t start_byte = start_lba * RAW_SD_SECTOR_BYTES;
    if (byte_offset > capacity_bytes - start_byte ||
        total_bytes > capacity_bytes - start_byte - byte_offset) {
        uart_line("ERR RANGE offset=%" PRIu64 " length=%" PRIu64,
                  byte_offset, total_bytes);
        return ESP_ERR_INVALID_SIZE;
    }
    if (!uart_line("OK STREAM total=%" PRIu64 " packet_header=24",
                   total_bytes)) {
        return ESP_FAIL;
    }

    uint64_t sent = 0U;
    uint32_t sequence = 0U;
    uint32_t stream_crc = 0U;
    while (sent < total_bytes) {
        const uint64_t absolute = start_byte + byte_offset + sent;
        const uint64_t lba = absolute / RAW_SD_SECTOR_BYTES;
        const size_t in_sector = (size_t)(absolute % RAW_SD_SECTOR_BYTES);
        size_t payload_bytes = (size_t)(total_bytes - sent);
        if (payload_bytes > PACKET_PAYLOAD_BYTES) {
            payload_bytes = PACKET_PAYLOAD_BYTES;
        }
        const size_t sector_count =
            (in_sector + payload_bytes + RAW_SD_SECTOR_BYTES - 1U) /
            RAW_SD_SECTOR_BYTES;
        const uint8_t *sector_data = NULL;
        const esp_err_t result = read_sectors(
            access, lba, sector_count, &sector_data);
        if (result != ESP_OK) {
            uart_line("ERR READ lba=%" PRIu64 " code=0x%x name=%s",
                      lba, (unsigned int)result, esp_err_to_name(result));
            return result;
        }
        const uint8_t *payload = sector_data + in_sector;
        packet_header_t header = {
            .sequence = sequence,
            .stream_offset = sent,
            .payload_bytes = (uint32_t)payload_bytes,
            .payload_crc32 = crc32_update(0U, payload, payload_bytes),
        };
        memcpy(header.magic, PACKET_MAGIC, sizeof(header.magic));
        if (!uart_write_all(&header, sizeof(header)) ||
            !uart_write_all(payload, payload_bytes)) {
            return ESP_FAIL;
        }
        stream_crc = crc32_update(stream_crc, payload, payload_bytes);
        sent += payload_bytes;
        sequence++;
    }
    return uart_line("OK END bytes=%" PRIu64 " crc32=%08" PRIX32,
                     sent, stream_crc) ? ESP_OK : ESP_FAIL;
}

static esp_err_t command_list(emmc_storage_access_t *access)
{
    raw_sd_superblock_t superblock;
    esp_err_t result = load_superblock(access, &superblock);
    if (result != ESP_OK) {
        uart_line("ERR LIST metadata code=0x%x name=%s",
                  (unsigned int)result, esp_err_to_name(result));
        return ESP_OK;
    }
    uint32_t count = superblock.segment_count;
    if (count > RAW_SD_SEGMENT_DIRECTORY_CAPACITY) {
        count = RAW_SD_SEGMENT_DIRECTORY_CAPACITY;
    }
    uart_line("OK LIST count=%" PRIu32, count);
    for (uint32_t index = 0U; index < count; ++index) {
        raw_sd_segment_t segment;
        result = load_segment(access, index, &segment);
        if (result == ESP_OK) {
            uart_line("SEG index=%" PRIu32 " id=%" PRIu32
                      " state=%s run=%" PRIu32 " start_lba=%" PRIu64
                      " physical=%" PRIu64 " valid=%" PRIu64
                      " frames=%" PRIu64 " data_crc=%08" PRIX32
                      " outcome=%" PRIu32 " events=%" PRIu32,
                      index, segment.segment_id,
                      segment_state_name(segment.state), segment.run_id,
                      segment.start_lba,
                      segment.physical_bytes, segment.valid_bytes,
                      segment.frame_count, segment.data_checksum,
                      segment.capture_outcome,
                      segment.event_count);
        } else {
            uart_line("SEG index=%" PRIu32 " invalid=1 code=0x%x",
                      index, (unsigned int)result);
        }
    }
    uart_line("OK END");
    return ESP_OK;
}

static esp_err_t execute_read(emmc_storage_access_t *access, void *opaque)
{
    read_context_t *context = opaque;
    context->executed = true;
    if (context->command == READ_LIST) {
        return command_list(access);
    }
    if (context->command == READ_LBA) {
        return stream_bytes(access, context->first, 0U,
                            context->length * RAW_SD_SECTOR_BYTES);
    }

    raw_sd_segment_t segment;
    const esp_err_t result = load_segment(
        access, (uint32_t)context->first, &segment);
    if (result != ESP_OK) {
        uart_line("ERR METADATA index=%" PRIu64 " code=0x%x name=%s",
                  context->first, (unsigned int)result,
                  esp_err_to_name(result));
        return ESP_OK;
    }
    if (context->command == READ_EVENTS) {
        return stream_bytes(access, segment.event_start_lba, 0U,
                            (uint64_t)segment.event_sector_count *
                            RAW_SD_SECTOR_BYTES);
    }

    const uint64_t offset = context->has_offset ? context->offset : 0U;
    if (offset > segment.valid_bytes) {
        uart_line("ERR RANGE valid_bytes=%" PRIu64, segment.valid_bytes);
        return ESP_OK;
    }
    const uint64_t length = context->has_length
        ? context->length : segment.valid_bytes - offset;
    if (length > segment.valid_bytes - offset) {
        uart_line("ERR RANGE valid_bytes=%" PRIu64, segment.valid_bytes);
        return ESP_OK;
    }
    return stream_bytes(access, segment.start_lba, offset, length);
}

static void report_storage_error(esp_err_t result)
{
    emmc_status_t status;
    (void)emmc_storage_get_status(&status);
    if (result == EMMC_STORAGE_ERR_INTERLOCK) {
        uart_line("ERR INTERLOCK gpio7=1 reason=WRITE_SWITCH_ACTIVE");
    } else if (result == EMMC_STORAGE_ERR_BUSY) {
        uart_line("ERR BUSY mode=%s", emmc_storage_state_name(status.state));
    } else if (result == EMMC_STORAGE_ERR_CARD) {
        uart_line("ERR CARD state=%s code=0x%x name=%s",
                  emmc_storage_state_name(status.state),
                  (unsigned int)status.failure_code,
                  esp_err_to_name(status.failure_code));
    } else {
        uart_line("ERR STORAGE code=0x%x name=%s",
                  (unsigned int)result, esp_err_to_name(result));
    }
}

static void command_status(void)
{
    emmc_status_t status;
    if (emmc_storage_get_status(&status) != ESP_OK) {
        uart_line("ERR STATUS unavailable");
        return;
    }
    const double raw_rate = status.write_elapsed_us > 0U
        ? ((double)status.physical_bytes * 1000000.0) /
          ((double)status.write_elapsed_us * 1024.0 * 1024.0)
        : 0.0;
    const double end_to_end_rate = status.wall_elapsed_us > 0U
        ? ((double)status.physical_bytes * 1000000.0) /
          ((double)status.wall_elapsed_us * 1024.0 * 1024.0)
        : 0.0;
    uart_line("OK STATUS state=%s gpio7=%s write_switch=%s write_armed=%u"
              " card=%s bytes=%" PRIu64 " target=%" PRIu64
              " failure=0x%x",
              emmc_storage_state_name(status.state),
              status.gpio7_high ? "HIGH" : "LOW",
              status.gpio7_high ? "ON" : "OFF",
              status.write_armed ? 1U : 0U,
              status.card_ready ? "READY" : "ERROR",
              status.physical_bytes, status.target_bytes,
              (unsigned int)status.failure_code);
    uart_line("OK BENCH result_available=%u valid=%" PRIu64
              " verified=%" PRIu64 " write_us=%" PRIu64
              " wall_us=%" PRIu64 " raw_mib_s=%.2f"
              " end_to_end_mib_s=%.2f",
              status.result_available ? 1U : 0U,
              status.valid_bytes, status.verified_samples,
              status.write_elapsed_us, status.wall_elapsed_us,
              raw_rate, end_to_end_rate);
    uart_line("OK END");
}

static void command_info(void)
{
    emmc_status_t status;
    (void)emmc_storage_get_status(&status);
    uart_line("OK INFO sector_bytes=%" PRIu32
              " capacity_sectors=%" PRIu64
              " capacity_bytes=%" PRIu64
              " emmc_clock_khz=%" PRIu32
              " uart_baud=%d format=ADCSEG1 version=%" PRIu32,
              RAW_SD_SECTOR_BYTES, status.capacity_sectors,
              status.capacity_sectors * RAW_SD_SECTOR_BYTES,
              status.emmc_clock_khz, CONFIG_EMMC_CTRL_UART_BAUD_RATE,
              RAW_SD_FORMAT_VERSION);
    uart_line("OK END");
}

static void command_help(void)
{
    uart_line("OK HELP");
    uart_line("PING | STATUS | INFO | LIST | REINIT");
    uart_line("DATA <segment_index> [byte_offset] [byte_length]");
    uart_line("READ <lba> <sector_count>");
    uart_line("EVENTS <segment_index>");
    uart_line("GPIO7 high starts ADC capture; low stops and permits reads");
    uart_line("OK END");
}

static void run_read(read_context_t *context)
{
    const esp_err_t result = emmc_storage_execute_read(execute_read, context);
    if (!context->executed && result != ESP_OK) {
        report_storage_error(result);
    }
}

static void dispatch_command(char *line)
{
    char *save = NULL;
    char *command = strtok_r(line, " \t", &save);
    char *arg1 = strtok_r(NULL, " \t", &save);
    char *arg2 = strtok_r(NULL, " \t", &save);
    char *arg3 = strtok_r(NULL, " \t", &save);
    char *extra = strtok_r(NULL, " \t", &save);
    if (command == NULL) {
        return;
    }
    if (extra != NULL) {
        uart_line("ERR ARG too_many_arguments");
        return;
    }
    if (strcasecmp(command, "PING") == 0 && arg1 == NULL) {
        uart_line("OK PONG");
        uart_line("OK END");
    } else if (strcasecmp(command, "STATUS") == 0 && arg1 == NULL) {
        command_status();
    } else if (strcasecmp(command, "INFO") == 0 && arg1 == NULL) {
        command_info();
    } else if (strcasecmp(command, "HELP") == 0 && arg1 == NULL) {
        command_help();
    } else if (strcasecmp(command, "REINIT") == 0 && arg1 == NULL) {
        const esp_err_t result = emmc_storage_request_reinit();
        if (result == ESP_OK) {
            uart_line("OK REINIT");
            uart_line("OK END");
        } else {
            report_storage_error(result);
        }
    } else {
        read_context_t context = {0};
        bool valid = true;
        if (strcasecmp(command, "LIST") == 0 && arg1 == NULL) {
            context.command = READ_LIST;
        } else if (strcasecmp(command, "READ") == 0 && arg1 != NULL &&
                   arg2 != NULL && arg3 == NULL &&
                   parse_u64(arg1, &context.first) &&
                   parse_u64(arg2, &context.length) &&
                   context.length > 0U &&
                   context.length <= UINT64_MAX / RAW_SD_SECTOR_BYTES) {
            context.command = READ_LBA;
        } else if (strcasecmp(command, "DATA") == 0 && arg1 != NULL &&
                   parse_u64(arg1, &context.first) &&
                   context.first < RAW_SD_SEGMENT_DIRECTORY_CAPACITY) {
            context.command = READ_DATA;
            if (arg2 != NULL) {
                context.has_offset = parse_u64(arg2, &context.offset);
                valid = context.has_offset;
            }
            if (valid && arg3 != NULL) {
                context.has_length = parse_u64(arg3, &context.length);
                valid = context.has_length;
            }
        } else if (strcasecmp(command, "EVENTS") == 0 && arg1 != NULL &&
                   arg2 == NULL && parse_u64(arg1, &context.first) &&
                   context.first < RAW_SD_SEGMENT_DIRECTORY_CAPACITY) {
            context.command = READ_EVENTS;
        } else {
            valid = false;
        }
        if (valid) {
            run_read(&context);
        } else {
            uart_line("ERR COMMAND type_HELP_for_usage");
        }
    }
}

esp_err_t uart_bridge_init(void)
{
    const uart_config_t config = {
        .baud_rate = CONFIG_EMMC_CTRL_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t result = uart_driver_install(bridge_uart, 2048, 8192,
                                           0, NULL, 0);
    if (result != ESP_OK) {
        return result;
    }
    result = uart_param_config(bridge_uart, &config);
    if (result == ESP_OK) {
        result = uart_set_pin(bridge_uart,
                              CONFIG_EMMC_CTRL_UART_TX_GPIO,
                              CONFIG_EMMC_CTRL_UART_RX_GPIO,
                              UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (result != ESP_OK) {
        (void)uart_driver_delete(bridge_uart);
    } else {
        /* UART0 is the production control/data channel. ESP_LOG uses the
         * same console UART and can corrupt STATUS or EMB1 framing, so all
         * subsequent diagnostics are reported through protocol responses. */
        esp_log_level_set("*", ESP_LOG_NONE);
    }
    return result;
}

void uart_bridge_run(void)
{
    uart_line("EMMC_ADC_CTRL READY version=1 mode=REAL_ADC");
    uart_line("Type HELP for commands");

    char command[COMMAND_BYTES];
    size_t used = 0U;
    for (;;) {
        uint8_t byte = 0U;
        const int received = uart_read_bytes(
            bridge_uart, &byte, 1U, portMAX_DELAY);
        if (received <= 0) {
            continue;
        }
        if (byte == '\r' || byte == '\n') {
            if (used > 0U) {
                command[used] = '\0';
                dispatch_command(command);
                used = 0U;
            }
        } else if (byte == '\b' || byte == 0x7FU) {
            if (used > 0U) {
                used--;
            }
        } else if (isprint((int)byte)) {
            if (used + 1U < sizeof(command)) {
                command[used++] = (char)byte;
            } else {
                used = 0U;
                uart_line("ERR COMMAND line_too_long");
            }
        }
    }
}
