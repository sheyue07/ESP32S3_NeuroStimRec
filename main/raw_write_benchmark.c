/* Raw-SD write benchmark: deliberately overwrites the selected card LBAs. */

#include "raw_write_benchmark.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "frame_sync.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"

static const char *TAG = "RAW_SD_BENCH";

#define RAW_BENCHMARK_METADATA_LBA_A  UINT32_C(0)
#define RAW_BENCHMARK_METADATA_LBA_B  UINT32_C(1)
#define RAW_BENCHMARK_DATA_START_LBA       UINT32_C(2048)
#define RAW_BENCHMARK_SECTOR_BYTES          512U
#define RAW_BENCHMARK_WRITE_BUFFER_BYTES   (64U * 1024U)
#define RAW_BENCHMARK_SECTORS_PER_WRITE    128U
#define RAW_BENCHMARK_PHYSICAL_BYTES       UINT64_C(1073741824)
#define RAW_BENCHMARK_VALID_BYTES          UINT64_C(1073741760)
#define RAW_BENCHMARK_FRAME_COUNT          UINT64_C(4129776)
#define RAW_BENCHMARK_RATE_INTERVAL_US     INT64_C(1000000)
#define RAW_BENCHMARK_METADATA_UPDATE_BYTES (64U * 1024U * 1024U)

#define SDMMC_PIN_CLK                       GPIO_NUM_41
#define SDMMC_PIN_CMD                       GPIO_NUM_42
#define SDMMC_PIN_D0                        GPIO_NUM_40
#define SDMMC_PIN_D1                        GPIO_NUM_39
#define SDMMC_PIN_D2                        GPIO_NUM_1
#define SDMMC_PIN_D3                        GPIO_NUM_2
#define SDMMC_MAX_FREQ_KHZ                  20000

#define RECORD_SWITCH_GPIO                  GPIO_NUM_18
#define SWITCH_DEBOUNCE_MS                  20

typedef enum {
    RAW_BENCHMARK_STATE_RUNNING = 1,
    RAW_BENCHMARK_STATE_PASS = 2,
    RAW_BENCHMARK_STATE_FAILED = 3,
} raw_benchmark_state_t;

typedef struct __attribute__((packed)) {
    uint8_t magic[8];
    uint32_t version;
    uint32_t state;
    uint32_t data_start_lba;
    uint32_t reserved0;
    uint64_t valid_bytes;
    uint64_t physical_bytes;
    uint64_t complete_frames;
    uint64_t elapsed_us;
    uint32_t failure_code;
    uint32_t checksum;
    uint8_t reserved[448];
} raw_benchmark_metadata_t;

_Static_assert(sizeof(raw_benchmark_metadata_t) == RAW_BENCHMARK_SECTOR_BYTES,
               "metadata must occupy exactly one sector");
_Static_assert(RAW_BENCHMARK_WRITE_BUFFER_BYTES ==
                   RAW_BENCHMARK_SECTOR_BYTES * RAW_BENCHMARK_SECTORS_PER_WRITE,
               "write buffer must contain an integer number of sectors");
_Static_assert(RAW_BENCHMARK_VALID_BYTES % ADC_FRAME_SIZE_BYTES == 0U,
               "valid benchmark bytes must contain complete frames");

static sdmmc_card_t raw_benchmark_card;

static uint32_t metadata_checksum(const raw_benchmark_metadata_t *metadata)
{
    const uint8_t *bytes = (const uint8_t *)metadata;
    uint32_t checksum = 0;
    for (size_t index = 0; index < sizeof(*metadata); ++index) {
        checksum = checksum * 33U + bytes[index];
    }
    return checksum;
}

static esp_err_t write_metadata(raw_benchmark_state_t state,
                                uint64_t valid_bytes,
                                uint64_t physical_bytes,
                                uint64_t elapsed_us,
                                esp_err_t failure_code)
{
    raw_benchmark_metadata_t metadata;
    memset(&metadata, 0, sizeof(metadata));
    memcpy(metadata.magic, "ADC_RAW1", sizeof(metadata.magic));
    metadata.version = 1U;
    metadata.state = (uint32_t)state;
    metadata.data_start_lba = RAW_BENCHMARK_DATA_START_LBA;
    metadata.valid_bytes = valid_bytes;
    metadata.physical_bytes = physical_bytes;
    metadata.complete_frames = valid_bytes / ADC_FRAME_SIZE_BYTES;
    metadata.elapsed_us = elapsed_us;
    metadata.failure_code = (uint32_t)failure_code;
    metadata.checksum = 0U;
    metadata.checksum = metadata_checksum(&metadata);

    esp_err_t result = sdmmc_write_sectors(&raw_benchmark_card,
                                           &metadata,
                                           RAW_BENCHMARK_METADATA_LBA_A,
                                           1U);
    if (result != ESP_OK) {
        return result;
    }
    return sdmmc_write_sectors(&raw_benchmark_card,
                               &metadata,
                               RAW_BENCHMARK_METADATA_LBA_B,
                               1U);
}

static void fill_test_frame(uint8_t frame[ADC_FRAME_SIZE_BYTES])
{
    frame[0] = 0xFF;
    frame[1] = 0xFF;
    frame[2] = 0x00;
    frame[3] = 0x00;
    for (size_t channel = 0; channel < ADC_CHANNEL_COUNT; ++channel) {
        const size_t offset = 4U + channel * 4U;
        frame[offset] = 0x12;
        frame[offset + 1U] = 0x34;
        frame[offset + 2U] = 0x00;
        frame[offset + 3U] = 0x00;
    }
}

static void fill_write_buffer(uint8_t *buffer,
                              const uint8_t frame[ADC_FRAME_SIZE_BYTES],
                              uint64_t physical_offset)
{
    size_t buffer_offset = 0U;
    uint64_t source_offset = physical_offset;
    while (buffer_offset < RAW_BENCHMARK_WRITE_BUFFER_BYTES) {
        if (source_offset >= RAW_BENCHMARK_VALID_BYTES) {
            memset(buffer + buffer_offset, 0,
                   RAW_BENCHMARK_WRITE_BUFFER_BYTES - buffer_offset);
            return;
        }

        const size_t frame_offset =
            (size_t)(source_offset % ADC_FRAME_SIZE_BYTES);
        size_t copy_size = ADC_FRAME_SIZE_BYTES - frame_offset;
        const uint64_t valid_remaining =
            RAW_BENCHMARK_VALID_BYTES - source_offset;
        const size_t buffer_remaining =
            RAW_BENCHMARK_WRITE_BUFFER_BYTES - buffer_offset;
        if (copy_size > valid_remaining) {
            copy_size = (size_t)valid_remaining;
        }
        if (copy_size > buffer_remaining) {
            copy_size = buffer_remaining;
        }

        memcpy(buffer + buffer_offset, frame + frame_offset, copy_size);
        buffer_offset += copy_size;
        source_offset += copy_size;
    }
}

static esp_err_t init_raw_sdmmc(void)
{
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_MAX_FREQ_KHZ;
    host.command_timeout_ms = 3000;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;
    slot_config.clk = SDMMC_PIN_CLK;
    slot_config.cmd = SDMMC_PIN_CMD;
    slot_config.d0 = SDMMC_PIN_D0;
    slot_config.d1 = SDMMC_PIN_D1;
    slot_config.d2 = SDMMC_PIN_D2;
    slot_config.d3 = SDMMC_PIN_D3;
    slot_config.cd = GPIO_NUM_NC;
    slot_config.wp = GPIO_NUM_NC;

    esp_err_t result = sdmmc_host_init();
    if (result != ESP_OK) {
        return result;
    }
    result = sdmmc_host_init_slot(host.slot, &slot_config);
    if (result != ESP_OK) {
        (void)sdmmc_host_deinit();
        return result;
    }
    memset(&raw_benchmark_card, 0, sizeof(raw_benchmark_card));
    result = sdmmc_card_init(&host, &raw_benchmark_card);
    if (result != ESP_OK) {
        (void)sdmmc_host_deinit();
    }
    return result;
}

static void log_interval_rate(uint64_t interval_bytes, int64_t elapsed_us)
{
    if (elapsed_us <= 0) {
        return;
    }
    const double rate_mib_per_second =
        ((double)interval_bytes * 1000000.0) /
        ((double)elapsed_us * 1024.0 * 1024.0);
    ESP_LOGI(TAG, "RAW BENCH rate=%.2f MiB/s", rate_mib_per_second);
}

static esp_err_t run_raw_benchmark(void)
{
    const uint64_t required_sectors = RAW_BENCHMARK_DATA_START_LBA +
        RAW_BENCHMARK_PHYSICAL_BYTES / RAW_BENCHMARK_SECTOR_BYTES;
    if (raw_benchmark_card.csd.sector_size != RAW_BENCHMARK_SECTOR_BYTES) {
        ESP_LOGE(TAG, "Unsupported sector size: %d bytes",
                 raw_benchmark_card.csd.sector_size);
        return ESP_ERR_NOT_SUPPORTED;
    }
    if ((uint64_t)raw_benchmark_card.csd.capacity < required_sectors) {
        ESP_LOGE(TAG, "Card too small: sectors=%d required=%" PRIu64,
                 raw_benchmark_card.csd.capacity, required_sectors);
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t *buffer = heap_caps_malloc(RAW_BENCHMARK_WRITE_BUFFER_BYTES,
                                       MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (buffer == NULL) {
        return ESP_ERR_NO_MEM;
    }

    uint8_t frame[ADC_FRAME_SIZE_BYTES];
    fill_test_frame(frame);

    esp_err_t result = write_metadata(RAW_BENCHMARK_STATE_RUNNING,
                                      0U, 0U, 0U, ESP_OK);
    uint64_t written_bytes = 0U;
    int64_t write_elapsed_us = 0;
    if (result == ESP_OK) {
        int64_t interval_write_elapsed_us = 0;
        uint64_t interval_written_bytes = 0U;
        uint64_t next_metadata_update_bytes =
            RAW_BENCHMARK_METADATA_UPDATE_BYTES;

        while (written_bytes < RAW_BENCHMARK_PHYSICAL_BYTES) {
            fill_write_buffer(buffer, frame, written_bytes);
            const size_t lba = RAW_BENCHMARK_DATA_START_LBA +
                (size_t)(written_bytes / RAW_BENCHMARK_SECTOR_BYTES);
            const int64_t write_start_us = esp_timer_get_time();
            result = sdmmc_write_sectors(&raw_benchmark_card,
                                         buffer,
                                         lba,
                                         RAW_BENCHMARK_SECTORS_PER_WRITE);
            const int64_t write_end_us = esp_timer_get_time();
            if (result != ESP_OK) {
                break;
            }
            const int64_t current_write_elapsed_us =
                write_end_us - write_start_us;
            write_elapsed_us += current_write_elapsed_us;
            interval_write_elapsed_us += current_write_elapsed_us;
            written_bytes += RAW_BENCHMARK_WRITE_BUFFER_BYTES;
            interval_written_bytes += RAW_BENCHMARK_WRITE_BUFFER_BYTES;

            if (written_bytes >= next_metadata_update_bytes) {
                const uint64_t progress_valid_bytes =
                    written_bytes > RAW_BENCHMARK_VALID_BYTES
                        ? RAW_BENCHMARK_VALID_BYTES
                        : written_bytes -
                              (written_bytes % ADC_FRAME_SIZE_BYTES);
                result = write_metadata(RAW_BENCHMARK_STATE_RUNNING,
                                        progress_valid_bytes,
                                        written_bytes,
                                        (uint64_t)write_elapsed_us,
                                        ESP_OK);
                if (result != ESP_OK) {
                    break;
                }
                next_metadata_update_bytes +=
                    RAW_BENCHMARK_METADATA_UPDATE_BYTES;
            }

            if (interval_write_elapsed_us >= RAW_BENCHMARK_RATE_INTERVAL_US) {
                log_interval_rate(interval_written_bytes,
                                  interval_write_elapsed_us);
                interval_write_elapsed_us = 0;
                interval_written_bytes = 0U;
            }
        }
    }

    const uint64_t valid_bytes = written_bytes > RAW_BENCHMARK_VALID_BYTES
                                     ? RAW_BENCHMARK_VALID_BYTES
                                     : written_bytes -
                                           (written_bytes % ADC_FRAME_SIZE_BYTES);
    esp_err_t metadata_result;
    if (result == ESP_OK) {
        metadata_result = write_metadata(RAW_BENCHMARK_STATE_PASS,
                                         valid_bytes,
                                         written_bytes,
                                         (uint64_t)write_elapsed_us,
                                         ESP_OK);
    } else {
        metadata_result = write_metadata(RAW_BENCHMARK_STATE_FAILED,
                                         valid_bytes,
                                         written_bytes,
                                         (uint64_t)write_elapsed_us,
                                         result);
    }
    heap_caps_free(buffer);

    if (metadata_result != ESP_OK) {
        ESP_LOGE(TAG, "Metadata write failed: 0x%x (%s)",
                 (unsigned int)metadata_result, esp_err_to_name(metadata_result));
        return metadata_result;
    }
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "RAW BENCH FAILED: written=%" PRIu64 " error=0x%x (%s)",
                 written_bytes, (unsigned int)result, esp_err_to_name(result));
        return result;
    }

    const double average_mib_per_second = write_elapsed_us > 0
        ? ((double)written_bytes * 1000000.0) /
              ((double)write_elapsed_us * 1024.0 * 1024.0)
        : 0.0;
    ESP_LOGI(TAG,
             "RAW BENCH PASS: valid=%" PRIu64 " physical=%" PRIu64
             " frames=%" PRIu64 " elapsed=%.3f s average=%.2f MiB/s",
             valid_bytes,
             written_bytes,
             valid_bytes / ADC_FRAME_SIZE_BYTES,
             (double)write_elapsed_us / 1000000.0,
             average_mib_per_second);
    return ESP_OK;
}

static void raw_benchmark_task(void *parameter)
{
    (void)parameter;
    bool start_armed = gpio_get_level(RECORD_SWITCH_GPIO) == 0;

    ESP_LOGI(TAG, "GPIO18 high starts one 1 GiB raw-write benchmark");
    while (true) {
        if (gpio_get_level(RECORD_SWITCH_GPIO) == 0) {
            start_armed = true;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (!start_armed) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(SWITCH_DEBOUNCE_MS));
        if (gpio_get_level(RECORD_SWITCH_GPIO) == 0) {
            continue;
        }
        start_armed = false;
        const esp_err_t result = run_raw_benchmark();
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "RAW BENCH result: 0x%x (%s)",
                     (unsigned int)result, esp_err_to_name(result));
        }
    }
}

void raw_write_benchmark_app_main(void)
{
    const gpio_config_t switch_config = {
        .pin_bit_mask = UINT64_C(1) << RECORD_SWITCH_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t result = gpio_config(&switch_config);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "GPIO18 initialization failed: 0x%x (%s)",
                 (unsigned int)result, esp_err_to_name(result));
        return;
    }

    ESP_LOGW(TAG, "Raw mode overwrites LBA 0 through the 1 GiB benchmark range");
    result = init_raw_sdmmc();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Raw SDMMC initialization failed: 0x%x (%s)",
                 (unsigned int)result, esp_err_to_name(result));
        return;
    }
    sdmmc_card_print_info(stdout, &raw_benchmark_card);

    if (xTaskCreatePinnedToCore(raw_benchmark_task,
                                "RAW_SD_BENCH",
                                4096,
                                NULL,
                                6,
                                NULL,
                                1) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create raw benchmark task");
        (void)sdmmc_host_deinit();
    }
}

