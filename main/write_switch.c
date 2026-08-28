#include "write_switch.h"

#include <stdint.h>

#include "driver/gpio.h"
#include "emmc_storage_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define WRITE_SWITCH_SAMPLE_MS 10U
#define WRITE_SWITCH_DEBOUNCE_SAMPLES 2U

static portMUX_TYPE switch_lock = portMUX_INITIALIZER_UNLOCKED;
static bool switch_armed = true;
static bool monitor_started;

bool write_switch_is_high(void)
{
    return gpio_get_level((gpio_num_t)CONFIG_EMMC_CTRL_WRITE_SWITCH_GPIO) != 0;
}

bool write_switch_is_armed(void)
{
    bool armed;
    taskENTER_CRITICAL(&switch_lock);
    armed = switch_armed;
    taskEXIT_CRITICAL(&switch_lock);
    return armed;
}

esp_err_t write_switch_init(void)
{
    const gpio_config_t config = {
        .pin_bit_mask = UINT64_C(1) << CONFIG_EMMC_CTRL_WRITE_SWITCH_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&config);
}

static void write_switch_task(void *parameter)
{
    (void)parameter;
    bool candidate = write_switch_is_high();
    bool stable = !candidate; /* Force handling of the power-on level. */
    unsigned int candidate_samples = 0U;

    for (;;) {
        const bool level = write_switch_is_high();
        if (level != candidate) {
            candidate = level;
            candidate_samples = 1U;
        } else if (candidate_samples < WRITE_SWITCH_DEBOUNCE_SAMPLES) {
            candidate_samples++;
        }

        if (candidate_samples >= WRITE_SWITCH_DEBOUNCE_SAMPLES &&
            candidate != stable) {
            stable = candidate;
            if (!stable) {
                taskENTER_CRITICAL(&switch_lock);
                switch_armed = true;
                taskEXIT_CRITICAL(&switch_lock);
                (void)emmc_storage_request_write_stop();
            } else {
                bool should_start;
                taskENTER_CRITICAL(&switch_lock);
                should_start = switch_armed;
                switch_armed = false;
                taskEXIT_CRITICAL(&switch_lock);
                if (should_start) {
                    (void)emmc_storage_request_write_start();
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(WRITE_SWITCH_SAMPLE_MS));
    }
}

esp_err_t write_switch_start_monitor(void)
{
    if (monitor_started) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xTaskCreatePinnedToCore(write_switch_task, "WRITE_SWITCH", 3072,
                                NULL, 7, NULL, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    monitor_started = true;
    return ESP_OK;
}
