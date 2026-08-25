#include "stim_controller.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stim_waveform.h"

static const char *TAG = "STIM_CONTROLLER";

#define STIM_ENABLE_GPIO GPIO_NUM_5
#define STIM_DEBOUNCE_US 100U
#define STIM_TASK_STACK_WORDS 3072U
#define STIM_TASK_PRIORITY 8U

#define STIM_NOTIFY_GPIO_EDGE (1U << 0)
#define STIM_NOTIFY_DEBOUNCE_EXPIRED (1U << 1)
#define STIM_NOTIFY_WAVEFORM_FAULT (1U << 2)
#define STIM_NOTIFY_SEQUENCE_COMPLETE (1U << 3)
#define STIM_NOTIFY_STOP_ACTIVE (1U << 4)

typedef struct {
    esp_timer_handle_t debounce_timer;
    StaticTask_t task_storage;
    StackType_t task_stack[STIM_TASK_STACK_WORDS];
    TaskHandle_t task;
    volatile uint32_t isr_edge_generation;
    uint32_t processed_edge_generation;
    int64_t last_edge_time_us;
    int pending_level;
    bool committed_enabled;
    bool gpio_handler_registered;
} stim_controller_context_t;

static stim_controller_context_t s_controller;

static void IRAM_ATTR stim_gpio_isr(void *argument)
{
    (void)argument;
    ++s_controller.isr_edge_generation;
    BaseType_t task_woken = pdFALSE;
    xTaskNotifyFromISR(s_controller.task,
                       STIM_NOTIFY_GPIO_EDGE,
                       eSetBits,
                       &task_woken);
    if (task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void stim_debounce_timer_callback(void *argument)
{
    (void)argument;
    xTaskNotify(s_controller.task,
                STIM_NOTIFY_DEBOUNCE_EXPIRED,
                eSetBits);
}

static bool IRAM_ATTR stim_waveform_event_isr(stim_waveform_state_t event,
                                              void *user_data)
{
    (void)user_data;
    uint32_t notification = 0U;
    if (event == STIM_WAVEFORM_FAULT) {
        notification = STIM_NOTIFY_WAVEFORM_FAULT;
    } else if (event == STIM_WAVEFORM_ENABLED_IDLE) {
        notification = STIM_NOTIFY_SEQUENCE_COMPLETE;
    } else if (event == STIM_WAVEFORM_STOP_LOOP) {
        notification = STIM_NOTIFY_STOP_ACTIVE;
    } else {
        return false;
    }
    BaseType_t task_woken = pdFALSE;
    xTaskNotifyFromISR(s_controller.task,
                       notification,
                       eSetBits,
                       &task_woken);
    return task_woken == pdTRUE;
}

static esp_err_t stim_arm_debounce_timer(uint64_t timeout_us)
{
    esp_err_t result;
    if (esp_timer_is_active(s_controller.debounce_timer)) {
        result = esp_timer_restart(s_controller.debounce_timer, timeout_us);
        if (result != ESP_ERR_INVALID_STATE) {
            return result;
        }
    }
    return esp_timer_start_once(s_controller.debounce_timer, timeout_us);
}

static void stim_controller_task(void *argument)
{
    (void)argument;
    uint32_t notifications = 0U;
    while (xTaskNotifyWait(0U,
                           UINT32_MAX,
                           &notifications,
                           portMAX_DELAY) == pdTRUE) {
        if ((notifications & STIM_NOTIFY_WAVEFORM_FAULT) != 0U) {
            stim_waveform_status_t status;
            stim_waveform_get_status(&status);
            ESP_LOGE(TAG,
                     "Stimulus pipeline fault: descriptor=%" PRIu32
                     ", fifo_underflow=%" PRIu32
                     ", unexpected_stop=%" PRIu32
                     "; CSb/MOSI forced high; ADC/SD capture remains active",
                     status.descriptor_errors,
                     status.fifo_underflow_errors,
                     status.unexpected_stop_errors);
            continue;
        }

        if ((notifications & STIM_NOTIFY_SEQUENCE_COMPLETE) != 0U) {
            ESP_LOGI(TAG, "Stimulus start sequence complete: 10/10 frames");
        }
        if ((notifications & STIM_NOTIFY_STOP_ACTIVE) != 0U) {
            ESP_LOGI(TAG, "Stimulus STOP_LOOP active at a 101-clock boundary");
        }

        if ((notifications & STIM_NOTIFY_GPIO_EDGE) != 0U) {
            s_controller.processed_edge_generation =
                s_controller.isr_edge_generation;
            s_controller.pending_level = gpio_get_level(STIM_ENABLE_GPIO);
            s_controller.last_edge_time_us = esp_timer_get_time();
            const esp_err_t result = stim_arm_debounce_timer(STIM_DEBOUNCE_US);
            if (result != ESP_OK) {
                ESP_LOGE(TAG, "Failed to arm debounce timer: %s",
                         esp_err_to_name(result));
            }
        }

        if ((notifications & STIM_NOTIFY_DEBOUNCE_EXPIRED) == 0U) {
            continue;
        }

        const uint32_t latest_generation = s_controller.isr_edge_generation;
        const int64_t elapsed_us =
            esp_timer_get_time() - s_controller.last_edge_time_us;
        if (s_controller.processed_edge_generation != latest_generation ||
            elapsed_us < STIM_DEBOUNCE_US) {
            s_controller.processed_edge_generation = latest_generation;
            s_controller.pending_level = gpio_get_level(STIM_ENABLE_GPIO);
            s_controller.last_edge_time_us = esp_timer_get_time();
            const esp_err_t result = stim_arm_debounce_timer(STIM_DEBOUNCE_US);
            if (result != ESP_OK) {
                ESP_LOGE(TAG, "Failed to re-arm debounce timer: %s",
                         esp_err_to_name(result));
            }
            continue;
        }

        const int stable_level = gpio_get_level(STIM_ENABLE_GPIO);
        if (stable_level != s_controller.pending_level) {
            continue;
        }
        const bool enabled = stable_level != 0;
        if (enabled == s_controller.committed_enabled) {
            continue;
        }
        const esp_err_t result = stim_waveform_request_enabled(enabled);
        if (result == ESP_OK) {
            s_controller.committed_enabled = enabled;
            ESP_LOGI(TAG,
                     "GPIO%d stable for %u us: stimulus %s requested",
                     STIM_ENABLE_GPIO, STIM_DEBOUNCE_US,
                     enabled ? "START_SEQUENCE" : "STOP_LOOP");
        } else {
            ESP_LOGE(TAG, "Stimulus transition failed: %s",
                     esp_err_to_name(result));
        }
    }
}

static void stim_controller_cleanup(void)
{
    if (s_controller.gpio_handler_registered) {
        (void)gpio_isr_handler_remove(STIM_ENABLE_GPIO);
        s_controller.gpio_handler_registered = false;
    }
    if (s_controller.debounce_timer != NULL) {
        if (esp_timer_is_active(s_controller.debounce_timer)) {
            (void)esp_timer_stop(s_controller.debounce_timer);
        }
        (void)esp_timer_delete(s_controller.debounce_timer);
        s_controller.debounce_timer = NULL;
    }
    stim_waveform_deinit();
    if (s_controller.task != NULL) {
        TaskHandle_t task = s_controller.task;
        s_controller.task = NULL;
        vTaskDelete(task);
    }
}

esp_err_t stim_controller_init(void)
{
    s_controller.task = xTaskCreateStaticPinnedToCore(
        stim_controller_task,
        "stim_ctrl",
        STIM_TASK_STACK_WORDS,
        NULL,
        STIM_TASK_PRIORITY,
        s_controller.task_stack,
        &s_controller.task_storage,
        1);
    if (s_controller.task == NULL) {
        stim_waveform_enter_safe_state();
        return ESP_ERR_NO_MEM;
    }

    esp_err_t result = stim_waveform_init(stim_waveform_event_isr, NULL);
    if (result != ESP_OK) {
        stim_controller_cleanup();
        return result;
    }

    const esp_timer_create_args_t timer_args = {
        .callback = stim_debounce_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "stim_db",
        .skip_unhandled_events = true,
    };
    result = esp_timer_create(&timer_args, &s_controller.debounce_timer);
    if (result != ESP_OK) {
        stim_controller_cleanup();
        return result;
    }

    const gpio_config_t input_config = {
        .pin_bit_mask = UINT64_C(1) << STIM_ENABLE_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    result = gpio_config(&input_config);
    if (result != ESP_OK) {
        stim_controller_cleanup();
        return result;
    }

    result = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        stim_controller_cleanup();
        return result;
    }
    result = gpio_isr_handler_add(STIM_ENABLE_GPIO, stim_gpio_isr, NULL);
    if (result != ESP_OK) {
        stim_controller_cleanup();
        return result;
    }
    s_controller.gpio_handler_registered = true;

    s_controller.pending_level = gpio_get_level(STIM_ENABLE_GPIO);
    s_controller.committed_enabled = false;
    s_controller.isr_edge_generation = 1U;
    xTaskNotify(s_controller.task, STIM_NOTIFY_GPIO_EDGE, eSetBits);
    ESP_LOGI(TAG,
             "GPIO%d stimulus enable ready: pull-down, %u us continuous-stable debounce",
             STIM_ENABLE_GPIO, STIM_DEBOUNCE_US);
    return ESP_OK;
}
