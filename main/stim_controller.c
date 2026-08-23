#include "stim_controller.h"

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "stim_waveform.h"

static const char *TAG = "STIM_CONTROLLER";

#define STIM_ENABLE_GPIO GPIO_NUM_6
#define STIM_DEBOUNCE_US 100U
#define STIM_EVENT_QUEUE_LENGTH 16U
#define STIM_TASK_STACK_WORDS 3072U
#define STIM_TASK_PRIORITY 8U

typedef enum {
    STIM_EVENT_GPIO_EDGE = 0,
    STIM_EVENT_DEBOUNCE_EXPIRED,
    STIM_EVENT_WAVEFORM_FAULT,
} stim_event_type_t;

typedef struct {
    stim_event_type_t type;
    uint32_t edge_generation;
} stim_event_t;

typedef struct {
    QueueHandle_t queue;
    StaticQueue_t queue_storage;
    uint8_t queue_items[STIM_EVENT_QUEUE_LENGTH * sizeof(stim_event_t)];
    esp_timer_handle_t debounce_timer;
    StaticTask_t task_storage;
    StackType_t task_stack[STIM_TASK_STACK_WORDS];
    TaskHandle_t task;
    volatile uint32_t isr_edge_generation;
    uint32_t processed_edge_generation;
    int64_t last_edge_time_us;
    int pending_level;
    bool committed_enabled;
} stim_controller_context_t;

static stim_controller_context_t s_controller;

static void IRAM_ATTR stim_gpio_isr(void *argument)
{
    (void)argument;
    const stim_event_t event = {
        .type = STIM_EVENT_GPIO_EDGE,
        .edge_generation = ++s_controller.isr_edge_generation,
    };
    BaseType_t task_woken = pdFALSE;
    xQueueSendFromISR(s_controller.queue, &event, &task_woken);
    if (task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void stim_debounce_timer_callback(void *argument)
{
    (void)argument;
    const stim_event_t event = {
        .type = STIM_EVENT_DEBOUNCE_EXPIRED,
        .edge_generation = s_controller.isr_edge_generation,
    };
    xQueueSend(s_controller.queue, &event, 0);
}

static bool IRAM_ATTR stim_waveform_fault_isr(void *user_data)
{
    (void)user_data;
    const stim_event_t event = {
        .type = STIM_EVENT_WAVEFORM_FAULT,
        .edge_generation = 0U,
    };
    BaseType_t task_woken = pdFALSE;
    xQueueSendFromISR(s_controller.queue, &event, &task_woken);
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
    stim_event_t event;
    while (xQueueReceive(s_controller.queue, &event, portMAX_DELAY) == pdTRUE) {
        if (event.type == STIM_EVENT_GPIO_EDGE) {
            s_controller.processed_edge_generation = event.edge_generation;
            s_controller.pending_level = gpio_get_level(STIM_ENABLE_GPIO);
            s_controller.last_edge_time_us = esp_timer_get_time();
            const esp_err_t result = stim_arm_debounce_timer(STIM_DEBOUNCE_US);
            if (result != ESP_OK) {
                ESP_LOGE(TAG, "Failed to arm debounce timer: %s",
                         esp_err_to_name(result));
            }
            continue;
        }

        if (event.type == STIM_EVENT_WAVEFORM_FAULT) {
            ESP_LOGE(TAG,
                     "Stimulus GDMA descriptor error; forcing CSb/MOSI high; "
                     "ADC/SD capture remains active");
            stim_waveform_enter_safe_state();
            continue;
        }

        const uint32_t latest_generation = s_controller.isr_edge_generation;
        const int64_t elapsed_us =
            esp_timer_get_time() - s_controller.last_edge_time_us;
        if (event.edge_generation != latest_generation ||
            s_controller.processed_edge_generation != latest_generation ||
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

esp_err_t stim_controller_init(void)
{
    s_controller.queue = xQueueCreateStatic(
        STIM_EVENT_QUEUE_LENGTH,
        sizeof(stim_event_t),
        s_controller.queue_items,
        &s_controller.queue_storage);
    if (s_controller.queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t result = stim_waveform_init(stim_waveform_fault_isr, NULL);
    if (result != ESP_OK) {
        stim_waveform_enter_safe_state();
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
        stim_waveform_enter_safe_state();
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
        stim_waveform_enter_safe_state();
        return result;
    }

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

    result = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        stim_waveform_enter_safe_state();
        return result;
    }
    result = gpio_isr_handler_add(STIM_ENABLE_GPIO, stim_gpio_isr, NULL);
    if (result != ESP_OK) {
        stim_waveform_enter_safe_state();
        return result;
    }

    s_controller.pending_level = gpio_get_level(STIM_ENABLE_GPIO);
    s_controller.committed_enabled = false;
    s_controller.isr_edge_generation = 1U;
    const stim_event_t initial_event = {
        .type = STIM_EVENT_GPIO_EDGE,
        .edge_generation = 1U,
    };
    xQueueSend(s_controller.queue, &initial_event, 0);
    ESP_LOGI(TAG,
             "GPIO%d stimulus enable ready: pull-down, %u us continuous-stable debounce",
             STIM_ENABLE_GPIO, STIM_DEBOUNCE_US);
    return ESP_OK;
}
