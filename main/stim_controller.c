#include "stim_controller.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stim_waveform.h"

static const char *TAG = "STIM_CONTROLLER";

#define STIM_TASK_STACK_WORDS 3072U
#define STIM_TASK_PRIORITY 8U

#define STIM_NOTIFY_WAVEFORM_FAULT (1U << 0)
#define STIM_NOTIFY_SEQUENCE_COMPLETE (1U << 1)
#define STIM_NOTIFY_STOP_ACTIVE (1U << 2)

typedef struct {
    StaticTask_t task_storage;
    StackType_t task_stack[STIM_TASK_STACK_WORDS];
    TaskHandle_t task;
    bool initialized;
} stim_controller_context_t;

static stim_controller_context_t s_controller;

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

static void stim_controller_task(void *argument)
{
    (void)argument;
    uint32_t notifications = 0U;
    for (;;) {
        if (xTaskNotifyWait(0U,
                            UINT32_MAX,
                            &notifications,
                            portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if ((notifications & STIM_NOTIFY_WAVEFORM_FAULT) != 0U) {
            stim_waveform_status_t status;
            stim_waveform_get_status(&status);
            ESP_LOGE(TAG,
                     "Stimulus pipeline fault: descriptor=%" PRIu32
                     ", fifo_underflow=%" PRIu32
                     ", unexpected_stop=%" PRIu32
                     "; output forced safe; ADC/eMMC remains active",
                     status.descriptor_errors,
                     status.fifo_underflow_errors,
                     status.unexpected_stop_errors);
        }
        if ((notifications & STIM_NOTIFY_SEQUENCE_COMPLETE) != 0U) {
            ESP_LOGI(TAG, "BLE-requested stimulus sequence complete: 10/10 frames");
        }
        if ((notifications & STIM_NOTIFY_STOP_ACTIVE) != 0U) {
            ESP_LOGI(TAG, "BLE-requested stimulus stop reached a 101-clock boundary");
        }
    }
}

esp_err_t stim_controller_init(void)
{
    if (s_controller.initialized) {
        return ESP_OK;
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
    const esp_err_t result = stim_waveform_init(stim_waveform_event_isr, NULL);
    if (result != ESP_OK) {
        TaskHandle_t task = s_controller.task;
        s_controller.task = NULL;
        vTaskDelete(task);
        stim_waveform_enter_safe_state();
        return result;
    }
    s_controller.initialized = true;
    ESP_LOGI(TAG,
             "Phone-only stimulation control ready; GPIO5 input control disabled");
    return ESP_OK;
}

esp_err_t stim_controller_configure(
    const stim_protocol_parameters_t *parameters)
{
    if (!s_controller.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    return stim_waveform_configure(parameters);
}

esp_err_t stim_controller_request_enabled(bool enabled)
{
    if (!s_controller.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    return stim_waveform_request_enabled(enabled);
}
