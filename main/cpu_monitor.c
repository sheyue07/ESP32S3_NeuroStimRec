#include "cpu_monitor.h"
#include "cpu_monitor_math.h"
#include "sdkconfig.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static portMUX_TYPE snapshot_lock = portMUX_INITIALIZER_UNLOCKED;
static cpu_monitor_snapshot_t latest;

const char *cpu_monitor_state_name(cpu_monitor_state_t state)
{
    static const char *const names[] = {"NOT_READY", "OK", "DISABLED",
        "START_FAILED", "BAD_COUNTER", "BAD_WINDOW", "OUT_OF_RANGE", "STALE"};
    return (unsigned)state < sizeof(names) / sizeof(names[0]) ? names[state] : "UNKNOWN";
}

#if CONFIG_NEURO_CPU_MONITOR
_Static_assert(sizeof(configRUN_TIME_COUNTER_TYPE) == 8, "CPU monitor requires U64 runtime stats");
#if !CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER
#error CPU monitor requires ESP TIMER runtime clock
#endif
static bool started;

static void monitor_task(void *arg)
{
    (void)arg;
    TaskHandle_t idle[2] = {xTaskGetIdleTaskHandleForCore(0),
                           xTaskGetIdleTaskHandleForCore(1)};
    uint64_t previous[2] = {0}, previous_time = 0;
    bool baseline = false;
    cpu_monitor_snapshot_t result = {0};
    for (;;) {
        uint64_t counters[2];
        const uint64_t begin = esp_timer_get_time();
        /* Permanent idle tasks cannot disappear. No full task enumeration or
         * stack high-water scan under the kernel lock. Runtime is accounted
         * at context switches, not an independent ISR-time measurement. */
        for (unsigned core = 0; core < 2; ++core) {
            TaskStatus_t info;
            vTaskGetInfo(idle[core], &info, pdFALSE, eReady);
            counters[core] = info.ulRunTimeCounter;
        }
        const uint64_t end = esp_timer_get_time();
        const uint64_t midpoint = begin + (end - begin) / 2;
        ++result.sequence;
        result.sampled_us = midpoint;
        result.snapshot_us = end - begin;
        result.window_us = baseline && midpoint > previous_time ? midpoint - previous_time : 0;
        result.busy_permille[0] = result.busy_permille[1] = 0;
        result.state = CPU_MONITOR_NOT_READY;
        if (baseline) {
            result.state = CPU_MONITOR_OK;
            if (!result.window_us) result.state = CPU_MONITOR_BAD_WINDOW;
            else for (unsigned core = 0; core < 2; ++core) {
                if (counters[core] < previous[core]) {
                    result.state = CPU_MONITOR_BAD_COUNTER;
                    break;
                }
                if (!cpu_monitor_busy(previous[core], counters[core],
                                      result.window_us, &result.busy_permille[core])) {
                    result.state = CPU_MONITOR_OUT_OF_RANGE;
                    break;
                }
            }
            if (result.state != CPU_MONITOR_OK) ++result.invalid_windows;
        }
        /* Every sample rebuilds the baseline, including invalid windows. */
        previous[0] = counters[0]; previous[1] = counters[1];
        previous_time = midpoint;
        baseline = true;
        portENTER_CRITICAL(&snapshot_lock);
        latest = result;
        portEXIT_CRITICAL(&snapshot_lock);
        /* Relative delay avoids bursts of catch-up snapshots after starvation. */
        vTaskDelay(pdMS_TO_TICKS(CONFIG_NEURO_CPU_MONITOR_PERIOD_MS));
    }
}
#endif

esp_err_t cpu_monitor_start(void)
{
#if CONFIG_NEURO_CPU_MONITOR
    if (started) return ESP_OK;
    if (!xTaskGetIdleTaskHandleForCore(0) || !xTaskGetIdleTaskHandleForCore(1)) {
        portENTER_CRITICAL(&snapshot_lock);
        latest.state = CPU_MONITOR_START_FAILED;
        latest.error = ESP_ERR_INVALID_STATE;
        portEXIT_CRITICAL(&snapshot_lock);
        return ESP_ERR_INVALID_STATE;
    }
    if (xTaskCreatePinnedToCore(monitor_task, "CPU_MONITOR", 4096, NULL, 1,
                                NULL, 1) != pdPASS) {
        portENTER_CRITICAL(&snapshot_lock);
        latest.state = CPU_MONITOR_START_FAILED;
        latest.error = ESP_ERR_NO_MEM;
        portEXIT_CRITICAL(&snapshot_lock);
        return ESP_ERR_NO_MEM;
    }
    started = true;
    return ESP_OK;
#else
    latest.state = CPU_MONITOR_DISABLED;
    return ESP_OK;
#endif
}

bool cpu_monitor_get_snapshot(cpu_monitor_snapshot_t *out)
{
    if (!out) return false;
    portENTER_CRITICAL(&snapshot_lock);
    *out = latest;
    portEXIT_CRITICAL(&snapshot_lock);
    const uint64_t now = esp_timer_get_time();
    out->age_ms = out->sampled_us && now >= out->sampled_us ? (now - out->sampled_us) / 1000 : 0;
#if CONFIG_NEURO_CPU_MONITOR
    if (out->state == CPU_MONITOR_OK && out->age_ms > 3U * CONFIG_NEURO_CPU_MONITOR_PERIOD_MS)
        out->state = CPU_MONITOR_STALE;
#else
    out->state = CPU_MONITOR_DISABLED;
#endif
    return out->state == CPU_MONITOR_OK;
}
