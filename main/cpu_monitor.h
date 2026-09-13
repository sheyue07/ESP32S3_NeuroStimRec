#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    CPU_MONITOR_NOT_READY, CPU_MONITOR_OK, CPU_MONITOR_DISABLED,
    CPU_MONITOR_START_FAILED, CPU_MONITOR_BAD_COUNTER,
    CPU_MONITOR_BAD_WINDOW, CPU_MONITOR_OUT_OF_RANGE, CPU_MONITOR_STALE
} cpu_monitor_state_t;

typedef struct {
    uint64_t sequence, sampled_us, window_us, snapshot_us, age_ms;
    uint32_t invalid_windows;
    uint16_t busy_permille[2];
    cpu_monitor_state_t state;
    esp_err_t error;
} cpu_monitor_snapshot_t;

/* Optional diagnosis; never prints to UART or changes acquisition state. */
esp_err_t cpu_monitor_start(void);
bool cpu_monitor_get_snapshot(cpu_monitor_snapshot_t *out);
const char *cpu_monitor_state_name(cpu_monitor_state_t state);
