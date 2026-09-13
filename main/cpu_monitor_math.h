#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Strictly reject inconsistent windows; do not hide them by clamping. */
static inline bool cpu_monitor_busy(uint64_t previous, uint64_t current,
                                    uint64_t dt, uint16_t *busy)
{
    if (!dt || current < previous || current - previous > dt ||
        dt > UINT64_MAX / 1000U) return false;
    *busy = (uint16_t)(1000U - ((current - previous) * 1000U / dt));
    return true;
}
