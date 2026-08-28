#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t write_switch_init(void);
esp_err_t write_switch_start_monitor(void);
bool write_switch_is_high(void);
bool write_switch_is_armed(void);

#ifdef __cplusplus
}
#endif
