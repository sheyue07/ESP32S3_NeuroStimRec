#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t uart_bridge_init(void);
void uart_bridge_run(void) __attribute__((noreturn));

#ifdef __cplusplus
}
#endif
