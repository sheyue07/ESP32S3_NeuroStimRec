#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t uart_bridge_init(void);
void uart_bridge_run(void) __attribute__((noreturn));
/* Bounded RAM history, never writes UART from the caller's task. */
void uart_bridge_stim_log(const char *format, ...)
    __attribute__((format(printf, 1, 2)));

#ifdef __cplusplus
}
#endif
