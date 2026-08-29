#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "stim_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t stim_controller_init(void);
esp_err_t stim_controller_configure(
    const stim_protocol_parameters_t *parameters);
esp_err_t stim_controller_request_enabled(bool enabled);

#ifdef __cplusplus
}
#endif
