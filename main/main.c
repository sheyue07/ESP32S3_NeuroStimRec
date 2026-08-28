#include "emmc_storage_manager.h"
#include "uart_bridge.h"
#include "write_switch.h"

#include "esp_err.h"

void app_main(void)
{
    /* GPIO first, then the control UART, then the sole eMMC owner. */
    ESP_ERROR_CHECK(write_switch_init());
    ESP_ERROR_CHECK(uart_bridge_init());
    ESP_ERROR_CHECK(emmc_storage_manager_init());
    ESP_ERROR_CHECK(write_switch_start_monitor());

    uart_bridge_run();
}
