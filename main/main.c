#include "ble_service.h"
#include "emmc_storage_manager.h"
#include "uart_bridge.h"

#include "esp_err.h"

void app_main(void)
{
    ESP_ERROR_CHECK(emmc_storage_manager_init());
    ESP_ERROR_CHECK(ble_service_init());
    /* UART0 switches baud rate and suppresses logs, so initialize it last. */
    ESP_ERROR_CHECK(uart_bridge_init());

    uart_bridge_run();
}
