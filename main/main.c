#include "ble_service.h"
#include "emmc_storage_manager.h"
#include "uart_bridge.h"

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "neurostimrec";

static void log_internal_memory(const char *stage)
{
    ESP_LOGI(TAG,
             "Internal RAM after %s: free=%u bytes, largest=%u bytes",
             stage,
             (unsigned int)heap_caps_get_free_size(MALLOC_CAP_INTERNAL |
                                                   MALLOC_CAP_8BIT),
             (unsigned int)heap_caps_get_largest_free_block(
                 MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
}

void app_main(void)
{
    ESP_ERROR_CHECK(emmc_storage_manager_init());
    log_internal_memory("storage init");
    ESP_ERROR_CHECK(ble_service_init());
    log_internal_memory("BLE init");
    /* UART0 switches baud rate and suppresses logs, so initialize it last. */
    ESP_ERROR_CHECK(uart_bridge_init());

    uart_bridge_run();
}
