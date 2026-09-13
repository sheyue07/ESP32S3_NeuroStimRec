#include "ble_service.h"
#include "cpu_monitor.h"
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

/* 启动入口：存储及采集资源 -> BLE -> UART -> 可选CPU诊断。
 * UART0会切换波特率并关闭普通日志，因此保留其最后初始化的顺序。
 * 后续数据处理由FreeRTOS任务负责，本任务运行串口命令循环。 */
void app_main(void)
{
    ESP_ERROR_CHECK(emmc_storage_manager_init());
    log_internal_memory("storage init");
    ESP_ERROR_CHECK(ble_service_init());
    log_internal_memory("BLE init");
    /* UART0 switches baud rate and suppresses logs, so initialize it last. */
    ESP_ERROR_CHECK(uart_bridge_init());
    /* Optional diagnosis must never prevent acquisition startup. */
    (void)cpu_monitor_start();

    uart_bridge_run();
}
