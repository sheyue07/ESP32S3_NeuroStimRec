idf_component_register(
    SRCS "uart_bridge.c" "main.c" "emmc_storage_manager.c" "stim_waveform.c" "stim_controller.c" "stim_waveform_builder.c" "stim_protocol.c" "raw_sd_segment_format.c" "raw_sd_segment_recorder.c" "ble_protocol.c" "adc_preview.c" "ble_service.c"
         "frame_sync.c" "continuous_rx.c"
    INCLUDE_DIRS "."
    REQUIRES
        sdmmc
        esp_driver_sdmmc
        esp_driver_spi
        esp_driver_gpio
        esp_driver_uart
        esp_driver_dma
        esp_hal_gpspi
        esp_hal_lcd
        esp_hw_support
        esp_timer
        esp_ringbuf
        bt
        nvs_flash
)
