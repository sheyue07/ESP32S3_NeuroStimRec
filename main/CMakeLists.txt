idf_component_register(
    SRCS "raw_sd_segment_format.c" "raw_sd_segment_recorder.c"
         "main.c" "frame_sync.c" "continuous_rx.c"
    INCLUDE_DIRS "."
    REQUIRES
        sdmmc
        esp_driver_sdmmc
        esp_driver_spi
        esp_driver_gpio
        esp_driver_dma
        esp_hal_gpspi
        esp_timer
        esp_ringbuf
)
