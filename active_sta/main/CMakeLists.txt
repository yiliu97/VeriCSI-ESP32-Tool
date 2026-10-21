idf_component_register(
        SRCS main.cc
        INCLUDE_DIRS ""
        PRIV_REQUIRES esp_driver_tsens esp_wifi esp_timer esp_event esp_netif esp_system
                      nvs_flash lwip esp_http_server esp_http_client driver
                      esp_driver_gptimer esp_driver_usb_serial_jtag esp_driver_sdspi fatfs sdmmc vfs esp_hw_support)
