idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_wifi esp_event esp_netif esp_timer
                                esp_driver_usb_serial_jtag esp_driver_uart mbedtls)
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
