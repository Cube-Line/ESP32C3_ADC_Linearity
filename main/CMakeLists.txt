idf_component_register(
    SRCS "adc_linearity.c"
    INCLUDE_DIRS "."
    REQUIRES esp_adc esp_timer esp_wifi esp_netif nvs_flash lwip
)
