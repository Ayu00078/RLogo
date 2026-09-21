idf_component_register(
    SRCS "app_main.cpp" "controller.cpp"
    INCLUDE_DIRS "."
    REQUIRES pr_common freertos nvs_flash esp_wifi esp_netif lwip log)
