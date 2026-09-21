idf_component_register(
    SRCS
        # ===== 灯板测试模式（启用下面两行，注释掉其余两组）=====
        # "test_led_arrow.cpp"
        # "led_controller.cpp"

        # ===== 传感器阈值调试模式（启用下面四行，注释掉其余两组）=====
        # 调参入口在 test_sensor_threshold.cpp 顶部的 PARAM_* 宏
        # "test_sensor_threshold.cpp"
        # "uart_handler.cpp"
        # "firmware.cpp"
        # "led_controller.cpp"

        # ===== 正式固件（测试完成后恢复：取消注释下面各行，注释掉上面两组）=====
        "app_main.cpp"
        "armour_app.cpp"
        "hit_reporter.cpp"
        "hit_filter.cpp"
        "sensor_processor.cpp"
        "uart_handler.cpp"
        "led_controller.cpp"
        "tcp_protocol.cpp"
        "firmware.cpp"
        "ble_sensor_monitor.cpp"
    INCLUDE_DIRS  "."
    REQUIRES
        pr_common
        nvs_flash
        esp_event
        esp_wifi
        esp_timer
        lwip
        app_update
        esp_http_client
        esp_https_ota
        driver
        espressif__led_strip
        bt
)
