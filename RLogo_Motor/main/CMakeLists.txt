idf_component_register(SRCS "motor_ctrl.cpp" "LED.cpp" "firmware.cpp" "tcp_protocol.cpp" "main.cpp" "PID.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES 
                        pr_common
        nvs_flash
        esp_event
        esp_wifi
        lwip
        app_update
        esp_http_client
        esp_https_ota
        driver
)
                    
