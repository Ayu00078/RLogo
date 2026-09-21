idf_component_register(SRCS "controller_gateway.cpp" "mech_engine.cpp" "hit_filter.cpp" "device_registry.cpp" "pr_cmd_events.cpp" "pr_app.cpp" "app_main.cpp" "LED_Strip.cpp" "LED.cpp" "firmware.cpp" "tcp_protocol.cpp" "comm_scheduler.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES 
                        pr_common
                        freertos
                        nvs_flash
                        esp_wifi
                        lwip
                        app_update
                        driver
                        log
                        esp_http_client
                        esp_https_ota
                        cxx
                        pthread)
