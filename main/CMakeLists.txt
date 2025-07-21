idf_component_register(SRCS "main.cpp" "gy25t.cpp" "motor_control.cpp"
                    INCLUDE_DIRS "."
                    # 关键修正：添加主程序所依赖的所有 ESP-IDF 元件
                    # driver: 提供 uart.h 等硬件驱动
                    # log: 提供 ESP_LOGI 等日志功能
                    # esp_timer: 提供 esp_timer.h 时间功能
                    # nvs_flash: WiFi 初始化需要
                    # esp_netif: TCP/IP 网络接口
                    # esp_wifi: WiFi 驱动
                    # esp_event: 事件循环框架
                    # esp_http_server: HTTP 和 WebSocket 服务器
                    PRIV_REQUIRES driver log esp_timer nvs_flash esp_netif esp_wifi esp_event esp_http_server)