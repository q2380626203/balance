idf_component_register(SRCS "main.cpp" 
                           "motor_control.cpp" 
                           "ble_imu.c"
                           "shared_data.cpp"
                           "wifi_ap.cpp"
                           "web_server.cpp"
                           "balance_controller.cpp"
                    INCLUDE_DIRS "."
                    # ESP32 平衡车双协议通信系统依赖组件
                    # driver: UART, GPIO等硬件驱动
                    # esp_timer: 看门狗功能
                    # log: ESP_LOG日志功能  
                    # nvs_flash: BLE和WiFi初始化需要
                    # bt: 蓝牙基础组件(NimBLE)
                    # esp_wifi: WiFi AP功能
                    # esp_netif: 网络接口
                    # esp_event: 事件循环
                    # esp_http_server: HTTP服务器
                    # json: cJSON库用于API数据交换
                    PRIV_REQUIRES driver esp_timer log nvs_flash bt esp_wifi esp_netif esp_event esp_http_server json)