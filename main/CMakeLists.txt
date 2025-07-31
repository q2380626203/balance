idf_component_register(SRCS "main.cpp" "motor_control.cpp" "ble_imu.c"
                    INCLUDE_DIRS "."
                    # BLE IMU 平衡车系统依赖的组件
                    # driver: 提供 uart.h, gpio.h 等硬件驱动
                    # esp_timer: 提供 esp_task_wdt.h 看门狗功能
                    # log: 提供 ESP_LOG* 日志功能
                    # nvs_flash: BLE初始化需要
                    # bt: 蓝牙基础组件(包含NimBLE)
                    PRIV_REQUIRES driver esp_timer log nvs_flash bt)