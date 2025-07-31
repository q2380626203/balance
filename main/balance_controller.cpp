#include "balance_controller.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdlib.h>

static const char* TAG = "BalanceController";

// =====================================================================================
// --- 平衡控制器句柄结构 ---
// =====================================================================================

struct balance_controller_handle {
    shared_data_t* shared_data;
    TaskHandle_t control_task_handle;
    TaskHandle_t display_task_handle;
    bool running;
    bool initialized;
};

// =====================================================================================
// --- 实时显示任务 ---
// =====================================================================================

static void realtime_display_task(void *pvParameters) {
    balance_controller_handle_t* handle = (balance_controller_handle_t*)pvParameters;
    const TickType_t refresh_rate = pdMS_TO_TICKS(500); // 2Hz刷新频率
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (handle->running) {
        vTaskDelayUntil(&last_wake_time, refresh_rate);
        
        // 获取系统状态
        system_status_t status;
        if (!shared_data_get_status(handle->shared_data, &status)) {
            continue;
        }
        
        printf("\\033[2J\\033[H"); // 清屏
        printf("=== ESP32 BLE IMU 平衡控制系统 ===\\n");
        printf("BLE: 接收%lu字节 | 连接状态: %s\\n", 
               status.ble_bytes_received,
               status.ble_connected ? "已连接" : "未连接");
        
        if (status.sensor_data_valid) {
            printf("Roll: %6.2f°  **PITCH: %6.2f°**  Yaw: %6.2f°\\n",
                   status.sensor_data.roll, status.sensor_data.pitch, status.sensor_data.yaw);
            printf("时间戳: %lu | 包#%lu\\n",
                   status.sensor_data.timestamp, status.sensor_data.packet_count);
            
            balance_config_t config;
            if (shared_data_get_config(handle->shared_data, &config)) {
                printf("平衡控制: PITCH=%.2f° (目标%.1f°±%.1f°) | 电机: %s\\n", 
                       status.sensor_data.pitch, config.target_pitch_angle, config.pitch_tolerance,
                       status.motor_enabled ? "运行" : "停止");
            }
        } else {
            printf("等待BLE IMU数据... (检查BLE连接状态)\\n");
        }
        
        fflush(stdout);
    }
    
    vTaskDelete(NULL);
}

// =====================================================================================
// --- 平衡控制任务 ---
// =====================================================================================

static void balance_control_task(void *pvParameters) {
    balance_controller_handle_t* handle = (balance_controller_handle_t*)pvParameters;
    
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 5ms, 200Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    // 控制状态变量
    float current_pitch = 0.0f;
    float pitch_error = 0.0f;
    bool motor_should_run = false;
    bool last_in_tolerance = true;
    TickType_t out_of_tolerance_start_time = 0;
    
    // PITCH角变化检测变量
    float pitch_at_check_start = 0.0f;
    TickType_t pitch_check_start_time = 0;
    TickType_t last_motor_restart_time = 0;
    TickType_t last_restart_velocity_time = 0;
    
    // 速度指令发送计时器
    TickType_t xLastVelocityTime = xTaskGetTickCount();
    const TickType_t xVelocityFrequency = pdMS_TO_TICKS(10); // 10ms发送一次
    
    while (handle->running) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // 获取当前配置和状态
        balance_config_t config;
        system_status_t status;
        
        if (!shared_data_get_config(handle->shared_data, &config) ||
            !shared_data_get_status(handle->shared_data, &status)) {
            continue;
        }
        
        // 获取模块句柄
        motor_controller_t* motor_controller = shared_data_get_motor_controller(handle->shared_data);
        ble_imu_handle_t* ble_imu_handle = shared_data_get_ble_imu_handle(handle->shared_data);
        
        if (!motor_controller || !ble_imu_handle) {
            continue;
        }
        
        // 1. 数据更新：通过BLE IMU模块获取pitch角度
        current_pitch = ble_imu_get_pitch(ble_imu_handle);
        
        // 初始化检测起始点
        if (pitch_check_start_time == 0) {
            pitch_check_start_time = xTaskGetTickCount();
            pitch_at_check_start = current_pitch;
        }
        
        // 2. 控制逻辑
        pitch_error = current_pitch - config.target_pitch_angle;
        bool in_tolerance = (fabs(pitch_error) <= config.pitch_tolerance);
        
        const TickType_t enable_delay = pdMS_TO_TICKS((uint32_t)config.enable_delay_ms);
        const float pitch_significant_change = config.restart_threshold;
        const TickType_t pitch_check_timeout = pdMS_TO_TICKS(500);
        const TickType_t motor_restart_cooldown = pdMS_TO_TICKS(1500);
        const TickType_t restart_velocity_period = pdMS_TO_TICKS(500);
        const TickType_t restart_velocity_freq = pdMS_TO_TICKS(10);
        
        if (in_tolerance) {
            motor_control_set_velocity(motor_controller, 0.0f);
            motor_should_run = false;
            last_in_tolerance = true;
        } else {
            if (last_in_tolerance) {
                out_of_tolerance_start_time = xTaskGetTickCount();
                pitch_check_start_time = xTaskGetTickCount();
                pitch_at_check_start = current_pitch;
                last_in_tolerance = false;
                ESP_LOGI(TAG, "PITCH离开平衡区(%.2f°)，开始计时", current_pitch);
            }
            
            TickType_t current_time = xTaskGetTickCount();
            
            // 检查是否需要重启电机
            if (config.auto_restart_enabled &&
                current_time - pitch_check_start_time >= pitch_check_timeout && 
                current_time - last_motor_restart_time >= motor_restart_cooldown) {
                
                float pitch_change_amount = fabs(current_pitch - pitch_at_check_start);
                if (pitch_change_amount <= pitch_significant_change) {
                    ESP_LOGW(TAG, "平衡外超过500ms且PITCH角变化仅%.2f°，重启电机", pitch_change_amount);
                    
                    motor_control_clear_errors(motor_controller);
                    
                    out_of_tolerance_start_time = current_time;
                    last_motor_restart_time = current_time;
                    last_restart_velocity_time = current_time;
                    
                    // 使能电机
                    for (int i = 0; i < 3; i++) {
                        motor_control_enable(motor_controller, true);
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }
                    
                    float motor_speed = (pitch_error > 0) ? config.motor_fixed_speed : -config.motor_fixed_speed;
                    motor_control_set_velocity(motor_controller, motor_speed);
                    motor_should_run = true;
                }
                
                pitch_check_start_time = current_time;
                pitch_at_check_start = current_pitch;
            } else if (current_time - out_of_tolerance_start_time >= enable_delay) {
                // 延时后正常使能
                if (!motor_control_is_enabled(motor_controller)) {
                    for (int i = 0; i < 3; i++) {
                        motor_control_enable(motor_controller, true);
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }
                }
                motor_should_run = true;
            } else {
                // 延时期间保持失能
                if (motor_control_is_enabled(motor_controller)) {
                    for (int i = 0; i < 3; i++) {
                        motor_control_enable(motor_controller, false);
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }
                    motor_control_set_velocity(motor_controller, 0.0f);
                }
                motor_should_run = false;
            }
        }
        
        // 3. 定期发送速度指令
        TickType_t now = xTaskGetTickCount();
        bool in_restart_period = (now - last_motor_restart_time <= restart_velocity_period);
        
        if (motor_should_run) {
            if (in_restart_period) {
                if (now - last_restart_velocity_time >= restart_velocity_freq) {
                    float motor_speed = (pitch_error > 0) ? config.motor_fixed_speed : -config.motor_fixed_speed;
                    motor_control_set_velocity(motor_controller, motor_speed);
                    last_restart_velocity_time = now;
                }
            } else {
                if (now - xLastVelocityTime >= xVelocityFrequency) {
                    float motor_speed = (pitch_error > 0) ? config.motor_fixed_speed : -config.motor_fixed_speed;
                    motor_control_set_velocity(motor_controller, motor_speed);
                    xLastVelocityTime = now;
                }
            }
        }
        
        // 4. 定期清理错误
        static TickType_t xLastClearErrorTime = 0;
        const TickType_t xClearErrorFrequency = pdMS_TO_TICKS(3000);
        if (xTaskGetTickCount() - xLastClearErrorTime >= xClearErrorFrequency) {
            motor_control_clear_errors(motor_controller);
            xLastClearErrorTime = xTaskGetTickCount();
        }
        
        // 5. 更新系统状态
        status.sensor_data_valid = ble_imu_get_data(ble_imu_handle, &status.sensor_data);
        status.ble_connected = ble_imu_is_connected(ble_imu_handle);
        status.ble_bytes_received = ble_imu_get_bytes_received(ble_imu_handle);
        status.motor_enabled = motor_control_is_enabled(motor_controller);
        status.current_motor_speed = motor_should_run ? 
            ((pitch_error > 0) ? config.motor_fixed_speed : -config.motor_fixed_speed) : 0.0f;
        status.motor_should_run = motor_should_run;
        status.pitch_error = pitch_error;
        status.in_tolerance = in_tolerance;
        status.last_control_update_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        status.control_loop_count++;
        
        shared_data_update_status(handle->shared_data, &status);
    }
    
    vTaskDelete(NULL);
}

// =====================================================================================
// --- 公共接口实现 ---
// =====================================================================================

balance_controller_handle_t* balance_controller_init(shared_data_t* shared_data) {
    if (!shared_data) {
        ESP_LOGE(TAG, "Shared data is required");
        return NULL;
    }
    
    balance_controller_handle_t* handle = (balance_controller_handle_t*)malloc(sizeof(balance_controller_handle_t));
    if (!handle) {
        ESP_LOGE(TAG, "Failed to allocate memory for balance controller handle");
        return NULL;
    }
    
    memset(handle, 0, sizeof(balance_controller_handle_t));
    handle->shared_data = shared_data;
    handle->running = false;
    handle->initialized = true;
    
    ESP_LOGI(TAG, "Balance controller initialized successfully");
    return handle;
}

bool balance_controller_start(balance_controller_handle_t* handle) {
    if (!handle || !handle->initialized) {
        ESP_LOGE(TAG, "Balance controller handle not initialized");
        return false;
    }
    
    if (handle->running) {
        ESP_LOGW(TAG, "Balance controller already running");
        return true;
    }
    
    handle->running = true;
    
    // 创建显示任务
    BaseType_t ret = xTaskCreate(realtime_display_task, "realtime_display", 4096, handle, 3, &handle->display_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create display task");
        handle->running = false;
        return false;
    }
    
    // 创建控制任务
    ret = xTaskCreate(balance_control_task, "balance_control", 4096, handle, 5, &handle->control_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create control task");
        handle->running = false;
        if (handle->display_task_handle) {
            vTaskDelete(handle->display_task_handle);
            handle->display_task_handle = NULL;
        }
        return false;
    }
    
    ESP_LOGI(TAG, "Balance controller started successfully");
    return true;
}

bool balance_controller_stop(balance_controller_handle_t* handle) {
    if (!handle || !handle->initialized) {
        ESP_LOGE(TAG, "Balance controller handle not initialized");
        return false;
    }
    
    if (!handle->running) {
        ESP_LOGW(TAG, "Balance controller already stopped");
        return true;
    }
    
    handle->running = false;
    
    // 等待任务结束
    if (handle->control_task_handle) {
        vTaskDelete(handle->control_task_handle);
        handle->control_task_handle = NULL;
    }
    
    if (handle->display_task_handle) {
        vTaskDelete(handle->display_task_handle);
        handle->display_task_handle = NULL;
    }
    
    ESP_LOGI(TAG, "Balance controller stopped");
    return true;
}

bool balance_controller_is_running(balance_controller_handle_t* handle) {
    if (!handle) {
        return false;
    }
    
    return handle->running;
}

void balance_controller_destroy(balance_controller_handle_t* handle) {
    if (!handle) return;
    
    if (handle->running) {
        balance_controller_stop(handle);
    }
    
    free(handle);
    ESP_LOGI(TAG, "Balance controller destroyed");
}