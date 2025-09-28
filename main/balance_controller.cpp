#include "balance_controller.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static const char* TAG = "BalanceController";

// =====================================================================================
// --- PID控制器实现 ---
// =====================================================================================

void pid_init(pid_controller_t* pid, float kp, float ki, float kd, float output_min, float output_max) {
    if (!pid) return;
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = output_min;
    pid->output_max = output_max;
    
    // 设置积分限幅，防止积分饱和
    if (fabs(ki) > 0.001f) {
        pid->integral_min = -output_max / ki;
        pid->integral_max = output_max / ki;
    } else {
        pid->integral_min = -1000.0f;
        pid->integral_max = 1000.0f;
    }
}

float pid_update(pid_controller_t* pid, float error, float dt) {
    if (!pid || dt <= 0.0f) {
        return 0.0f;
    }
    
    // 比例项
    float proportional = pid->kp * error;
    
    // 积分项（带限幅防饱和）
    pid->integral += error * dt;
    
    // 积分限幅
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < pid->integral_min) {
        pid->integral = pid->integral_min;
    }
    
    float integral_term = pid->ki * pid->integral;
    
    // 微分项
    float derivative = (error - pid->prev_error) / dt;
    float derivative_term = pid->kd * derivative;
    
    // 计算PID输出
    float output = proportional + integral_term + derivative_term;
    
    // 输出限幅
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }
    
    // 保存当前误差用于下一次微分计算
    pid->prev_error = error;
    
    return output;
}

void pid_reset(pid_controller_t* pid) {
    if (pid) {
        pid->integral = 0.0f;
        pid->prev_error = 0.0f;
    }
}

// =====================================================================================
// --- 平衡控制器句柄结构 ---
// =====================================================================================

struct balance_controller_handle {
    shared_data_t* shared_data;
    TaskHandle_t display_task_handle;
    TaskHandle_t sensor_task_handle;  // 新增传感器数据处理任务
    bool running;
    bool initialized;
    pid_controller_t pitch_pid; // 添加PID控制器成员
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
// --- 平衡控制任务（使用PID控制，优化版）---
// =====================================================================================

static void balance_control_task(void *pvParameters) {
    balance_controller_handle_t* handle = (balance_controller_handle_t*)pvParameters;
    
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz控制频率
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    // 控制状态变量
    float current_pitch = 0.0f;
    float prev_pitch = 0.0f;
    float pitch_error = 0.0f;
    float pitch_derivative = 0.0f;
    bool motor_should_run = false;
    bool last_in_tolerance = true;
    TickType_t out_of_tolerance_start_time = 0;
    
    // 时间跟踪变量
    TickType_t last_control_time = xTaskGetTickCount();
    
    // 初始化PID控制器
    pid_controller_t* pid = &handle->pitch_pid;
    
    // 保守的PID参数（减小抖动）
    float kp = 1.8f;
    float ki = 0.05f;
    float kd = 0.5f;
    float min_output = -30.0f;
    float max_output = 30.0f;
    
    pid_init(pid, kp, ki, kd, min_output, max_output);
    
    // 滤波和平滑变量
    float filtered_pitch = 0.0f;
    const float alpha = 0.3f;
    float smoothed_speed = 0.0f;
    const float smoothing_factor = 0.2f;
    
    ESP_LOGI(TAG, "PID控制启动: kp=%.2f, ki=%.2f, kd=%.2f", kp, ki, kd);
    
    while (handle->running) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // 计算时间间隔
        TickType_t current_time = xTaskGetTickCount();
        float dt = (current_time - last_control_time) * portTICK_PERIOD_MS / 1000.0f;
        if (dt <= 0) dt = 0.01f;
        last_control_time = current_time;
        
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
        
        // 1. 数据更新和滤波
        current_pitch = ble_imu_get_pitch(ble_imu_handle);
        filtered_pitch = alpha * current_pitch + (1 - alpha) * filtered_pitch;
        
        // 计算角度变化率
        if (dt > 0) {
            pitch_derivative = (filtered_pitch - prev_pitch) / dt;
        }
        prev_pitch = filtered_pitch;
        
        // 2. 控制逻辑
        pitch_error = filtered_pitch - config.target_pitch_angle;
        bool in_tolerance = (fabs(pitch_error) <= config.pitch_tolerance);
        
        const TickType_t enable_delay = pdMS_TO_TICKS((uint32_t)config.enable_delay_ms);
        
        if (in_tolerance) {
            // 平衡区内停止电机
            smoothed_speed = smoothing_factor * 0.0f + (1 - smoothing_factor) * smoothed_speed;
            motor_control_set_velocity(motor_controller, smoothed_speed);
            
            motor_should_run = false;
            last_in_tolerance = true;
            pid_reset(pid);
            
            // 速度接近零时停止电机
            if (fabs(smoothed_speed) < 1.0f) {
                if (motor_control_is_enabled(motor_controller)) {
                    motor_control_enable(motor_controller, false);
                }
            }
        } else {
            if (last_in_tolerance) {
                out_of_tolerance_start_time = current_time;
                last_in_tolerance = false;
                pid_reset(pid);
                ESP_LOGI(TAG, "PITCH离开平衡区(%.2f°→%.2f°)", config.target_pitch_angle, filtered_pitch);
            }
            
            // 延时后使能电机
            if (current_time - out_of_tolerance_start_time >= enable_delay) {
                if (!motor_control_is_enabled(motor_controller)) {
                    motor_control_enable(motor_controller, true);
                    vTaskDelay(pdMS_TO_TICKS(5));
                }
                motor_should_run = true;
            } else {
                // 延时期间保持失能
                if (motor_control_is_enabled(motor_controller)) {
                    motor_control_enable(motor_controller, false);
                    motor_control_set_velocity(motor_controller, 0.0f);
                }
                motor_should_run = false;
                smoothed_speed = 0.0f;
            }
        }
        
        // 3. PID速度计算
        float motor_speed = 0.0f;
        if (motor_should_run) {
            motor_speed = pid_update(pid, pitch_error, dt);
            
            // 动态增益调整
            float derivative_gain = 1.0f;
            if (fabs(pitch_derivative) > 10.0f) {
                derivative_gain = 0.7f;
            }
            motor_speed *= derivative_gain;
            
            // 死区补偿
            float deadzone_threshold = config.pitch_tolerance;
            if (fabs(pitch_error) < deadzone_threshold) {
                float ratio = fabs(pitch_error) / deadzone_threshold;
                motor_speed *= (0.2f + 0.8f * ratio);
            }
            
            // 限制最大速度
            float speed_limit = config.motor_fixed_speed;
            if (motor_speed > speed_limit) motor_speed = speed_limit;
            if (motor_speed < -speed_limit) motor_speed = -speed_limit;
            
            // 输出平滑
            smoothed_speed = smoothing_factor * motor_speed + (1 - smoothing_factor) * smoothed_speed;
            if (fabs(smoothed_speed) < 0.5f) smoothed_speed = 0.0f;
            
            motor_control_set_velocity(motor_controller, smoothed_speed);
        }
        
        // 4. 错误清理
        static TickType_t xLastClearErrorTime = 0;
        const TickType_t xClearErrorFrequency = pdMS_TO_TICKS(10000);
        if (motor_should_run && (current_time - xLastClearErrorTime >= xClearErrorFrequency)) {
            motor_control_clear_errors(motor_controller);
            xLastClearErrorTime = current_time;
        }
        
        // 5. 更新系统状态
        status.sensor_data_valid = ble_imu_get_data(ble_imu_handle, &status.sensor_data);
        status.ble_connected = ble_imu_is_connected(ble_imu_handle);
        status.ble_bytes_received = ble_imu_get_bytes_received(ble_imu_handle);
        status.motor_enabled = motor_control_is_enabled(motor_controller);
        status.current_motor_speed = smoothed_speed;
        status.motor_should_run = motor_should_run;
        status.pitch_error = pitch_error;
        status.in_tolerance = in_tolerance;
        status.last_control_update_ms = current_time * portTICK_PERIOD_MS;
        status.control_loop_count++;
        
        // 添加新状态信息（如果system_status_t已更新）
        // status.pid_output = motor_speed;
        // status.filtered_pitch = filtered_pitch;
        // status.pitch_derivative = pitch_derivative;
        
        shared_data_update_status(handle->shared_data, &status);
    }
    
    // 退出前停止电机
    motor_controller_t* motor_controller = shared_data_get_motor_controller(handle->shared_data);
    if (motor_controller) {
        motor_control_set_velocity(motor_controller, 0.0f);
        motor_control_enable(motor_controller, false);
    }
    
    vTaskDelete(NULL);
}

// =====================================================================================
// --- 传感器数据处理任务（仅更新状态，不控制电机）---
// =====================================================================================

static void sensor_data_task(void *pvParameters) {
    balance_controller_handle_t* handle = (balance_controller_handle_t*)pvParameters;
    
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 100ms, 10Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
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
        ble_imu_handle_t* ble_imu_handle = shared_data_get_ble_imu_handle(handle->shared_data);
        
        if (!ble_imu_handle) {
            continue;
        }
        
        // 1. 数据更新：通过BLE IMU模块获取传感器数据
        status.sensor_data_valid = ble_imu_get_data(ble_imu_handle, &status.sensor_data);
        status.ble_connected = ble_imu_is_connected(ble_imu_handle);
        status.ble_bytes_received = ble_imu_get_bytes_received(ble_imu_handle);
        
        // 2. 计算平衡状态信息（仅用于显示，不控制电机）
        if (status.sensor_data_valid) {
            float pitch_error = status.sensor_data.pitch - config.target_pitch_angle;
            bool in_tolerance = fabs(pitch_error) <= config.pitch_tolerance;
            
            status.pitch_error = pitch_error;
            status.in_tolerance = in_tolerance;
            
            // 记录控制循环计数（即使没有实际控制）
            status.control_loop_count++;
        }
        
        // 3. 电机状态设置为禁用
        status.motor_enabled = false;
        status.motor_should_run = false;
        status.current_motor_speed = 0.0f;
        status.last_control_update_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // 4. 更新系统状态
        shared_data_update_status(handle->shared_data, &status);
        
        // 5. 可选：记录传感器数据（用于调试）
        static TickType_t last_log_time = 0;
        const TickType_t log_interval = pdMS_TO_TICKS(5000); // 5秒记录一次
        TickType_t current_time = xTaskGetTickCount();
        
        if (current_time - last_log_time >= log_interval) {
            if (status.sensor_data_valid) {
                ESP_LOGI(TAG, "传感器数据 - Roll:%.2f Pitch:%.2f Yaw:%.2f", 
                        status.sensor_data.roll, status.sensor_data.pitch, status.sensor_data.yaw);
            }
            last_log_time = current_time;
        }
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
    
    // 初始化PID控制器
    memset(&handle->pitch_pid, 0, sizeof(pid_controller_t));
    
    ESP_LOGI(TAG, "Balance controller with PID initialized successfully");
    return handle;
}

extern "C" {

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
    ret = xTaskCreate(balance_control_task, "balance_control", 4096, handle, 5, &handle->sensor_task_handle);
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
    if (handle->sensor_task_handle) {
        vTaskDelete(handle->sensor_task_handle);
        handle->sensor_task_handle = NULL;
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

} // extern "C"
