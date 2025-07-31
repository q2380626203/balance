#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "ble_imu.h"
#include "motor_control.h"

#ifdef __cplusplus
extern "C" {
#endif

// =====================================================================================
// --- 平衡控制配置参数 (Balance Control Configuration) ---
// =====================================================================================

typedef struct {
    float target_pitch_angle;    // 目标pitch角度 (度)
    float pitch_tolerance;       // pitch角度容差 (度)
    float motor_fixed_speed;     // 电机固定转动速度
    float enable_delay_ms;       // 启动延时 (毫秒)
    bool  auto_restart_enabled;  // 自动重启使能
    float restart_threshold;     // 重启阈值 (角度变化)
} balance_config_t;

// =====================================================================================
// --- 系统状态数据 (System Status Data) ---
// =====================================================================================

typedef struct {
    // BLE IMU状态
    bool ble_connected;
    uint32_t ble_bytes_received;
    ble_imu_data_t sensor_data;
    bool sensor_data_valid;
    
    // 电机状态
    bool motor_enabled;
    float current_motor_speed;
    bool motor_should_run;
    
    // 平衡控制状态
    float pitch_error;
    bool in_tolerance;
    uint32_t last_control_update_ms;
    
    // 系统统计
    uint32_t uptime_ms;
    uint32_t control_loop_count;
    float cpu_usage_percent;
} system_status_t;

// =====================================================================================
// --- 共享数据管理器 ---
// =====================================================================================

typedef struct {
    SemaphoreHandle_t mutex;
    balance_config_t config;
    system_status_t status;
    
    // 模块句柄
    motor_controller_t* motor_controller;
    ble_imu_handle_t* ble_imu_handle;
} shared_data_t;

// =====================================================================================
// --- 公共接口函数 ---
// =====================================================================================

// 初始化共享数据管理器
shared_data_t* shared_data_init(void);

// 销毁共享数据管理器
void shared_data_destroy(shared_data_t* shared_data);

// 配置参数操作（线程安全）
bool shared_data_get_config(shared_data_t* shared_data, balance_config_t* config);
bool shared_data_set_config(shared_data_t* shared_data, const balance_config_t* config);

// 系统状态操作（线程安全）
bool shared_data_get_status(shared_data_t* shared_data, system_status_t* status);
bool shared_data_update_status(shared_data_t* shared_data, const system_status_t* status);

// 模块句柄操作
void shared_data_set_motor_controller(shared_data_t* shared_data, motor_controller_t* controller);
void shared_data_set_ble_imu_handle(shared_data_t* shared_data, ble_imu_handle_t* handle);
motor_controller_t* shared_data_get_motor_controller(shared_data_t* shared_data);
ble_imu_handle_t* shared_data_get_ble_imu_handle(shared_data_t* shared_data);

// 默认配置
balance_config_t shared_data_get_default_config(void);

#ifdef __cplusplus
}
#endif