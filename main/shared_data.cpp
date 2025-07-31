#include "shared_data.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"

static const char* TAG = "SharedData";

// =====================================================================================
// --- 默认配置 ---
// =====================================================================================

balance_config_t shared_data_get_default_config(void) {
    balance_config_t config = {
        .target_pitch_angle = 0.0f,     // 目标pitch角度
        .pitch_tolerance = 10.0f,       // pitch角度容差
        .motor_fixed_speed = 30.0f,     // 电机固定速度
        .enable_delay_ms = 500.0f,      // 启动延时500ms
        .auto_restart_enabled = true,   // 启用自动重启
        .restart_threshold = 1.0f       // 重启阈值1度
    };
    return config;
}

// =====================================================================================
// --- 共享数据管理器实现 ---
// =====================================================================================

shared_data_t* shared_data_init(void) {
    shared_data_t* shared_data = (shared_data_t*)malloc(sizeof(shared_data_t));
    if (!shared_data) {
        ESP_LOGE(TAG, "Failed to allocate memory for shared data");
        return NULL;
    }
    
    // 创建互斥锁
    shared_data->mutex = xSemaphoreCreateMutex();
    if (!shared_data->mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        free(shared_data);
        return NULL;
    }
    
    // 初始化配置为默认值
    shared_data->config = shared_data_get_default_config();
    
    // 初始化状态
    memset(&shared_data->status, 0, sizeof(system_status_t));
    
    // 初始化模块句柄
    shared_data->motor_controller = NULL;
    shared_data->ble_imu_handle = NULL;
    
    ESP_LOGI(TAG, "Shared data initialized successfully");
    return shared_data;
}

void shared_data_destroy(shared_data_t* shared_data) {
    if (!shared_data) return;
    
    if (shared_data->mutex) {
        vSemaphoreDelete(shared_data->mutex);
    }
    
    free(shared_data);
    ESP_LOGI(TAG, "Shared data destroyed");
}

// =====================================================================================
// --- 配置参数操作 ---
// =====================================================================================

bool shared_data_get_config(shared_data_t* shared_data, balance_config_t* config) {
    if (!shared_data || !config) return false;
    
    if (xSemaphoreTake(shared_data->mutex, pdMS_TO_TICKS(100)) == pdPASS) {
        *config = shared_data->config;
        xSemaphoreGive(shared_data->mutex);
        return true;
    }
    
    ESP_LOGW(TAG, "Failed to acquire mutex for config read");
    return false;
}

bool shared_data_set_config(shared_data_t* shared_data, const balance_config_t* config) {
    if (!shared_data || !config) return false;
    
    if (xSemaphoreTake(shared_data->mutex, pdMS_TO_TICKS(100)) == pdPASS) {
        shared_data->config = *config;
        xSemaphoreGive(shared_data->mutex);
        ESP_LOGI(TAG, "Configuration updated: pitch_target=%.1f, tolerance=%.1f, speed=%.1f", 
                 config->target_pitch_angle, config->pitch_tolerance, config->motor_fixed_speed);
        return true;
    }
    
    ESP_LOGW(TAG, "Failed to acquire mutex for config write");
    return false;
}

// =====================================================================================
// --- 系统状态操作 ---
// =====================================================================================

bool shared_data_get_status(shared_data_t* shared_data, system_status_t* status) {
    if (!shared_data || !status) return false;
    
    if (xSemaphoreTake(shared_data->mutex, pdMS_TO_TICKS(10)) == pdPASS) {
        *status = shared_data->status;
        xSemaphoreGive(shared_data->mutex);
        return true;
    }
    
    return false;
}

bool shared_data_update_status(shared_data_t* shared_data, const system_status_t* status) {
    if (!shared_data || !status) return false;
    
    if (xSemaphoreTake(shared_data->mutex, pdMS_TO_TICKS(10)) == pdPASS) {
        shared_data->status = *status;
        xSemaphoreGive(shared_data->mutex);
        return true;
    }
    
    return false;
}

// =====================================================================================
// --- 模块句柄操作 ---
// =====================================================================================

void shared_data_set_motor_controller(shared_data_t* shared_data, motor_controller_t* controller) {
    if (!shared_data) return;
    
    if (xSemaphoreTake(shared_data->mutex, pdMS_TO_TICKS(100)) == pdPASS) {
        shared_data->motor_controller = controller;
        xSemaphoreGive(shared_data->mutex);
    }
}

void shared_data_set_ble_imu_handle(shared_data_t* shared_data, ble_imu_handle_t* handle) {
    if (!shared_data) return;
    
    if (xSemaphoreTake(shared_data->mutex, pdMS_TO_TICKS(100)) == pdPASS) {
        shared_data->ble_imu_handle = handle;
        xSemaphoreGive(shared_data->mutex);
    }
}

motor_controller_t* shared_data_get_motor_controller(shared_data_t* shared_data) {
    if (!shared_data) return NULL;
    
    motor_controller_t* controller = NULL;
    if (xSemaphoreTake(shared_data->mutex, pdMS_TO_TICKS(10)) == pdPASS) {
        controller = shared_data->motor_controller;
        xSemaphoreGive(shared_data->mutex);
    }
    
    return controller;
}

ble_imu_handle_t* shared_data_get_ble_imu_handle(shared_data_t* shared_data) {
    if (!shared_data) return NULL;
    
    ble_imu_handle_t* handle = NULL;
    if (xSemaphoreTake(shared_data->mutex, pdMS_TO_TICKS(10)) == pdPASS) {
        handle = shared_data->ble_imu_handle;
        xSemaphoreGive(shared_data->mutex);
    }
    
    return handle;
}