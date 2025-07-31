#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "esp_log.h"
#include "nvs_flash.h"

// 模块化系统组件
#include "shared_data.h"
#include "wifi_ap.h"
#include "web_server.h"
#include "balance_controller.h"
#include "motor_control.h"
#include "ble_imu.h"

static const char* TAG = "MainSystem";

// =====================================================================================
// --- 硬件配置常量 ---
// =====================================================================================

#define MOTOR_UART_PORT     UART_NUM_2
#define MOTOR_UART_TXD      GPIO_NUM_13
#define MOTOR_UART_RXD      GPIO_NUM_12
#define UART_BUF_SIZE       (1024)

// =====================================================================================
// --- 系统模块句柄 ---
// =====================================================================================

typedef struct {
    shared_data_t* shared_data;
    wifi_ap_handle_t* wifi_ap;
    web_server_handle_t* web_server;
    balance_controller_handle_t* balance_controller;
    motor_controller_t* motor_controller;
    ble_imu_handle_t* ble_imu_handle;
} system_modules_t;

static system_modules_t g_system = {
    .shared_data = NULL,
    .wifi_ap = NULL,
    .web_server = NULL,
    .balance_controller = NULL,
    .motor_controller = NULL,
    .ble_imu_handle = NULL
};

// =====================================================================================
// --- WiFi事件回调 ---
// =====================================================================================

static void wifi_event_callback(wifi_ap_state_t state, uint8_t connected_clients) {
    ESP_LOGI(TAG, "WiFi AP状态: %d, 连接客户端数: %d", state, connected_clients);
}

// =====================================================================================
// --- 系统初始化函数 ---
// =====================================================================================

static bool system_init_shared_data(void) {
    g_system.shared_data = shared_data_init();
    if (!g_system.shared_data) {
        ESP_LOGE(TAG, "共享数据初始化失败");
        return false;
    }
    ESP_LOGI(TAG, "✓ 共享数据模块初始化成功");
    return true;
}

static bool system_init_ble_imu(void) {
    g_system.ble_imu_handle = ble_imu_init();
    if (!g_system.ble_imu_handle) {
        ESP_LOGE(TAG, "BLE IMU初始化失败");
        return false;
    }
    
    // 将BLE IMU句柄注册到共享数据
    shared_data_set_ble_imu_handle(g_system.shared_data, g_system.ble_imu_handle);
    ESP_LOGI(TAG, "✓ BLE IMU模块初始化成功");
    return true;
}

static bool system_init_motor_controller(void) {
    motor_driver_config_t motor_config = {
        .uart_port = MOTOR_UART_PORT,
        .txd_pin = MOTOR_UART_TXD,
        .rxd_pin = MOTOR_UART_RXD,
        .baud_rate = 115200,
        .buf_size = UART_BUF_SIZE
    };
    
    balance_config_t config;
    shared_data_get_config(g_system.shared_data, &config);
    
    g_system.motor_controller = motor_control_init(&motor_config, config.motor_fixed_speed);
    if (!g_system.motor_controller) {
        ESP_LOGE(TAG, "电机控制器初始化失败");
        return false;
    }
    
    // 将电机控制器句柄注册到共享数据
    shared_data_set_motor_controller(g_system.shared_data, g_system.motor_controller);
    ESP_LOGI(TAG, "✓ 电机控制器模块初始化成功");
    return true;
}

static bool system_init_wifi_ap(void) {
    g_system.wifi_ap = wifi_ap_init(wifi_event_callback);
    if (!g_system.wifi_ap) {
        ESP_LOGE(TAG, "WiFi AP初始化失败");
        return false;
    }
    
    if (!wifi_ap_start(g_system.wifi_ap)) {
        ESP_LOGE(TAG, "WiFi AP启动失败");
        return false;
    }
    
    ESP_LOGI(TAG, "✓ WiFi AP模块初始化并启动成功");
    return true;
}

static bool system_init_web_server(void) {
    g_system.web_server = web_server_init(g_system.shared_data);
    if (!g_system.web_server) {
        ESP_LOGE(TAG, "Web服务器初始化失败");
        return false;
    }
    
    if (!web_server_start(g_system.web_server)) {
        ESP_LOGE(TAG, "Web服务器启动失败");
        return false;
    }
    
    ESP_LOGI(TAG, "✓ Web服务器模块初始化并启动成功");
    return true;
}

static bool system_init_balance_controller(void) {
    g_system.balance_controller = balance_controller_init(g_system.shared_data);
    if (!g_system.balance_controller) {
        ESP_LOGE(TAG, "平衡控制器初始化失败");
        return false;
    }
    
    if (!balance_controller_start(g_system.balance_controller)) {
        ESP_LOGE(TAG, "平衡控制器启动失败");
        return false;
    }
    
    ESP_LOGI(TAG, "✓ 平衡控制器模块初始化并启动成功");
    return true;
}

// =====================================================================================
// --- 系统销毁函数 ---
// =====================================================================================

static void system_destroy(void) {
    if (g_system.balance_controller) {
        balance_controller_destroy(g_system.balance_controller);
        g_system.balance_controller = NULL;
    }
    
    if (g_system.web_server) {
        web_server_destroy(g_system.web_server);
        g_system.web_server = NULL;
    }
    
    if (g_system.wifi_ap) {
        wifi_ap_destroy(g_system.wifi_ap);
        g_system.wifi_ap = NULL;
    }
    
    if (g_system.motor_controller) {
        motor_control_deinit(g_system.motor_controller);
        g_system.motor_controller = NULL;
    }
    
    if (g_system.ble_imu_handle) {
        ble_imu_destroy(g_system.ble_imu_handle);
        g_system.ble_imu_handle = NULL;
    }
    
    if (g_system.shared_data) {
        shared_data_destroy(g_system.shared_data);
        g_system.shared_data = NULL;
    }
    
    ESP_LOGI(TAG, "系统模块全部销毁完成");
}

// =====================================================================================
// --- 主程序入口 (Main Entry Point) ---
// =====================================================================================

extern "C" void app_main(void) {
    // 禁用看门狗
    esp_task_wdt_deinit();
    
    ESP_LOGI(TAG, "🚀 ESP32 平衡车双协议通信系统启动中...");
    
    // 初始化NVS - 必须在其他模块之前
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS分区需要擦除，正在重新初始化...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ NVS初始化失败: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "✓ NVS存储系统初始化成功");
    
    ESP_LOGI(TAG, "--- 系统将在5秒后启动，请保持设备静止 ---");
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    ESP_LOGI(TAG, "========== 模块化系统初始化开始 ==========");
    
    // 按依赖顺序初始化所有模块
    if (!system_init_shared_data()) {
        ESP_LOGE(TAG, "❌ 系统初始化失败：共享数据模块");
        goto cleanup;
    }
    
    if (!system_init_ble_imu()) {
        ESP_LOGE(TAG, "❌ 系统初始化失败：BLE IMU模块");
        goto cleanup;
    }
    
    if (!system_init_motor_controller()) {
        ESP_LOGE(TAG, "❌ 系统初始化失败：电机控制器模块");
        goto cleanup;
    }
    
    if (!system_init_wifi_ap()) {
        ESP_LOGE(TAG, "❌ 系统初始化失败：WiFi AP模块");
        goto cleanup;
    }
    
    if (!system_init_web_server()) {
        ESP_LOGE(TAG, "❌ 系统初始化失败：Web服务器模块");
        goto cleanup;
    }
    
    if (!system_init_balance_controller()) {
        ESP_LOGE(TAG, "❌ 系统初始化失败：平衡控制器模块");
        goto cleanup;
    }
    
    ESP_LOGI(TAG, "========== 系统初始化完成 ==========");
    ESP_LOGI(TAG, "🎯 系统功能:");
    ESP_LOGI(TAG, "  📡 BLE IMU: 实时姿态数据采集 (e8:cb:ed:5a:52:8e)");
    ESP_LOGI(TAG, "  ⚖️  平衡控制: 基于PITCH角度自动调节");
    ESP_LOGI(TAG, "  📶 WiFi AP: %s (密码: %s)", "ESP32_Balance_Config", "balance123");
    ESP_LOGI(TAG, "  🌐 Web界面: http://192.168.4.1");
    ESP_LOGI(TAG, "  📊 实时监控: 串口输出 + Web界面");
    
    ESP_LOGI(TAG, "✅ 系统运行中，所有模块协调工作");
    
    // 主循环：监控系统状态
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // 每10秒检查一次
        
        // 可以在这里添加系统健康检查逻辑
        // 例如：检查各模块状态，记录运行统计等
    }
    
cleanup:
    ESP_LOGE(TAG, "系统初始化失败，开始清理资源...");
    system_destroy();
}
