#pragma once

#include "esp_wifi.h"
#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

// =====================================================================================
// --- WiFi AP配置常量 ---
// =====================================================================================

#define WIFI_SSID      "ESP32_Balance_Config"
#define WIFI_PASSWORD  "balance123"
#define WIFI_CHANNEL   6
#define MAX_CONNECTIONS 4

#define AP_IP_ADDR     "192.168.4.1"
#define AP_NETMASK     "255.255.255.0"

// =====================================================================================
// --- WiFi AP状态 ---
// =====================================================================================

typedef enum {
    WIFI_AP_STOPPED,
    WIFI_AP_STARTING,
    WIFI_AP_STARTED,
    WIFI_AP_ERROR
} wifi_ap_state_t;

typedef struct {
    wifi_ap_state_t state;
    uint8_t connected_clients;
    char ssid[32];
    char ip_address[16];
} wifi_ap_status_t;

// =====================================================================================
// --- WiFi AP模块句柄 ---
// =====================================================================================

typedef struct wifi_ap_handle wifi_ap_handle_t;

// =====================================================================================
// --- 回调函数类型 ---
// =====================================================================================

typedef void (*wifi_ap_event_callback_t)(wifi_ap_state_t state, uint8_t connected_clients);

// =====================================================================================
// --- 公共接口函数 ---
// =====================================================================================

/**
 * @brief 初始化WiFi AP模块
 * @param callback 状态变化回调函数（可选）
 * @return WiFi AP句柄，失败返回NULL
 */
wifi_ap_handle_t* wifi_ap_init(wifi_ap_event_callback_t callback);

/**
 * @brief 启动WiFi AP热点
 * @param handle WiFi AP句柄
 * @return true成功，false失败
 */
bool wifi_ap_start(wifi_ap_handle_t* handle);

/**
 * @brief 停止WiFi AP热点
 * @param handle WiFi AP句柄
 * @return true成功，false失败
 */
bool wifi_ap_stop(wifi_ap_handle_t* handle);

/**
 * @brief 获取WiFi AP状态
 * @param handle WiFi AP句柄
 * @param status 状态结构体指针
 * @return true成功，false失败
 */
bool wifi_ap_get_status(wifi_ap_handle_t* handle, wifi_ap_status_t* status);

/**
 * @brief 获取当前连接的客户端数量
 * @param handle WiFi AP句柄
 * @return 连接的客户端数量
 */
uint8_t wifi_ap_get_client_count(wifi_ap_handle_t* handle);

/**
 * @brief 销毁WiFi AP模块
 * @param handle WiFi AP句柄
 */
void wifi_ap_destroy(wifi_ap_handle_t* handle);

/**
 * @brief 检查WiFi AP是否正在运行
 * @param handle WiFi AP句柄
 * @return true正在运行，false未运行
 */
bool wifi_ap_is_running(wifi_ap_handle_t* handle);

#ifdef __cplusplus
}
#endif