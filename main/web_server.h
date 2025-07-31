#pragma once

#include "esp_http_server.h"
#include "shared_data.h"

#ifdef __cplusplus
extern "C" {
#endif

// =====================================================================================
// --- Web服务器配置常量 ---
// =====================================================================================

#define WEB_SERVER_PORT 80
#define WEB_SERVER_MAX_URI_LEN 128
#define WEB_SERVER_MAX_RESP_LEN 2048

// =====================================================================================
// --- Web服务器状态 ---
// =====================================================================================

typedef enum {
    WEB_SERVER_STOPPED,
    WEB_SERVER_STARTING,
    WEB_SERVER_RUNNING,
    WEB_SERVER_ERROR
} web_server_state_t;

typedef struct {
    web_server_state_t state;
    uint16_t port;
    uint32_t request_count;
    uint32_t active_connections;
} web_server_status_t;

// =====================================================================================
// --- Web服务器句柄 ---
// =====================================================================================

typedef struct web_server_handle web_server_handle_t;

// =====================================================================================
// --- 公共接口函数 ---
// =====================================================================================

/**
 * @brief 初始化Web服务器模块
 * @param shared_data 共享数据句柄
 * @return Web服务器句柄，失败返回NULL
 */
web_server_handle_t* web_server_init(shared_data_t* shared_data);

/**
 * @brief 启动Web服务器
 * @param handle Web服务器句柄
 * @return true成功，false失败
 */
bool web_server_start(web_server_handle_t* handle);

/**
 * @brief 停止Web服务器
 * @param handle Web服务器句柄
 * @return true成功，false失败
 */
bool web_server_stop(web_server_handle_t* handle);

/**
 * @brief 获取Web服务器状态
 * @param handle Web服务器句柄
 * @param status 状态结构体指针
 * @return true成功，false失败
 */
bool web_server_get_status(web_server_handle_t* handle, web_server_status_t* status);

/**
 * @brief 检查Web服务器是否正在运行
 * @param handle Web服务器句柄
 * @return true正在运行，false未运行
 */
bool web_server_is_running(web_server_handle_t* handle);

/**
 * @brief 销毁Web服务器模块
 * @param handle Web服务器句柄
 */
void web_server_destroy(web_server_handle_t* handle);

#ifdef __cplusplus
}
#endif