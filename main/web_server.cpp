#include "web_server.h"
#include "web_page.h"
#include "esp_log.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

static const char* TAG = "WebServer";

// =====================================================================================
// --- Web服务器句柄结构 ---
// =====================================================================================

// WebSocket客户端信息
typedef struct {
    int fd;
    bool active;
} ws_client_t;

struct web_server_handle {
    web_server_status_t status;
    httpd_handle_t server;
    shared_data_t* shared_data;
    bool initialized;
    
    // WebSocket相关
    ws_client_t ws_clients[WEB_SERVER_MAX_WS_CLIENTS];
    TaskHandle_t ws_broadcast_task_handle;
    SemaphoreHandle_t ws_mutex;
};


// =====================================================================================
// --- HTTP处理函数 ---
// =====================================================================================

// WebSocket测试页面
static const char* test_page = 
"<!DOCTYPE html>"
"<html><head><title>WebSocket Test</title></head><body>"
"<h1>WebSocket Connection Test</h1>"
"<div id='status'>连接中...</div>"
"<div id='log'></div>"
"<script>"
"function log(msg) {"
"  const logDiv = document.getElementById('log');"
"  logDiv.innerHTML += '<div>' + new Date().toLocaleTimeString() + ': ' + msg + '</div>';"
"}"
"const ws = new WebSocket('ws://' + window.location.host + '/ws');"
"ws.onopen = function() { document.getElementById('status').textContent = '已连接'; log('WebSocket连接成功'); };"
"ws.onclose = function() { document.getElementById('status').textContent = '已断开'; log('WebSocket连接关闭'); };"
"ws.onerror = function(e) { document.getElementById('status').textContent = '错误'; log('WebSocket错误: ' + e); };"
"ws.onmessage = function(e) { log('收到消息: ' + e.data); };"
"</script></body></html>";

// WebSocket测试页面处理
static esp_err_t test_handler(httpd_req_t *req) {
    web_server_handle_t* handle = (web_server_handle_t*)req->user_ctx;
    handle->status.request_count++;
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, test_page, strlen(test_page));
    return ESP_OK;
}

// 主页处理
static esp_err_t root_handler(httpd_req_t *req) {
    web_server_handle_t* handle = (web_server_handle_t*)req->user_ctx;
    handle->status.request_count++;
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page_fixed, strlen(html_page_fixed));
    return ESP_OK;
}

// 获取系统状态API
static esp_err_t status_handler(httpd_req_t *req) {
    web_server_handle_t* handle = (web_server_handle_t*)req->user_ctx;
    handle->status.request_count++;
    
    system_status_t status;
    if (!shared_data_get_status(handle->shared_data, &status)) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    cJSON *json = cJSON_CreateObject();
    cJSON *json_ble = cJSON_CreateBool(status.ble_connected);
    cJSON *json_motor = cJSON_CreateBool(status.motor_enabled);
    cJSON *json_tolerance = cJSON_CreateBool(status.in_tolerance);
    cJSON *json_sensor = cJSON_CreateObject();
    
    cJSON_AddItemToObject(json, "ble_connected", json_ble);
    cJSON_AddItemToObject(json, "motor_enabled", json_motor);
    cJSON_AddItemToObject(json, "in_tolerance", json_tolerance);
    
    cJSON_AddNumberToObject(json_sensor, "pitch", status.sensor_data.pitch);
    cJSON_AddNumberToObject(json_sensor, "roll", status.sensor_data.roll);
    cJSON_AddNumberToObject(json_sensor, "yaw", status.sensor_data.yaw);
    cJSON_AddItemToObject(json, "sensor_data", json_sensor);
    
    char *json_string = cJSON_Print(json);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));
    
    free(json_string);
    cJSON_Delete(json);
    return ESP_OK;
}

// 获取配置API
static esp_err_t config_get_handler(httpd_req_t *req) {
    web_server_handle_t* handle = (web_server_handle_t*)req->user_ctx;
    handle->status.request_count++;
    
    balance_config_t config;
    if (!shared_data_get_config(handle->shared_data, &config)) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "target_pitch_angle", config.target_pitch_angle);
    cJSON_AddNumberToObject(json, "pitch_tolerance", config.pitch_tolerance);
    cJSON_AddNumberToObject(json, "motor_fixed_speed", config.motor_fixed_speed);
    cJSON_AddNumberToObject(json, "enable_delay_ms", config.enable_delay_ms);
    cJSON_AddBoolToObject(json, "auto_restart_enabled", config.auto_restart_enabled);
    cJSON_AddNumberToObject(json, "restart_threshold", config.restart_threshold);
    
    char *json_string = cJSON_Print(json);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));
    
    free(json_string);
    cJSON_Delete(json);
    return ESP_OK;
}

// 设置配置API
static esp_err_t config_post_handler(httpd_req_t *req) {
    web_server_handle_t* handle = (web_server_handle_t*)req->user_ctx;
    handle->status.request_count++;
    
    char buf[512];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    
    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    balance_config_t config;
    config.target_pitch_angle = cJSON_GetObjectItem(json, "target_pitch_angle")->valuedouble;
    config.pitch_tolerance = cJSON_GetObjectItem(json, "pitch_tolerance")->valuedouble;
    config.motor_fixed_speed = cJSON_GetObjectItem(json, "motor_fixed_speed")->valuedouble;
    config.enable_delay_ms = cJSON_GetObjectItem(json, "enable_delay_ms")->valuedouble;
    config.auto_restart_enabled = cJSON_IsTrue(cJSON_GetObjectItem(json, "auto_restart_enabled"));
    config.restart_threshold = cJSON_GetObjectItem(json, "restart_threshold")->valuedouble;
    
    bool success = shared_data_set_config(handle->shared_data, &config);
    
    // 如果配置设置成功，同时保存到NVS
    if (success) {
        success = shared_data_save_config_to_nvs(handle->shared_data);
        if (!success) {
            ESP_LOGW(TAG, "Configuration updated but failed to save to NVS");
        }
    }
    
    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "success", success);
    
    char *response_string = cJSON_Print(response);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_string, strlen(response_string));
    
    free(response_string);
    cJSON_Delete(response);
    cJSON_Delete(json);
    return ESP_OK;
}

// 重置配置API
static esp_err_t config_reset_handler(httpd_req_t *req) {
    web_server_handle_t* handle = (web_server_handle_t*)req->user_ctx;
    handle->status.request_count++;
    
    balance_config_t default_config = shared_data_get_default_config();
    bool success = shared_data_set_config(handle->shared_data, &default_config);
    
    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "success", success);
    
    char *response_string = cJSON_Print(response);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_string, strlen(response_string));
    
    free(response_string);
    cJSON_Delete(response);
    return ESP_OK;
}

// =====================================================================================
// --- WebSocket处理函数 ---
// =====================================================================================

// WebSocket数据广播函数 - 优化栈使用
static void ws_broadcast_data(web_server_handle_t* handle) {
    if (!handle || !handle->shared_data) {
        ESP_LOGD(TAG, "Invalid handle or shared_data in ws_broadcast_data");
        return;
    }
    
    // 获取系统状态
    system_status_t status;
    if (!shared_data_get_status(handle->shared_data, &status)) {
        ESP_LOGD(TAG, "Failed to get system status for WebSocket broadcast");
        return;
    }
    
    // 使用静态缓冲区而不是动态分配，减少栈压力
    static char json_buffer[512];
    
    // 手动构建JSON字符串，避免cJSON的栈开销
    int len = snprintf(json_buffer, sizeof(json_buffer),
        "{"
        "\"ble_connected\":%s,"
        "\"motor_enabled\":%s,"
        "\"in_tolerance\":%s,"
        "\"pitch\":%.1f,"
        "\"roll\":%.1f,"
        "\"yaw\":%.1f,"
        "\"uptime_ms\":%lu"
        "}",
        status.ble_connected ? "true" : "false",
        status.motor_enabled ? "true" : "false", 
        status.in_tolerance ? "true" : "false",
        status.sensor_data.pitch,
        status.sensor_data.roll,
        status.sensor_data.yaw,
        status.uptime_ms
    );
    
    if (len <= 0 || len >= sizeof(json_buffer)) {
        ESP_LOGW(TAG, "JSON buffer overflow or format error");
        return;
    }
    
    // 广播给所有WebSocket客户端
    if (xSemaphoreTake(handle->ws_mutex, pdMS_TO_TICKS(5)) == pdPASS) {
        int active_clients = 0;
        for (int i = 0; i < WEB_SERVER_MAX_WS_CLIENTS; i++) {
            if (handle->ws_clients[i].active) {
                active_clients++;
                httpd_ws_frame_t ws_pkt;
                memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
                ws_pkt.payload = (uint8_t*)json_buffer;
                ws_pkt.len = (size_t)len;
                ws_pkt.type = HTTPD_WS_TYPE_TEXT;
                
                esp_err_t ret = httpd_ws_send_frame_async(handle->server, handle->ws_clients[i].fd, &ws_pkt);
                if (ret != ESP_OK) {
                    ESP_LOGD(TAG, "WebSocket client %d disconnected: %s", i, esp_err_to_name(ret));
                    handle->ws_clients[i].active = false;
                    active_clients--;
                }
            }
        }
        
        // 每10秒记录一次活跃连接数
        static int log_counter = 0;
        if (++log_counter >= 10) {
            ESP_LOGI(TAG, "WebSocket: %d active clients, broadcasting: %.50s...", active_clients, json_buffer);
            log_counter = 0;
        }
        
        xSemaphoreGive(handle->ws_mutex);
    }
}

// WebSocket广播任务
static void ws_broadcast_task(void* parameter) {
    web_server_handle_t* handle = (web_server_handle_t*)parameter;
    
    ESP_LOGI(TAG, "WebSocket broadcast task started");
    
    while (handle) {
        // 检查服务器状态
        if (handle->status.state != WEB_SERVER_RUNNING) {
            ESP_LOGW(TAG, "Web server not running, WebSocket task exiting");
            break;
        }
        
        ws_broadcast_data(handle);
        vTaskDelay(pdMS_TO_TICKS(100)); // 每100ms广播一次
    }
    
    ESP_LOGI(TAG, "WebSocket broadcast task ended");
    vTaskDelete(NULL);
}

// WebSocket处理函数
static esp_err_t ws_handler(httpd_req_t *req) {
    web_server_handle_t* handle = (web_server_handle_t*)req->user_ctx;
    
    ESP_LOGI(TAG, "WebSocket handler called, method: %d", req->method);
    
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WebSocket connection established");
        
        // 添加客户端到列表
        if (xSemaphoreTake(handle->ws_mutex, pdMS_TO_TICKS(100)) == pdPASS) {
            for (int i = 0; i < WEB_SERVER_MAX_WS_CLIENTS; i++) {
                if (!handle->ws_clients[i].active) {
                    handle->ws_clients[i].fd = httpd_req_to_sockfd(req);
                    handle->ws_clients[i].active = true;
                    ESP_LOGI(TAG, "WebSocket client %d connected (fd=%d)", i, handle->ws_clients[i].fd);
                    break;
                }
            }
            xSemaphoreGive(handle->ws_mutex);
        }
        
        return ESP_OK;
    }
    
    // 处理WebSocket帧
    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get WebSocket frame length: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (ws_pkt.len) {
        buf = (uint8_t*)calloc(1, ws_pkt.len + 1);
        if (buf == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory for WebSocket buffer");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to receive WebSocket frame: %s", esp_err_to_name(ret));
            free(buf);
            return ret;
        }
        
        ESP_LOGI(TAG, "Received WebSocket message: %s", ws_pkt.payload);
        free(buf);
    }
    
    return ESP_OK;
}

// =====================================================================================
// --- 公共接口实现 ---
// =====================================================================================

web_server_handle_t* web_server_init(shared_data_t* shared_data) {
    if (!shared_data) {
        ESP_LOGE(TAG, "Shared data is required");
        return NULL;
    }
    
    web_server_handle_t* handle = (web_server_handle_t*)malloc(sizeof(web_server_handle_t));
    if (!handle) {
        ESP_LOGE(TAG, "Failed to allocate memory for web server handle");
        return NULL;
    }
    
    memset(handle, 0, sizeof(web_server_handle_t));
    handle->shared_data = shared_data;
    handle->status.state = WEB_SERVER_STOPPED;
    handle->status.port = WEB_SERVER_PORT;
    handle->initialized = true;
    
    // 初始化WebSocket相关资源
    handle->ws_mutex = xSemaphoreCreateMutex();
    if (!handle->ws_mutex) {
        ESP_LOGE(TAG, "Failed to create WebSocket mutex");
        free(handle);
        return NULL;
    }
    
    // 初始化WebSocket客户端列表
    for (int i = 0; i < WEB_SERVER_MAX_WS_CLIENTS; i++) {
        handle->ws_clients[i].active = false;
        handle->ws_clients[i].fd = -1;
    }
    
    handle->ws_broadcast_task_handle = NULL;
    
    ESP_LOGI(TAG, "Web server module initialized successfully");
    return handle;
}

bool web_server_start(web_server_handle_t* handle) {
    if (!handle || !handle->initialized) {
        ESP_LOGE(TAG, "Web server handle not initialized");
        return false;
    }
    
    if (handle->status.state == WEB_SERVER_RUNNING) {
        ESP_LOGW(TAG, "Web server already running");
        return true;
    }
    
    handle->status.state = WEB_SERVER_STARTING;
    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = WEB_SERVER_PORT;
    config.max_open_sockets = 7;
    config.max_uri_handlers = 8;
    
    esp_err_t ret = httpd_start(&handle->server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        handle->status.state = WEB_SERVER_ERROR;
        return false;
    }
    
    // 注册URI处理器
    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_handler,
        .user_ctx = handle
    };
    httpd_register_uri_handler(handle->server, &root_uri);
    
    httpd_uri_t test_uri = {
        .uri = "/test",
        .method = HTTP_GET,
        .handler = test_handler,
        .user_ctx = handle
    };
    httpd_register_uri_handler(handle->server, &test_uri);
    
    httpd_uri_t status_uri = {
        .uri = "/api/status",
        .method = HTTP_GET,
        .handler = status_handler,
        .user_ctx = handle
    };
    httpd_register_uri_handler(handle->server, &status_uri);
    
    httpd_uri_t config_get_uri = {
        .uri = "/api/config",
        .method = HTTP_GET,
        .handler = config_get_handler,
        .user_ctx = handle
    };
    httpd_register_uri_handler(handle->server, &config_get_uri);
    
    httpd_uri_t config_post_uri = {
        .uri = "/api/config",
        .method = HTTP_POST,
        .handler = config_post_handler,
        .user_ctx = handle
    };
    httpd_register_uri_handler(handle->server, &config_post_uri);
    
    httpd_uri_t config_reset_uri = {
        .uri = "/api/config/reset",
        .method = HTTP_POST,
        .handler = config_reset_handler,
        .user_ctx = handle
    };
    httpd_register_uri_handler(handle->server, &config_reset_uri);
    
    // 注册WebSocket处理器
    httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .user_ctx = handle,
        .is_websocket = true
    };
    httpd_register_uri_handler(handle->server, &ws_uri);
    
    // 确保状态已设置为RUNNING，任务才能正常运行
    handle->status.state = WEB_SERVER_RUNNING;
    
    // 创建WebSocket广播任务（更大的栈空间）
    BaseType_t task_created = xTaskCreate(
        ws_broadcast_task,           // 任务函数
        "ws_broadcast",              // 任务名称  
        4096,                        // 栈大小（4KB）
        handle,                      // 参数
        3,                           // 优先级（中等）
        &handle->ws_broadcast_task_handle // 任务句柄
    );
    
    if (task_created == pdPASS) {
        ESP_LOGI(TAG, "WebSocket broadcast task created successfully");
    } else {
        ESP_LOGE(TAG, "Failed to create WebSocket broadcast task");
        return false;
    }
    
    ESP_LOGI(TAG, "Web server started on port %d with WebSocket support", WEB_SERVER_PORT);
    return true;
}

bool web_server_stop(web_server_handle_t* handle) {
    if (!handle || !handle->initialized) {
        ESP_LOGE(TAG, "Web server handle not initialized");
        return false;
    }
    
    if (handle->status.state == WEB_SERVER_STOPPED) {
        ESP_LOGW(TAG, "Web server already stopped");
        return true;
    }
    
    // 停止WebSocket广播任务
    if (handle->ws_broadcast_task_handle) {
        vTaskDelete(handle->ws_broadcast_task_handle);
        handle->ws_broadcast_task_handle = NULL;
        ESP_LOGI(TAG, "WebSocket broadcast task stopped");
    }
    
    // 清理WebSocket客户端
    if (xSemaphoreTake(handle->ws_mutex, pdMS_TO_TICKS(100)) == pdPASS) {
        for (int i = 0; i < WEB_SERVER_MAX_WS_CLIENTS; i++) {
            handle->ws_clients[i].active = false;
            handle->ws_clients[i].fd = -1;
        }
        xSemaphoreGive(handle->ws_mutex);
    }
    
    if (handle->server) {
        esp_err_t ret = httpd_stop(handle->server);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to stop HTTP server: %s", esp_err_to_name(ret));
            return false;
        }
        handle->server = NULL;
    }
    
    handle->status.state = WEB_SERVER_STOPPED;
    ESP_LOGI(TAG, "Web server stopped");
    return true;
}

bool web_server_get_status(web_server_handle_t* handle, web_server_status_t* status) {
    if (!handle || !status) {
        return false;
    }
    
    *status = handle->status;
    return true;
}

bool web_server_is_running(web_server_handle_t* handle) {
    if (!handle) {
        return false;
    }
    
    return handle->status.state == WEB_SERVER_RUNNING;
}

void web_server_destroy(web_server_handle_t* handle) {
    if (!handle) return;
    
    if (handle->status.state == WEB_SERVER_RUNNING) {
        web_server_stop(handle);
    }
    
    // 清理WebSocket互斥锁
    if (handle->ws_mutex) {
        vSemaphoreDelete(handle->ws_mutex);
    }
    
    free(handle);
    ESP_LOGI(TAG, "Web server module destroyed");
}