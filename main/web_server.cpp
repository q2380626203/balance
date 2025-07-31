#include "web_server.h"
#include "esp_log.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>

static const char* TAG = "WebServer";

// =====================================================================================
// --- Web服务器句柄结构 ---
// =====================================================================================

struct web_server_handle {
    web_server_status_t status;
    httpd_handle_t server;
    shared_data_t* shared_data;
    bool initialized;
};

// =====================================================================================
// --- HTML页面内容 ---
// =====================================================================================

static const char* html_page = 
"<!DOCTYPE html>"
"<html lang='zh-CN'>"
"<head>"
"    <meta charset='UTF-8'>"
"    <meta name='viewport' content='width=device-width, initial-scale=1.0'>"
"    <title>ESP32 平衡车控制</title>"
"    <style>"
"        * { margin: 0; padding: 0; box-sizing: border-box; }"
"        body { font-family: Arial, sans-serif; background: #f5f5f5; padding: 20px; }"
"        .container { max-width: 800px; margin: 0 auto; background: white; border-radius: 10px; padding: 20px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }"
"        h1 { text-align: center; color: #333; margin-bottom: 30px; }"
"        .status-panel { background: #e8f4fd; padding: 15px; border-radius: 8px; margin-bottom: 20px; }"
"        .status-item { display: flex; justify-content: space-between; margin: 5px 0; }"
"        .config-panel { background: #f8f9fa; padding: 15px; border-radius: 8px; margin-bottom: 20px; }"
"        .config-item { margin: 15px 0; }"
"        .config-item label { display: block; margin-bottom: 5px; font-weight: bold; }"
"        .config-item input { width: 100%; padding: 8px; border: 1px solid #ddd; border-radius: 4px; }"
"        .button-group { text-align: center; margin-top: 20px; }"
"        .btn { padding: 10px 20px; margin: 0 10px; border: none; border-radius: 5px; cursor: pointer; font-size: 16px; }"
"        .btn-primary { background: #007bff; color: white; }"
"        .btn-success { background: #28a745; color: white; }"
"        .btn-warning { background: #ffc107; color: black; }"
"        .btn:hover { opacity: 0.8; }"
"        .data-display { font-family: monospace; font-size: 18px; font-weight: bold; }"
"        @media (max-width: 600px) { .container { padding: 15px; } .btn { display: block; width: 100%; margin: 5px 0; } }"
"    </style>"
"</head>"
"<body>"
"    <div class='container'>"
"        <h1>🤖 ESP32 平衡车控制台</h1>"
"        "
"        <div class='status-panel'>"
"            <h3>📊 实时状态</h3>"
"            <div class='status-item'><span>BLE连接状态:</span> <span id='ble-status'>检查中...</span></div>"
"            <div class='status-item'><span>WiFi客户端:</span> <span id='wifi-clients'>0</span></div>"
"            <div class='status-item'><span>当前Pitch角:</span> <span id='pitch-angle' class='data-display'>0.0°</span></div>"
"            <div class='status-item'><span>电机状态:</span> <span id='motor-status'>停止</span></div>"
"            <div class='status-item'><span>平衡状态:</span> <span id='balance-status'>平衡</span></div>"
"        </div>"
"        "
"        <div class='config-panel'>"
"            <h3>⚙️ 参数配置</h3>"
"            <div class='config-item'>"
"                <label for='target-pitch'>目标Pitch角 (度):</label>"
"                <input type='number' id='target-pitch' step='0.1' value='0.0'>"
"            </div>"
"            <div class='config-item'>"
"                <label for='pitch-tolerance'>Pitch容差 (度):</label>"
"                <input type='number' id='pitch-tolerance' step='0.5' value='10.0'>"
"            </div>"
"            <div class='config-item'>"
"                <label for='motor-speed'>电机速度:</label>"
"                <input type='number' id='motor-speed' step='1.0' value='30.0'>"
"            </div>"
"            <div class='config-item'>"
"                <label for='enable-delay'>启动延时 (ms):</label>"
"                <input type='number' id='enable-delay' step='50' value='500'>"
"            </div>"
"        </div>"
"        "
"        <div class='button-group'>"
"            <button class='btn btn-primary' onclick='saveConfig()'>💾 保存配置</button>"
"            <button class='btn btn-warning' onclick='resetConfig()'>🔄 恢复默认</button>"
"            <button class='btn btn-success' onclick='refreshData()'>📡 刷新数据</button>"
"        </div>"
"    </div>"
"    "
"    <script>"
"        function updateStatus() {"
"            fetch('/api/status')"
"                .then(response => response.json())"
"                .then(data => {"
"                    document.getElementById('ble-status').textContent = data.ble_connected ? '已连接' : '未连接';"
"                    document.getElementById('pitch-angle').textContent = data.sensor_data.pitch.toFixed(1) + '°';"
"                    document.getElementById('motor-status').textContent = data.motor_enabled ? '运行中' : '停止';"
"                    document.getElementById('balance-status').textContent = data.in_tolerance ? '平衡' : '调节中';"
"                });"
"        }"
"        "
"        function loadConfig() {"
"            fetch('/api/config')"
"                .then(response => response.json())"
"                .then(data => {"
"                    document.getElementById('target-pitch').value = data.target_pitch_angle;"
"                    document.getElementById('pitch-tolerance').value = data.pitch_tolerance;"
"                    document.getElementById('motor-speed').value = data.motor_fixed_speed;"
"                    document.getElementById('enable-delay').value = data.enable_delay_ms;"
"                });"
"        }"
"        "
"        function saveConfig() {"
"            const config = {"
"                target_pitch_angle: parseFloat(document.getElementById('target-pitch').value),"
"                pitch_tolerance: parseFloat(document.getElementById('pitch-tolerance').value),"
"                motor_fixed_speed: parseFloat(document.getElementById('motor-speed').value),"
"                enable_delay_ms: parseFloat(document.getElementById('enable-delay').value),"
"                auto_restart_enabled: true,"
"                restart_threshold: 1.0"
"            };"
"            "
"            fetch('/api/config', {"
"                method: 'POST',"
"                headers: { 'Content-Type': 'application/json' },"
"                body: JSON.stringify(config)"
"            })"
"            .then(response => response.json())"
"            .then(data => {"
"                alert(data.success ? '配置保存成功！' : '配置保存失败！');"
"            });"
"        }"
"        "
"        function resetConfig() {"
"            if (confirm('确定要恢复默认配置吗？')) {"
"                fetch('/api/config/reset', { method: 'POST' })"
"                    .then(response => response.json())"
"                    .then(data => {"
"                        if (data.success) {"
"                            loadConfig();"
"                            alert('已恢复默认配置！');"
"                        }"
"                    });"
"            }"
"        }"
"        "
"        function refreshData() {"
"            loadConfig();"
"            updateStatus();"
"        }"
"        "
"        // 页面加载时初始化"
"        window.onload = function() {"
"            loadConfig();"
"            updateStatus();"
"            setInterval(updateStatus, 1000); // 每秒更新状态"
"        };"
"    </script>"
"</body>"
"</html>";

// =====================================================================================
// --- HTTP处理函数 ---
// =====================================================================================

// 主页处理
static esp_err_t root_handler(httpd_req_t *req) {
    web_server_handle_t* handle = (web_server_handle_t*)req->user_ctx;
    handle->status.request_count++;
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, strlen(html_page));
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
    
    handle->status.state = WEB_SERVER_RUNNING;
    ESP_LOGI(TAG, "Web server started on port %d", WEB_SERVER_PORT);
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
    
    free(handle);
    ESP_LOGI(TAG, "Web server module destroyed");
}