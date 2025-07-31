#include "wifi_ap.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "lwip/inet.h"
#include <string.h>
#include <stdlib.h>
#include <sys/socket.h>

static const char* TAG = "WiFiAP";

// =====================================================================================
// --- WiFi AP句柄结构 ---
// =====================================================================================

struct wifi_ap_handle {
    wifi_ap_status_t status;
    wifi_ap_event_callback_t callback;
    esp_netif_t* netif;
    bool initialized;
};

// 静态实例指针（用于事件处理）
static wifi_ap_handle_t* g_wifi_ap_handle = NULL;

// =====================================================================================
// --- WiFi事件处理 ---
// =====================================================================================

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (!g_wifi_ap_handle) return;
    
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_AP_START:
                ESP_LOGI(TAG, "WiFi AP started");
                g_wifi_ap_handle->status.state = WIFI_AP_STARTED;
                if (g_wifi_ap_handle->callback) {
                    g_wifi_ap_handle->callback(WIFI_AP_STARTED, g_wifi_ap_handle->status.connected_clients);
                }
                break;
                
            case WIFI_EVENT_AP_STOP:
                ESP_LOGI(TAG, "WiFi AP stopped");
                g_wifi_ap_handle->status.state = WIFI_AP_STOPPED;
                g_wifi_ap_handle->status.connected_clients = 0;
                if (g_wifi_ap_handle->callback) {
                    g_wifi_ap_handle->callback(WIFI_AP_STOPPED, 0);
                }
                break;
                
            case WIFI_EVENT_AP_STACONNECTED: {
                wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
                g_wifi_ap_handle->status.connected_clients++;
                ESP_LOGI(TAG, "Client connected. MAC: " MACSTR ", AID: %d, Total clients: %d",
                         MAC2STR(event->mac), event->aid, g_wifi_ap_handle->status.connected_clients);
                if (g_wifi_ap_handle->callback) {
                    g_wifi_ap_handle->callback(WIFI_AP_STARTED, g_wifi_ap_handle->status.connected_clients);
                }
                break;
            }
            
            case WIFI_EVENT_AP_STADISCONNECTED: {
                wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
                if (g_wifi_ap_handle->status.connected_clients > 0) {
                    g_wifi_ap_handle->status.connected_clients--;
                }
                ESP_LOGI(TAG, "Client disconnected. MAC: " MACSTR ", AID: %d, Total clients: %d",
                         MAC2STR(event->mac), event->aid, g_wifi_ap_handle->status.connected_clients);
                if (g_wifi_ap_handle->callback) {
                    g_wifi_ap_handle->callback(WIFI_AP_STARTED, g_wifi_ap_handle->status.connected_clients);
                }
                break;
            }
            
            default:
                break;
        }
    }
}

// =====================================================================================
// --- 公共接口实现 ---
// =====================================================================================

wifi_ap_handle_t* wifi_ap_init(wifi_ap_event_callback_t callback) {
    if (g_wifi_ap_handle) {
        ESP_LOGW(TAG, "WiFi AP already initialized");
        return g_wifi_ap_handle;
    }
    
    wifi_ap_handle_t* handle = (wifi_ap_handle_t*)malloc(sizeof(wifi_ap_handle_t));
    if (!handle) {
        ESP_LOGE(TAG, "Failed to allocate memory for WiFi AP handle");
        return NULL;
    }
    
    memset(handle, 0, sizeof(wifi_ap_handle_t));
    handle->callback = callback;
    handle->status.state = WIFI_AP_STOPPED;
    strcpy(handle->status.ssid, WIFI_SSID);
    strcpy(handle->status.ip_address, AP_IP_ADDR);
    
    // 初始化网络接口
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // 创建AP网络接口
    handle->netif = esp_netif_create_default_wifi_ap();
    if (!handle->netif) {
        ESP_LOGE(TAG, "Failed to create default WiFi AP interface");
        free(handle);
        return NULL;
    }
    
    // 配置AP IP地址
    esp_netif_ip_info_t ip_info;
    inet_pton(AF_INET, AP_IP_ADDR, &ip_info.ip);
    inet_pton(AF_INET, "192.168.4.1", &ip_info.gw);
    inet_pton(AF_INET, AP_NETMASK, &ip_info.netmask);
    esp_netif_dhcps_stop(handle->netif);
    esp_netif_set_ip_info(handle->netif, &ip_info);
    esp_netif_dhcps_start(handle->netif);
    
    // 初始化WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // 注册事件处理器
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    
    g_wifi_ap_handle = handle;
    handle->initialized = true;
    
    ESP_LOGI(TAG, "WiFi AP module initialized successfully");
    return handle;
}

bool wifi_ap_start(wifi_ap_handle_t* handle) {
    if (!handle || !handle->initialized) {
        ESP_LOGE(TAG, "WiFi AP handle not initialized");
        return false;
    }
    
    if (handle->status.state == WIFI_AP_STARTED) {
        ESP_LOGW(TAG, "WiFi AP already started");
        return true;
    }
    
    handle->status.state = WIFI_AP_STARTING;
    
    // 配置WiFi AP参数
    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.ap.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.ap.password, WIFI_PASSWORD);
    wifi_config.ap.ssid_len = strlen(WIFI_SSID);
    wifi_config.ap.channel = WIFI_CHANNEL;
    wifi_config.ap.max_connection = MAX_CONNECTIONS;
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    
    // 如果密码为空，使用开放模式
    if (strlen(WIFI_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    
    esp_err_t ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi AP: %s", esp_err_to_name(ret));
        handle->status.state = WIFI_AP_ERROR;
        return false;
    }
    
    ESP_LOGI(TAG, "WiFi AP starting with SSID: %s", WIFI_SSID);
    return true;
}

bool wifi_ap_stop(wifi_ap_handle_t* handle) {
    if (!handle || !handle->initialized) {
        ESP_LOGE(TAG, "WiFi AP handle not initialized");
        return false;
    }
    
    if (handle->status.state == WIFI_AP_STOPPED) {
        ESP_LOGW(TAG, "WiFi AP already stopped");
        return true;
    }
    
    esp_err_t ret = esp_wifi_stop();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop WiFi AP: %s", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGI(TAG, "WiFi AP stopped");
    return true;
}

bool wifi_ap_get_status(wifi_ap_handle_t* handle, wifi_ap_status_t* status) {
    if (!handle || !status) {
        return false;
    }
    
    *status = handle->status;
    return true;
}

uint8_t wifi_ap_get_client_count(wifi_ap_handle_t* handle) {
    if (!handle) {
        return 0;
    }
    
    return handle->status.connected_clients;
}

bool wifi_ap_is_running(wifi_ap_handle_t* handle) {
    if (!handle) {
        return false;
    }
    
    return handle->status.state == WIFI_AP_STARTED;
}

void wifi_ap_destroy(wifi_ap_handle_t* handle) {
    if (!handle) return;
    
    if (handle->status.state == WIFI_AP_STARTED) {
        wifi_ap_stop(handle);
    }
    
    if (handle->initialized) {
        esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler);
        esp_wifi_deinit();
    }
    
    if (handle->netif) {
        esp_netif_destroy(handle->netif);
    }
    
    if (g_wifi_ap_handle == handle) {
        g_wifi_ap_handle = NULL;
    }
    
    free(handle);
    ESP_LOGI(TAG, "WiFi AP module destroyed");
}