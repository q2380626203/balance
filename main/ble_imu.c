/**
 * @file ble_imu.c
 * @brief BLE IMU 连接器模块实现
 */

#include "ble_imu.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"

/* BLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "esp_central.h"

static const char *TAG = "BLE_IMU";

// 目标IMU设备地址 (e8:cb:ed:5a:52:8e)
static const uint8_t target_addr[6] = {0x8e, 0x52, 0x5a, 0xed, 0xcb, 0xe8};

// IMU设备UUID常量
static const ble_uuid128_t imu_service_uuid = 
    BLE_UUID128_INIT(0xfb, 0x34, 0x9a, 0x5f, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0xe5, 0xff, 0x00, 0x00);
                     
static const ble_uuid128_t imu_read_char_uuid = 
    BLE_UUID128_INIT(0xfb, 0x34, 0x9a, 0x5f, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0xe4, 0xff, 0x00, 0x00);
                     
static const ble_uuid128_t imu_write_char_uuid = 
    BLE_UUID128_INIT(0xfb, 0x34, 0x9a, 0x5f, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0xe9, 0xff, 0x00, 0x00);

// BLE IMU句柄结构
struct ble_imu_handle {
    // BLE连接状态
    bool is_connected;
    uint16_t conn_handle;
    uint16_t write_char_handle;
    uint16_t read_char_handle;
    
    // 数据缓存和保护
    ble_imu_data_t sensor_data;
    SemaphoreHandle_t data_mutex;
    
    // 统计信息
    uint32_t total_bytes_received;
    uint32_t packet_count;
    
    // 临时数据缓存
    uint8_t temp_bytes[20];
    int temp_bytes_len;
    
    // 初始化标志
    bool initialized;
};

// 全局句柄（单例模式）
static ble_imu_handle_t* g_ble_imu_handle = NULL;

// 前置声明
static void ble_imu_scan(void);
static int ble_gap_event(struct ble_gap_event *event, void *arg);
void ble_store_config_init(void);

/**
 * 格式化地址字符串
 */
static char* format_addr_str(const uint8_t *addr) {
    static char addr_str_buf[18];
    snprintf(addr_str_buf, sizeof(addr_str_buf), "%02x:%02x:%02x:%02x:%02x:%02x",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
    return addr_str_buf;
}

/**
 * 获取有符号16位数
 */
static int16_t get_sign_int16(uint16_t num) {
    if (num >= 32768) {
        return num - 65536;
    }
    return num;
}

/**
 * 处理IMU数据包
 */
static void process_imu_data(ble_imu_handle_t* handle, const uint8_t *data, int len) {
    if (!handle || len != 20) {
        return;
    }

    if (data[0] != 0x55) {
        return;
    }

    ble_imu_data_t temp_data = handle->sensor_data; // 保持之前的数据

    if (data[1] == 0x61) {
        // 9轴传感器数据 (加速度 + 角速度 + 角度)
        int16_t ax_raw = (data[3] << 8) | data[2];
        int16_t ay_raw = (data[5] << 8) | data[4];
        int16_t az_raw = (data[7] << 8) | data[6];
        int16_t gx_raw = (data[9] << 8) | data[8];
        int16_t gy_raw = (data[11] << 8) | data[10];
        int16_t gz_raw = (data[13] << 8) | data[12];
        int16_t angx_raw = (data[15] << 8) | data[14];
        int16_t angy_raw = (data[17] << 8) | data[16];
        int16_t angz_raw = (data[19] << 8) | data[18];

        // 转换为实际值
        temp_data.acc_x = get_sign_int16(ax_raw) / 32768.0f * 16.0f;
        temp_data.acc_y = get_sign_int16(ay_raw) / 32768.0f * 16.0f;
        temp_data.acc_z = get_sign_int16(az_raw) / 32768.0f * 16.0f;
        
        temp_data.gyro_x = get_sign_int16(gx_raw) / 32768.0f * 2000.0f;
        temp_data.gyro_y = get_sign_int16(gy_raw) / 32768.0f * 2000.0f;
        temp_data.gyro_z = get_sign_int16(gz_raw) / 32768.0f * 2000.0f;
        
        // 重要：角度映射适配平衡车控制
        temp_data.pitch = get_sign_int16(angx_raw) / 32768.0f * 180.0f;  // angle_x -> pitch
        temp_data.roll = get_sign_int16(angy_raw) / 32768.0f * 180.0f;   // angle_y -> roll (平衡车主控制轴)
        temp_data.yaw = get_sign_int16(angz_raw) / 32768.0f * 180.0f;    // angle_z -> yaw

        temp_data.is_valid = true;
        temp_data.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;

    } else if (data[1] == 0x71) {
        // 扩展数据 (磁场 + 四元数)
        if (data[2] == 0x3A) {
            // 磁场数据
            int16_t hx_raw = (data[5] << 8) | data[4];
            int16_t hy_raw = (data[7] << 8) | data[6];
            int16_t hz_raw = (data[9] << 8) | data[8];
            
            temp_data.mag_x = get_sign_int16(hx_raw) / 120.0f;
            temp_data.mag_y = get_sign_int16(hy_raw) / 120.0f;
            temp_data.mag_z = get_sign_int16(hz_raw) / 120.0f;
                     
        } else if (data[2] == 0x51) {
            // 四元数数据
            int16_t q0_raw = (data[5] << 8) | data[4];
            int16_t q1_raw = (data[7] << 8) | data[6];
            int16_t q2_raw = (data[9] << 8) | data[8];
            int16_t q3_raw = (data[11] << 8) | data[10];
            
            temp_data.q0 = get_sign_int16(q0_raw) / 32768.0f;
            temp_data.q1 = get_sign_int16(q1_raw) / 32768.0f;
            temp_data.q2 = get_sign_int16(q2_raw) / 32768.0f;
            temp_data.q3 = get_sign_int16(q3_raw) / 32768.0f;
        }
    }

    // 线程安全更新数据
    if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        handle->sensor_data = temp_data;
        handle->sensor_data.packet_count++;
        handle->total_bytes_received += len;
        xSemaphoreGive(handle->data_mutex);
    }
}

/**
 * 数据接收回调
 */
static int on_data_received(uint16_t conn_handle,
                           const struct ble_gatt_error *error,
                           struct ble_gatt_attr *attr,
                           void *arg) {
    if (!g_ble_imu_handle || error->status != 0) {
        return 0;
    }

    // 将数据复制到临时缓存
    uint8_t data[attr->om->om_len];
    ble_hs_mbuf_to_flat(attr->om, data, attr->om->om_len, NULL);
    
    // 逐字节处理数据包
    for (int i = 0; i < attr->om->om_len; i++) {
        g_ble_imu_handle->temp_bytes[g_ble_imu_handle->temp_bytes_len++] = data[i];
        
        // 检查数据包头
        if (g_ble_imu_handle->temp_bytes_len == 1 && g_ble_imu_handle->temp_bytes[0] != 0x55) {
            g_ble_imu_handle->temp_bytes_len = 0;
            continue;
        }
        
        // 检查数据类型
        if (g_ble_imu_handle->temp_bytes_len == 2 && 
            (g_ble_imu_handle->temp_bytes[1] != 0x61 && g_ble_imu_handle->temp_bytes[1] != 0x71)) {
            g_ble_imu_handle->temp_bytes_len = 0;
            continue;
        }
        
        // 处理完整数据包
        if (g_ble_imu_handle->temp_bytes_len == 20) {
            process_imu_data(g_ble_imu_handle, g_ble_imu_handle->temp_bytes, g_ble_imu_handle->temp_bytes_len);
            g_ble_imu_handle->temp_bytes_len = 0;
        }
    }
    
    return 0;
}

/**
 * 发送读取寄存器命令
 */
static int send_read_reg_cmd(uint8_t reg_addr) {
    if (!g_ble_imu_handle || !g_ble_imu_handle->is_connected || g_ble_imu_handle->write_char_handle == 0) {
        return -1;
    }
    
    uint8_t cmd[5] = {0xff, 0xaa, 0x27, reg_addr, 0x00};
    
    int rc = ble_gattc_write_no_rsp_flat(g_ble_imu_handle->conn_handle, 
                                        g_ble_imu_handle->write_char_handle, 
                                        cmd, sizeof(cmd));
    if (rc != 0) {
        ESP_LOGE(TAG, "发送读取命令失败: %d", rc);
        return -1;
    }
    
    return 0;
}

/**
 * 定期读取IMU数据任务
 */
static void imu_read_task(void *param) {
    ESP_LOGI(TAG, "IMU数据读取任务启动");
    
    while (true) {
        if (g_ble_imu_handle && g_ble_imu_handle->is_connected) {
            // 读取磁场数据 (寄存器0x3A)
            send_read_reg_cmd(0x3A);
            vTaskDelay(pdMS_TO_TICKS(100));
            
            // 读取四元数数据 (寄存器0x51)  
            send_read_reg_cmd(0x51);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/**
 * 服务发现完成回调
 */
static void on_service_discovery_complete(const struct peer *peer, int status, void *arg) {
    const struct peer_svc *svc;
    const struct peer_chr *chr;
    
    if (!g_ble_imu_handle || status != 0) {
        ESP_LOGE(TAG, "服务发现失败: %d", status);
        if (g_ble_imu_handle) {
            ble_gap_terminate(g_ble_imu_handle->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        return;
    }
    
    ESP_LOGI(TAG, "服务发现完成");
    
    // 查找IMU服务
    svc = peer_svc_find_uuid(peer, &imu_service_uuid.u);
    if (svc == NULL) {
        ESP_LOGE(TAG, "未找到IMU服务");
        ble_gap_terminate(g_ble_imu_handle->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }
    
    ESP_LOGI(TAG, "找到IMU服务");
    
    // 查找写特征值
    chr = peer_chr_find_uuid(peer, &imu_service_uuid.u, &imu_write_char_uuid.u);
    if (chr != NULL) {
        g_ble_imu_handle->write_char_handle = chr->chr.val_handle;
        ESP_LOGI(TAG, "找到写特征值，句柄: %d", g_ble_imu_handle->write_char_handle);
    }
    
    // 查找读特征值（通知）
    chr = peer_chr_find_uuid(peer, &imu_service_uuid.u, &imu_read_char_uuid.u);
    if (chr != NULL) {
        g_ble_imu_handle->read_char_handle = chr->chr.val_handle;
        ESP_LOGI(TAG, "找到读特征值，句柄: %d", g_ble_imu_handle->read_char_handle);
        
        // 启用通知
        const struct peer_dsc *dsc;
        uint16_t notify_val = 1;
        int rc = -1;
        
        SLIST_FOREACH(dsc, &chr->dscs, next) {
            if (ble_uuid_cmp(&dsc->dsc.uuid.u, BLE_UUID16_DECLARE(0x2902)) == 0) {
                rc = ble_gattc_write_flat(g_ble_imu_handle->conn_handle, dsc->dsc.handle, 
                                         &notify_val, sizeof(notify_val), NULL, NULL);
                break;
            }
        }
        
        if (rc != 0) {
            ESP_LOGE(TAG, "启用通知失败: %d", rc);
        } else {
            ESP_LOGI(TAG, "通知已启用");
        }
    }
    
    if (g_ble_imu_handle->write_char_handle != 0 && g_ble_imu_handle->read_char_handle != 0) {
        ESP_LOGI(TAG, "IMU特征值配置完成，开始数据读取");
        // 启动数据读取任务
        xTaskCreate(imu_read_task, "imu_read_task", 4096, NULL, 5, NULL);
    } else {
        ESP_LOGE(TAG, "IMU特征值配置失败");
        ble_gap_terminate(g_ble_imu_handle->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
}

/**
 * 检查设备地址是否匹配
 */
static bool is_target_device(const ble_addr_t *addr) {
    return memcmp(addr->val, target_addr, 6) == 0;
}

/**
 * GAP事件处理
 */
static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    struct ble_gap_conn_desc desc;
    int rc;
    
    if (!g_ble_imu_handle) {
        return 0;
    }
    
    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        if (is_target_device(&event->disc.addr)) {
            ESP_LOGI(TAG, "发现目标IMU设备: %s", format_addr_str(event->disc.addr.val));
            ESP_LOGI(TAG, "信号强度: %d dBm", event->disc.rssi);
            
            ble_gap_disc_cancel();
            
            uint8_t own_addr_type;
            rc = ble_hs_id_infer_auto(0, &own_addr_type);
            if (rc != 0) {
                ESP_LOGE(TAG, "地址类型推断失败: %d", rc);
                return 0;
            }
            
            ESP_LOGI(TAG, "正在连接IMU设备...");
            rc = ble_gap_connect(own_addr_type, &event->disc.addr, 30000, NULL, ble_gap_event, NULL);
            if (rc != 0) {
                ESP_LOGE(TAG, "连接失败: %d", rc);
            }
        }
        return 0;
        
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ESP_LOGI(TAG, "IMU设备连接成功！");
            
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            if (rc != 0) {
                ESP_LOGE(TAG, "获取连接信息失败: %d", rc);
                return 0;
            }
            
            g_ble_imu_handle->is_connected = true;
            g_ble_imu_handle->conn_handle = event->connect.conn_handle;
            
            ESP_LOGI(TAG, "连接句柄: %d", g_ble_imu_handle->conn_handle);
            ESP_LOGI(TAG, "开始服务发现...");
            
            rc = peer_add(g_ble_imu_handle->conn_handle);
            if (rc != 0) {
                ESP_LOGE(TAG, "添加peer失败: %d", rc);
                return 0;
            }
            
            rc = peer_disc_all(g_ble_imu_handle->conn_handle, on_service_discovery_complete, NULL);
            if (rc != 0) {
                ESP_LOGE(TAG, "服务发现启动失败: %d", rc);
                return 0;
            }
            
        } else {
            ESP_LOGE(TAG, "连接失败，状态: %d", event->connect.status);
            ble_imu_scan();
        }
        return 0;
        
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "IMU设备已断开连接，原因: %d", event->disconnect.reason);
        g_ble_imu_handle->is_connected = false;
        g_ble_imu_handle->conn_handle = 0;
        g_ble_imu_handle->write_char_handle = 0;
        g_ble_imu_handle->read_char_handle = 0;
        
        // 清理数据有效性
        if (xSemaphoreTake(g_ble_imu_handle->data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            g_ble_imu_handle->sensor_data.is_valid = false;
            xSemaphoreGive(g_ble_imu_handle->data_mutex);
        }
        
        peer_delete(event->disconnect.conn.conn_handle);
        
        ESP_LOGI(TAG, "5秒后重新扫描...");
        vTaskDelay(pdMS_TO_TICKS(5000));
        ble_imu_scan();
        return 0;
        
    case BLE_GAP_EVENT_DISC_COMPLETE:
        if (!g_ble_imu_handle->is_connected) {
            ESP_LOGI(TAG, "扫描完成，未发现目标设备，重新扫描...");
            vTaskDelay(pdMS_TO_TICKS(2000));
            ble_imu_scan();
        }
        return 0;
        
    case BLE_GAP_EVENT_NOTIFY_RX:
        return on_data_received(event->notify_rx.conn_handle, 
                               &(struct ble_gatt_error){.status = 0}, 
                               &(struct ble_gatt_attr){
                                   .handle = event->notify_rx.attr_handle,
                                   .om = event->notify_rx.om
                               }, 
                               NULL);
        
    default:
        return 0;
    }
}

/**
 * 开始扫描IMU设备
 */
static void ble_imu_scan(void) {
    if (!g_ble_imu_handle) {
        return;
    }
    
    uint8_t own_addr_type;
    struct ble_gap_disc_params disc_params;
    int rc;

    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "地址类型确定失败: %d", rc);
        return;
    }

    memset(&disc_params, 0, sizeof(disc_params));
    disc_params.filter_duplicates = 1;
    disc_params.passive = 1;
    disc_params.itvl = 0;
    disc_params.window = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    ESP_LOGI(TAG, "开始扫描目标IMU设备: %s", format_addr_str(target_addr));
    
    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "扫描启动失败: %d", rc);
    }
}

/**
 * 蓝牙主机同步回调
 */
static void ble_on_sync(void) {
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "地址设置失败: %d", rc);
        return;
    }

    ble_imu_scan();
}

/**
 * 蓝牙主机重置回调
 */
static void ble_on_reset(int reason) {
    ESP_LOGE(TAG, "蓝牙主机重置: %d", reason);
}

/**
 * 蓝牙主机任务
 */
static void ble_host_task(void *param) {
    ESP_LOGI(TAG, "BLE IMU连接器启动");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// =============================================================================
// 公共接口实现
// =============================================================================

ble_imu_handle_t* ble_imu_init(void) {
    if (g_ble_imu_handle != NULL) {
        ESP_LOGW(TAG, "BLE IMU已经初始化");
        return g_ble_imu_handle;
    }
    
    // 分配句柄内存
    g_ble_imu_handle = (ble_imu_handle_t*)malloc(sizeof(ble_imu_handle_t));
    if (!g_ble_imu_handle) {
        ESP_LOGE(TAG, "内存分配失败");
        return NULL;
    }
    
    // 初始化句柄
    memset(g_ble_imu_handle, 0, sizeof(ble_imu_handle_t));
    
    // 创建互斥锁
    g_ble_imu_handle->data_mutex = xSemaphoreCreateMutex();
    if (!g_ble_imu_handle->data_mutex) {
        ESP_LOGE(TAG, "互斥锁创建失败");
        free(g_ble_imu_handle);
        g_ble_imu_handle = NULL;
        return NULL;
    }
    
    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS初始化失败: %d", ret);
        vSemaphoreDelete(g_ble_imu_handle->data_mutex);
        free(g_ble_imu_handle);
        g_ble_imu_handle = NULL;
        return NULL;
    }
    
    // 初始化NimBLE
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NimBLE初始化失败: %d", ret);
        vSemaphoreDelete(g_ble_imu_handle->data_mutex);
        free(g_ble_imu_handle);
        g_ble_imu_handle = NULL;
        return NULL;
    }
    
    // 配置主机
    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    // 初始化peer管理
    int rc = peer_init(MYNEWT_VAL(BLE_MAX_CONNECTIONS), 64, 64, 64);
    if (rc != 0) {
        ESP_LOGE(TAG, "peer初始化失败: %d", rc);
        vSemaphoreDelete(g_ble_imu_handle->data_mutex);
        free(g_ble_imu_handle);
        g_ble_imu_handle = NULL;
        return NULL;
    }

    // 设置设备名称
    rc = ble_svc_gap_device_name_set("ESP32S3-Balance");
    if (rc != 0) {
        ESP_LOGE(TAG, "设备名称设置失败: %d", rc);
    }

    // 配置存储
    ble_store_config_init();

    // 启动NimBLE主机任务
    nimble_port_freertos_init(ble_host_task);
    
    g_ble_imu_handle->initialized = true;
    ESP_LOGI(TAG, "BLE IMU模块初始化成功");
    
    return g_ble_imu_handle;
}

void ble_imu_destroy(ble_imu_handle_t* handle) {
    if (handle != g_ble_imu_handle || !handle) {
        return;
    }
    
    // 断开连接
    if (handle->is_connected) {
        ble_gap_terminate(handle->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    
    // 销毁互斥锁
    if (handle->data_mutex) {
        vSemaphoreDelete(handle->data_mutex);
    }
    
    // 释放内存
    free(handle);
    g_ble_imu_handle = NULL;
    
    ESP_LOGI(TAG, "BLE IMU模块已销毁");
}

bool ble_imu_get_data(ble_imu_handle_t* handle, ble_imu_data_t* data) {
    if (!handle || !data || !handle->initialized) {
        return false;
    }
    
    if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        *data = handle->sensor_data;
        xSemaphoreGive(handle->data_mutex);
        return handle->sensor_data.is_valid;
    }
    
    return false;
}

float ble_imu_get_roll(ble_imu_handle_t* handle) {
    if (!handle || !handle->initialized) {
        return 0.0f;
    }
    
    if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        float roll = handle->sensor_data.roll;
        xSemaphoreGive(handle->data_mutex);
        return roll;
    }
    
    return 0.0f;
}

float ble_imu_get_pitch(ble_imu_handle_t* handle) {
    if (!handle || !handle->initialized) {
        return 0.0f;
    }
    
    if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        float pitch = handle->sensor_data.pitch;
        xSemaphoreGive(handle->data_mutex);
        return pitch;
    }
    
    return 0.0f;
}

float ble_imu_get_yaw(ble_imu_handle_t* handle) {
    if (!handle || !handle->initialized) {
        return 0.0f;
    }
    
    if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        float yaw = handle->sensor_data.yaw;
        xSemaphoreGive(handle->data_mutex);
        return yaw;
    }
    
    return 0.0f;
}

bool ble_imu_is_connected(ble_imu_handle_t* handle) {
    return handle && handle->initialized && handle->is_connected;
}

uint32_t ble_imu_get_bytes_received(ble_imu_handle_t* handle) {
    if (!handle || !handle->initialized) {
        return 0;
    }
    
    if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        uint32_t bytes = handle->total_bytes_received;
        xSemaphoreGive(handle->data_mutex);
        return bytes;
    }
    
    return 0;
}