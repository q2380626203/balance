/*
 * BLE IMU Connector for ESP32S3
 * 连接指定IMU设备并读取传感器数据
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

// IMU设备UUID常量 (参考device_model.py)
static const ble_uuid128_t imu_service_uuid = 
    BLE_UUID128_INIT(0xfb, 0x34, 0x9a, 0x5f, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0xe5, 0xff, 0x00, 0x00);
                     
static const ble_uuid128_t imu_read_char_uuid = 
    BLE_UUID128_INIT(0xfb, 0x34, 0x9a, 0x5f, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0xe4, 0xff, 0x00, 0x00);
                     
static const ble_uuid128_t imu_write_char_uuid = 
    BLE_UUID128_INIT(0xfb, 0x34, 0x9a, 0x5f, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0xe9, 0xff, 0x00, 0x00);

// 连接状态
static bool is_connected = false;
static uint16_t conn_handle = 0;
static uint16_t write_char_handle = 0;
static uint16_t read_char_handle = 0;

// 数据缓存
static uint8_t temp_bytes[20];
static int temp_bytes_len = 0;

// IMU数据结构
typedef struct {
    float acc_x, acc_y, acc_z;      // 加速度
    float gyro_x, gyro_y, gyro_z;   // 角速度  
    float angle_x, angle_y, angle_z; // 角度
    float mag_x, mag_y, mag_z;      // 磁场
    float q0, q1, q2, q3;           // 四元数
} imu_data_t;

static imu_data_t imu_data = {0};

// 函数声明
void ble_imu_scan(void);
void ble_store_config_init(void);

// 格式化地址字符串的辅助函数
static char addr_str_buf[18];
static char* format_addr_str(const uint8_t *addr) {
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
 * 处理IMU数据包 (参考device_model.py的processData函数)
 */
static void process_imu_data(const uint8_t *data, int len) {
    if (len != 20) {
        ESP_LOGW(TAG, "数据包长度错误: %d", len);
        return;
    }

    if (data[0] != 0x55) {
        ESP_LOGW(TAG, "数据包头错误: 0x%02X", data[0]);
        return;
    }

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
        imu_data.acc_x = get_sign_int16(ax_raw) / 32768.0f * 16.0f;
        imu_data.acc_y = get_sign_int16(ay_raw) / 32768.0f * 16.0f;
        imu_data.acc_z = get_sign_int16(az_raw) / 32768.0f * 16.0f;
        
        imu_data.gyro_x = get_sign_int16(gx_raw) / 32768.0f * 2000.0f;
        imu_data.gyro_y = get_sign_int16(gy_raw) / 32768.0f * 2000.0f;
        imu_data.gyro_z = get_sign_int16(gz_raw) / 32768.0f * 2000.0f;
        
        imu_data.angle_x = get_sign_int16(angx_raw) / 32768.0f * 180.0f;
        imu_data.angle_y = get_sign_int16(angy_raw) / 32768.0f * 180.0f;
        imu_data.angle_z = get_sign_int16(angz_raw) / 32768.0f * 180.0f;

        ESP_LOGI(TAG, "========== IMU数据 ==========");
        ESP_LOGI(TAG, "加速度: X=%.3f, Y=%.3f, Z=%.3f g", 
                 imu_data.acc_x, imu_data.acc_y, imu_data.acc_z);
        ESP_LOGI(TAG, "角速度: X=%.3f, Y=%.3f, Z=%.3f °/s", 
                 imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);
        ESP_LOGI(TAG, "角度:   X=%.3f, Y=%.3f, Z=%.3f °", 
                 imu_data.angle_x, imu_data.angle_y, imu_data.angle_z);
        ESP_LOGI(TAG, "==========================\n");

    } else if (data[1] == 0x71) {
        // 扩展数据 (磁场 + 四元数)
        if (data[2] == 0x3A) {
            // 磁场数据
            int16_t hx_raw = (data[5] << 8) | data[4];
            int16_t hy_raw = (data[7] << 8) | data[6];
            int16_t hz_raw = (data[9] << 8) | data[8];
            
            imu_data.mag_x = get_sign_int16(hx_raw) / 120.0f;
            imu_data.mag_y = get_sign_int16(hy_raw) / 120.0f;
            imu_data.mag_z = get_sign_int16(hz_raw) / 120.0f;
            
            ESP_LOGI(TAG, "磁场: X=%.3f, Y=%.3f, Z=%.3f", 
                     imu_data.mag_x, imu_data.mag_y, imu_data.mag_z);
                     
        } else if (data[2] == 0x51) {
            // 四元数数据
            int16_t q0_raw = (data[5] << 8) | data[4];
            int16_t q1_raw = (data[7] << 8) | data[6];
            int16_t q2_raw = (data[9] << 8) | data[8];
            int16_t q3_raw = (data[11] << 8) | data[10];
            
            imu_data.q0 = get_sign_int16(q0_raw) / 32768.0f;
            imu_data.q1 = get_sign_int16(q1_raw) / 32768.0f;
            imu_data.q2 = get_sign_int16(q2_raw) / 32768.0f;
            imu_data.q3 = get_sign_int16(q3_raw) / 32768.0f;
            
            ESP_LOGI(TAG, "四元数: Q0=%.5f, Q1=%.5f, Q2=%.5f, Q3=%.5f", 
                     imu_data.q0, imu_data.q1, imu_data.q2, imu_data.q3);
        }
    }
}

/**
 * 数据接收回调 (参考device_model.py的onDataReceived)
 */
static int on_data_received(uint16_t conn_handle,
                           const struct ble_gatt_error *error,
                           struct ble_gatt_attr *attr,
                           void *arg) {
    if (error->status != 0) {
        ESP_LOGE(TAG, "数据接收错误: %d", error->status);
        return 0;
    }

    // 将数据复制到临时缓存
    uint8_t data[attr->om->om_len];
    ble_hs_mbuf_to_flat(attr->om, data, attr->om->om_len, NULL);
    
    // 逐字节处理数据包
    for (int i = 0; i < attr->om->om_len; i++) {
        temp_bytes[temp_bytes_len++] = data[i];
        
        // 检查数据包头
        if (temp_bytes_len == 1 && temp_bytes[0] != 0x55) {
            temp_bytes_len = 0;
            continue;
        }
        
        // 检查数据类型
        if (temp_bytes_len == 2 && (temp_bytes[1] != 0x61 && temp_bytes[1] != 0x71)) {
            temp_bytes_len = 0;
            continue;
        }
        
        // 处理完整数据包
        if (temp_bytes_len == 20) {
            process_imu_data(temp_bytes, temp_bytes_len);
            temp_bytes_len = 0;
        }
    }
    
    return 0;
}

/**
 * 发送读取寄存器命令
 */
static int send_read_reg_cmd(uint8_t reg_addr) {
    if (!is_connected || write_char_handle == 0) {
        return -1;
    }
    
    // 构造读取命令 (参考device_model.py的get_readBytes)
    uint8_t cmd[5] = {0xff, 0xaa, 0x27, reg_addr, 0x00};
    
    int rc = ble_gattc_write_no_rsp_flat(conn_handle, write_char_handle, cmd, sizeof(cmd));
    if (rc != 0) {
        ESP_LOGE(TAG, "发送读取命令失败: %d", rc);
        return -1;
    }
    
    ESP_LOGD(TAG, "发送读取寄存器命令: 0x%02X", reg_addr);
    return 0;
}

/**
 * 定期读取IMU数据任务
 */
static void imu_read_task(void *param) {
    ESP_LOGI(TAG, "IMU数据读取任务启动");
    
    while (true) {
        if (is_connected) {
            // 读取磁场数据 (寄存器0x3A)
            send_read_reg_cmd(0x3A);
            vTaskDelay(pdMS_TO_TICKS(100));
            
            // 读取四元数数据 (寄存器0x51)  
            send_read_reg_cmd(0x51);
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

/**
 * GATT服务发现完成回调
 */
static void on_service_discovery_complete(const struct peer *peer, int status, void *arg) {
    const struct peer_svc *svc;
    const struct peer_chr *chr;
    
    if (status != 0) {
        ESP_LOGE(TAG, "服务发现失败: %d", status);
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }
    
    ESP_LOGI(TAG, "服务发现完成");
    
    // 查找IMU服务
    svc = peer_svc_find_uuid(peer, &imu_service_uuid.u);
    if (svc == NULL) {
        ESP_LOGE(TAG, "未找到IMU服务");
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }
    
    ESP_LOGI(TAG, "找到IMU服务");
    
    // 查找写特征值
    chr = peer_chr_find_uuid(peer, &imu_service_uuid.u, &imu_write_char_uuid.u);
    if (chr != NULL) {
        write_char_handle = chr->chr.val_handle;
        ESP_LOGI(TAG, "找到写特征值，句柄: %d", write_char_handle);
    }
    
    // 查找读特征值（通知）
    chr = peer_chr_find_uuid(peer, &imu_service_uuid.u, &imu_read_char_uuid.u);
    if (chr != NULL) {
        read_char_handle = chr->chr.val_handle;
        ESP_LOGI(TAG, "找到读特征值，句柄: %d", read_char_handle);
        
        // 启用通知 - 查找并写入CCCD描述符
        const struct peer_dsc *dsc;
        uint16_t notify_val = 1;  // 启用通知
        int rc = -1;
        
        // 查找CCCD描述符
        SLIST_FOREACH(dsc, &chr->dscs, next) {
            if (ble_uuid_cmp(&dsc->dsc.uuid.u, BLE_UUID16_DECLARE(0x2902)) == 0) {
                // 找到CCCD，写入启用通知
                rc = ble_gattc_write_flat(conn_handle, dsc->dsc.handle, 
                                         &notify_val, sizeof(notify_val), NULL, NULL);
                break;
            }
        }
        
        if (rc != 0) {
            ESP_LOGE(TAG, "启用通知失败: %d，尝试直接读取数据", rc);
        } else {
            ESP_LOGI(TAG, "通知已启用");
        }
    }
    
    if (write_char_handle != 0 && read_char_handle != 0) {
        ESP_LOGI(TAG, "IMU特征值配置完成，开始数据读取");
        // 启动数据读取任务
        xTaskCreate(imu_read_task, "imu_read_task", 4096, NULL, 5, NULL);
    } else {
        ESP_LOGE(TAG, "IMU特征值配置失败");
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
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
    
    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        // 发现设备
        if (is_target_device(&event->disc.addr)) {
            ESP_LOGI(TAG, "发现目标IMU设备: %s", format_addr_str(event->disc.addr.val));
            ESP_LOGI(TAG, "信号强度: %d dBm", event->disc.rssi);
            
            // 停止扫描并连接
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
            // 连接成功
            ESP_LOGI(TAG, "IMU设备连接成功！");
            
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            if (rc != 0) {
                ESP_LOGE(TAG, "获取连接信息失败: %d", rc);
                return 0;
            }
            
            is_connected = true;
            conn_handle = event->connect.conn_handle;
            
            ESP_LOGI(TAG, "连接句柄: %d", conn_handle);
            ESP_LOGI(TAG, "开始服务发现...");
            
            // 添加peer并开始服务发现
            rc = peer_add(conn_handle);
            if (rc != 0) {
                ESP_LOGE(TAG, "添加peer失败: %d", rc);
                return 0;
            }
            
            rc = peer_disc_all(conn_handle, on_service_discovery_complete, NULL);
            if (rc != 0) {
                ESP_LOGE(TAG, "服务发现启动失败: %d", rc);
                return 0;
            }
            
        } else {
            ESP_LOGE(TAG, "连接失败，状态: %d", event->connect.status);
            // 重新开始扫描
            ble_imu_scan();
        }
        return 0;
        
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "IMU设备已断开连接，原因: %d", event->disconnect.reason);
        is_connected = false;
        conn_handle = 0;
        write_char_handle = 0;
        read_char_handle = 0;
        
        // 清理peer信息
        peer_delete(event->disconnect.conn.conn_handle);
        
        // 重新开始扫描
        ESP_LOGI(TAG, "5秒后重新扫描...");
        vTaskDelay(pdMS_TO_TICKS(5000));
        ble_imu_scan();
        return 0;
        
    case BLE_GAP_EVENT_DISC_COMPLETE:
        if (!is_connected) {
            ESP_LOGI(TAG, "扫描完成，未发现目标设备，重新扫描...");
            vTaskDelay(pdMS_TO_TICKS(2000));
            ble_imu_scan();
        }
        return 0;
        
    case BLE_GAP_EVENT_NOTIFY_RX:
        // 接收到通知数据
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
void ble_imu_scan(void) {
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
    int rc;

    rc = ble_hs_util_ensure_addr(0);
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
void ble_host_task(void *param) {
    ESP_LOGI(TAG, "BLE IMU连接器启动");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/**
 * 主函数
 */
void app_main(void) {
    int rc;

    ESP_LOGI(TAG, "=== ESP32S3 IMU连接器启动 ===");
    ESP_LOGI(TAG, "目标设备地址: %s", format_addr_str(target_addr));

    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS初始化完成");

    // 初始化NimBLE
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NimBLE初始化失败: %d", ret);
        return;
    }
    ESP_LOGI(TAG, "NimBLE协议栈初始化完成");

    // 配置主机
    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    // 初始化peer管理
    rc = peer_init(MYNEWT_VAL(BLE_MAX_CONNECTIONS), 64, 64, 64);
    if (rc != 0) {
        ESP_LOGE(TAG, "peer初始化失败: %d", rc);
        return;
    }

    // 设置设备名称
    rc = ble_svc_gap_device_name_set("ESP32S3-IMU");
    if (rc != 0) {
        ESP_LOGE(TAG, "设备名称设置失败: %d", rc);
    }

    // 配置存储
    ble_store_config_init();

    // 启动NimBLE主机任务
    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "IMU连接器初始化完成，开始搜索目标设备...");
}