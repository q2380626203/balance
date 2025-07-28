#include "ky9250.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char* TAG = "KY9250";

// 解析单个数据包
static bool parse_ky9250_packet(const uint8_t* ubuf, ky9250_data_t* data) {
    if (!ubuf || !data) return false;
    
    // 检查包头
    if (ubuf[0] != 0x50) return false;
    
    // 检查校验位
    float checksum = (ubuf[49] * 256.0 * 256.0 + ubuf[50] * 256.0 + ubuf[51] - 1000000) * 0.001;
    if (checksum != 128.0f) return false;
    
    // 解析数据（直接移植自Arduino代码）
    float a[16];
    a[0] = (ubuf[1] * 256.0 * 256.0 + ubuf[2] * 256.0 + ubuf[3] - 1000000) * 0.001;   // gx
    a[1] = (ubuf[4] * 256.0 * 256.0 + ubuf[5] * 256.0 + ubuf[6] - 1000000) * 0.001;   // gy
    a[2] = (ubuf[7] * 256.0 * 256.0 + ubuf[8] * 256.0 + ubuf[9] - 1000000) * 0.001;   // gz
    
    a[3] = (ubuf[10] * 256.0 * 256.0 + ubuf[11] * 256.0 + ubuf[12] - 1000000) * 0.001; // ax
    a[4] = (ubuf[13] * 256.0 * 256.0 + ubuf[14] * 256.0 + ubuf[15] - 1000000) * 0.001; // ay
    a[5] = (ubuf[16] * 256.0 * 256.0 + ubuf[17] * 256.0 + ubuf[18] - 1000000) * 0.001; // az
    
    a[6] = (ubuf[19] * 256.0 * 256.0 + ubuf[20] * 256.0 + ubuf[21] - 1000000) * 0.001; // mx
    a[7] = (ubuf[22] * 256.0 * 256.0 + ubuf[23] * 256.0 + ubuf[24] - 1000000) * 0.001; // my
    a[8] = (ubuf[25] * 256.0 * 256.0 + ubuf[26] * 256.0 + ubuf[27] - 1000000) * 0.001; // mz
    
    a[9]  = (ubuf[28] * 256.0 * 256.0 + ubuf[29] * 256.0 + ubuf[30] - 1000000) * 0.001; // roll
    a[10] = (ubuf[31] * 256.0 * 256.0 + ubuf[32] * 256.0 + ubuf[33] - 1000000) * 0.001; // pitch
    a[11] = (ubuf[34] * 256.0 * 256.0 + ubuf[35] * 256.0 + ubuf[36] - 1000000) * 0.001; // yaw
    
    a[12] = (ubuf[37] * 256.0 * 256.0 + ubuf[38] * 256.0 + ubuf[39] - 1000000) * 0.001; // q0
    a[13] = (ubuf[40] * 256.0 * 256.0 + ubuf[41] * 256.0 + ubuf[42] - 1000000) * 0.001; // q1
    a[14] = (ubuf[43] * 256.0 * 256.0 + ubuf[44] * 256.0 + ubuf[45] - 1000000) * 0.001; // q2
    a[15] = (ubuf[46] * 256.0 * 256.0 + ubuf[47] * 256.0 + ubuf[48] - 1000000) * 0.001; // q3
    
    // 填充数据结构
    data->gx = a[0]; data->gy = a[1]; data->gz = a[2];
    data->ax = a[3]; data->ay = a[4]; data->az = a[5];
    data->mx = a[6]; data->my = a[7]; data->mz = a[8];
    data->roll = a[9]; data->pitch = a[10]; data->yaw = a[11];
    data->q0 = a[12]; data->q1 = a[13]; data->q2 = a[14]; data->q3 = a[15];
    data->is_valid = true;
    data->timestamp = xTaskGetTickCount();
    
    return true;
}

// 数据解析任务
void ky9250_parse_task(void* pvParameters) {
    ky9250_handle_t* handle = (ky9250_handle_t*)pvParameters;
    if (!handle) {
        ESP_LOGE(TAG, "Invalid handle in parse task");
        vTaskDelete(NULL);
        return;
    }
    
    uint8_t ubuf[52];
    uint8_t ucRxCnt = 0;
    uint8_t temp_byte;
    
    ESP_LOGI(TAG, "KY9250 parse task started");
    
    while (1) {
        // 读取UART数据
        int len = uart_read_bytes(handle->uart_port, &temp_byte, 1, 10 / portTICK_PERIOD_MS);
        if (len > 0) {
            handle->total_bytes_received++;
            
            ubuf[ucRxCnt++] = temp_byte;
            
            // 检查包头
            if (ubuf[0] != 0x50) {
                ucRxCnt = 0;
                continue;
            }
            
            // 等待完整包
            if (ucRxCnt < 52) continue;
            
            // 解析数据包
            ky9250_data_t temp_data = {0};
            if (parse_ky9250_packet(ubuf, &temp_data)) {
                // 更新共享数据（线程安全）
                if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    handle->sensor_data = temp_data;
                    handle->sensor_data.packet_count++;
                    xSemaphoreGive(handle->data_mutex);
                }
            }
            
            ucRxCnt = 0;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

ky9250_handle_t* ky9250_init(uart_port_t uart_port, int rxd_pin, int txd_pin, int baud_rate) {
    ky9250_handle_t* handle = malloc(sizeof(ky9250_handle_t));
    if (!handle) {
        ESP_LOGE(TAG, "Failed to allocate memory for handle");
        return NULL;
    }
    
    memset(handle, 0, sizeof(ky9250_handle_t));
    handle->uart_port = uart_port;
    handle->rxd_pin = rxd_pin;
    handle->txd_pin = txd_pin;
    handle->baud_rate = baud_rate;
    
    // 创建互斥锁
    handle->data_mutex = xSemaphoreCreateMutex();
    if (!handle->data_mutex) {
        ESP_LOGE(TAG, "Failed to create data mutex");
        free(handle);
        return NULL;
    }
    
    // 配置UART
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    esp_err_t ret = uart_param_config(uart_port, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
        vSemaphoreDelete(handle->data_mutex);
        free(handle);
        return NULL;
    }
    
    ret = uart_set_pin(uart_port, txd_pin, rxd_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
        vSemaphoreDelete(handle->data_mutex);
        free(handle);
        return NULL;
    }
    
    ret = uart_driver_install(uart_port, 1024, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        vSemaphoreDelete(handle->data_mutex);
        free(handle);
        return NULL;
    }
    
    // 启动解析任务
    BaseType_t task_ret = xTaskCreate(ky9250_parse_task, "ky9250_parse", 4096, handle, 4, NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create parse task");
        uart_driver_delete(uart_port);
        vSemaphoreDelete(handle->data_mutex);
        free(handle);
        return NULL;
    }
    
    handle->initialized = true;
    ESP_LOGI(TAG, "KY9250 initialized successfully (UART%d, RX=%d, TX=%d, %d bps)", 
             uart_port, rxd_pin, txd_pin, baud_rate);
    
    return handle;
}

void ky9250_destroy(ky9250_handle_t* handle) {
    if (!handle) return;
    
    handle->initialized = false;
    uart_driver_delete(handle->uart_port);
    vSemaphoreDelete(handle->data_mutex);
    free(handle);
    
    ESP_LOGI(TAG, "KY9250 destroyed");
}

bool ky9250_get_data(ky9250_handle_t* handle, ky9250_data_t* data) {
    if (!handle || !data || !handle->initialized) return false;
    
    if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        *data = handle->sensor_data;
        xSemaphoreGive(handle->data_mutex);
        return data->is_valid;
    }
    
    return false;
}

float ky9250_get_pitch(ky9250_handle_t* handle) {
    if (!handle || !handle->initialized) return 0.0f;
    
    float pitch = 0.0f;
    if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        pitch = handle->sensor_data.pitch;
        xSemaphoreGive(handle->data_mutex);
    }
    
    return pitch;
}

float ky9250_get_roll(ky9250_handle_t* handle) {
    if (!handle || !handle->initialized) return 0.0f;
    
    float roll = 0.0f;
    if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        roll = handle->sensor_data.roll;
        xSemaphoreGive(handle->data_mutex);
    }
    
    return roll;
}

float ky9250_get_yaw(ky9250_handle_t* handle) {
    if (!handle || !handle->initialized) return 0.0f;
    
    float yaw = 0.0f;
    if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        yaw = handle->sensor_data.yaw;
        xSemaphoreGive(handle->data_mutex);
    }
    
    return yaw;
}

uint32_t ky9250_get_bytes_received(ky9250_handle_t* handle) {
    return handle ? handle->total_bytes_received : 0;
}

uint32_t ky9250_get_packet_count(ky9250_handle_t* handle) {
    if (!handle || !handle->initialized) return 0;
    
    uint32_t count = 0;
    if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        count = handle->sensor_data.packet_count;
        xSemaphoreGive(handle->data_mutex);
    }
    
    return count;
}