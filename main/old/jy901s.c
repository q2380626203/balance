#include "jy901s.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char* TAG = "JY901S";

// 校验和计算
static uint8_t calculate_checksum(const uint8_t* data, int len) {
    uint8_t sum = 0;
    for (int i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

// 解析JY901S数据包
static bool parse_jy901s_packet(const uint8_t* packet, jy901s_data_t* data) {
    if (!packet || !data) return false;
    
    // 检查包头
    if (packet[0] != JY901S_PACKET_HEADER1 || packet[1] != JY901S_PACKET_HEADER2) {
        return false;
    }
    
    // 计算校验和
    uint8_t expected_sum = calculate_checksum(packet, JY901S_PACKET_LENGTH - 1);
    if (packet[JY901S_PACKET_LENGTH - 1] != expected_sum) {
        ESP_LOGW(TAG, "Checksum mismatch: expected 0x%02X, got 0x%02X", 
                 expected_sum, packet[JY901S_PACKET_LENGTH - 1]);
        return false;
    }
    
    // 解析角度数据
    int16_t roll_raw = (packet[3] << 8) | packet[2];
    int16_t pitch_raw = (packet[5] << 8) | packet[4];
    int16_t yaw_raw = (packet[7] << 8) | packet[6];
    
    // 转换为角度值 角度=原始值/32768*180
    data->roll = ((float)roll_raw) / 32768.0f * 180.0f;
    data->pitch = ((float)pitch_raw) / 32768.0f * 180.0f;
    data->yaw = ((float)yaw_raw) / 32768.0f * 180.0f;
    
    // 版本号
    data->version = (packet[9] << 8) | packet[8];
    
    data->is_valid = true;
    data->timestamp = xTaskGetTickCount();
    
    return true;
}

// 数据解析任务
void jy901s_parse_task(void* pvParameters) {
    jy901s_handle_t* handle = (jy901s_handle_t*)pvParameters;
    if (!handle) {
        ESP_LOGE(TAG, "Invalid handle in parse task");
        vTaskDelete(NULL);
        return;
    }
    
    uint8_t packet_buffer[JY901S_PACKET_LENGTH];
    uint8_t buffer_index = 0;
    uint8_t temp_byte;
    
    ESP_LOGI(TAG, "JY901S parse task started");
    
    while (1) {
        // 读取UART数据
        int len = uart_read_bytes(handle->uart_port, &temp_byte, 1, 10 / portTICK_PERIOD_MS);
        if (len > 0) {
            handle->total_bytes_received++;
            
            // 状态机解析数据包
            if (buffer_index == 0) {
                // 寻找第一个包头
                if (temp_byte == JY901S_PACKET_HEADER1) {
                    packet_buffer[buffer_index++] = temp_byte;
                }
            } else if (buffer_index == 1) {
                // 检查第二个包头
                if (temp_byte == JY901S_PACKET_HEADER2) {
                    packet_buffer[buffer_index++] = temp_byte;
                } else {
                    // 重新开始
                    buffer_index = 0;
                    if (temp_byte == JY901S_PACKET_HEADER1) {
                        packet_buffer[buffer_index++] = temp_byte;
                    }
                }
            } else {
                // 收集数据
                packet_buffer[buffer_index++] = temp_byte;
                
                // 检查是否收集完整包
                if (buffer_index >= JY901S_PACKET_LENGTH) {
                    // 解析数据包
                    jy901s_data_t temp_data = {0};
                    if (parse_jy901s_packet(packet_buffer, &temp_data)) {
                        // 更新共享数据（线程安全）
                        if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                            handle->sensor_data = temp_data;
                            handle->sensor_data.packet_count++;
                            xSemaphoreGive(handle->data_mutex);
                        }
                    }
                    
                    // 重置缓冲区
                    buffer_index = 0;
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

jy901s_handle_t* jy901s_init(uart_port_t uart_port, int rxd_pin, int txd_pin, int baud_rate) {
    jy901s_handle_t* handle = malloc(sizeof(jy901s_handle_t));
    if (!handle) {
        ESP_LOGE(TAG, "Failed to allocate memory for handle");
        return NULL;
    }
    
    memset(handle, 0, sizeof(jy901s_handle_t));
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
    BaseType_t task_ret = xTaskCreate(jy901s_parse_task, "jy901s_parse", 4096, handle, 4, NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create parse task");
        uart_driver_delete(uart_port);
        vSemaphoreDelete(handle->data_mutex);
        free(handle);
        return NULL;
    }
    
    handle->initialized = true;
    ESP_LOGI(TAG, "JY901S initialized successfully (UART%d, RX=%d, TX=%d, %d bps)", 
             uart_port, rxd_pin, txd_pin, baud_rate);
    
    return handle;
}

void jy901s_destroy(jy901s_handle_t* handle) {
    if (!handle) return;
    
    handle->initialized = false;
    uart_driver_delete(handle->uart_port);
    vSemaphoreDelete(handle->data_mutex);
    free(handle);
    
    ESP_LOGI(TAG, "JY901S destroyed");
}

bool jy901s_get_data(jy901s_handle_t* handle, jy901s_data_t* data) {
    if (!handle || !data || !handle->initialized) return false;
    
    if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        *data = handle->sensor_data;
        xSemaphoreGive(handle->data_mutex);
        return data->is_valid;
    }
    
    return false;
}

float jy901s_get_pitch(jy901s_handle_t* handle) {
    if (!handle || !handle->initialized) return 0.0f;
    
    float pitch = 0.0f;
    if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        pitch = handle->sensor_data.pitch;
        xSemaphoreGive(handle->data_mutex);
    }
    
    return pitch;
}

float jy901s_get_roll(jy901s_handle_t* handle) {
    if (!handle || !handle->initialized) return 0.0f;
    
    float roll = 0.0f;
    if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        roll = handle->sensor_data.roll;
        xSemaphoreGive(handle->data_mutex);
    }
    
    return roll;
}

float jy901s_get_yaw(jy901s_handle_t* handle) {
    if (!handle || !handle->initialized) return 0.0f;
    
    float yaw = 0.0f;
    if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        yaw = handle->sensor_data.yaw;
        xSemaphoreGive(handle->data_mutex);
    }
    
    return yaw;
}

uint32_t jy901s_get_bytes_received(jy901s_handle_t* handle) {
    return handle ? handle->total_bytes_received : 0;
}

uint32_t jy901s_get_packet_count(jy901s_handle_t* handle) {
    if (!handle || !handle->initialized) return 0;
    
    uint32_t count = 0;
    if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        count = handle->sensor_data.packet_count;
        xSemaphoreGive(handle->data_mutex);
    }
    
    return count;
}