#include "gy25t.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/queue.h"

// 配置
#define DATA_QUEUE_SIZE         50          // YAW角数据队列大小
#define PARSER_BUFFER_SIZE      128         // 解析任务内部的静态缓冲区大小
static const char *TAG = "GY25T_YAW";

// 静态函数声明
static void apply_low_pass_filter(gy25t_handle_t* handle, float raw_yaw, float* filtered_yaw);
static void gy25t_event_task(void *pvParameters);
static bool gy25t_parse_and_send(gy25t_handle_t* handle, uint8_t* data, int len);
static uint8_t gy25t_calculate_checksum(uint8_t *data, int length);

// ====================================================================================
// --- GY-25T YAW角模块实现 ---
// ====================================================================================

gy25t_handle_t* gy25t_init(const gy25t_config_t* config) {
    if (!config) {
        ESP_LOGE(TAG, "配置参数为空！");
        return NULL;
    }

    gy25t_handle_t* handle = (gy25t_handle_t*)calloc(1, sizeof(gy25t_handle_t));
    if (!handle) {
        ESP_LOGE(TAG, "句柄内存分配失败！");
        return NULL;
    }

    memcpy(&handle->config, config, sizeof(gy25t_config_t));
    handle->filter.first_read = true;

    // 创建消息队列用于在解析任务和主任务之间传递YAW角数据
    handle->data_queue = xQueueCreate(DATA_QUEUE_SIZE, sizeof(float));
    if (handle->data_queue == NULL) {
        ESP_LOGE(TAG, "数据消息队列创建失败！");
        free(handle);
        return NULL;
    }

    // 初始化UART
    uart_config_t uart_config = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // 安装UART驱动
    ESP_ERROR_CHECK(uart_driver_install(config->uart_port, config->buf_size, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(config->uart_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(config->uart_port, config->txd_pin, config->rxd_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // 创建事件处理任务
    if (xTaskCreate(gy25t_event_task, "gy25t_event_task", 4096, handle, 5, &handle->read_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "GY-25T事件任务创建失败！");
        vQueueDelete(handle->data_queue);
        uart_driver_delete(config->uart_port);
        free(handle);
        return NULL;
    }

    ESP_LOGI(TAG, "GY-25T YAW角模块在 UART%d 上初始化完成 (事件驱动模式)", config->uart_port);
    return handle;
}

void gy25t_deinit(gy25t_handle_t* handle) {
    if (!handle) return;
    if (handle->read_task_handle) vTaskDelete(handle->read_task_handle);
    if (handle->data_queue) vQueueDelete(handle->data_queue);
    uart_driver_delete(handle->config.uart_port);
    free(handle);
    ESP_LOGI(TAG, "GY-25T YAW角模块已销毁");
}

// 该函数现在由主循环直接管理，此处保留API兼容性
float gy25t_get_filtered_yaw(gy25t_handle_t* handle) {
    return handle ? handle->last_yaw : 0.0f;
}

void gy25t_calibrate_zero_position(gy25t_handle_t* handle) {
    if (!handle) return;
    handle->filter.zero_angle = handle->last_yaw;
    ESP_LOGI(TAG, "GY-25T YAW角零位校准完成，零点角度: %.2f°", handle->filter.zero_angle);
}

void gy25t_set_output_rate(gy25t_handle_t* handle, gy25t_output_rate_t rate) {
    if (!handle) return;
    uint8_t command[4];
    command[0] = 0xA5;
    command[1] = 0x04;
    command[2] = (uint8_t)rate;
    command[3] = (command[0] + command[1] + command[2]) & 0xFF;
    uart_write_bytes(handle->config.uart_port, command, sizeof(command));
    ESP_LOGI(TAG, "发送频率设置指令: %02X %02X %02X %02X", command[0], command[1], command[2], command[3]);
}

// ====================================================================================
// --- 静态函数和事件驱动任务 ---
// ====================================================================================

static void apply_low_pass_filter(gy25t_handle_t* handle, float raw_yaw, float* filtered_yaw) {
    const float ALPHA_FILTER = 0.1f;
    if (handle->filter.first_read) {
        *filtered_yaw = raw_yaw;
        handle->filter.first_read = false;
    } else {
        *filtered_yaw = ALPHA_FILTER * raw_yaw + (1.0f - ALPHA_FILTER) * (*filtered_yaw);
    }
}

static void gy25t_event_task(void *pvParameters) {
    gy25t_handle_t* handle = (gy25t_handle_t*)pvParameters;
    uint8_t* dtmp = (uint8_t*)malloc(PARSER_BUFFER_SIZE);
    if (!dtmp) {
        ESP_LOGE(TAG, "解析缓冲区内存分配失败！");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "GY-25T事件任务已启动");

    while (1) {
        int len = uart_read_bytes(handle->config.uart_port, dtmp, PARSER_BUFFER_SIZE, pdMS_TO_TICKS(10));
        if (len > 0) {
            gy25t_parse_and_send(handle, dtmp, len);
        }
    }

    free(dtmp);
    vTaskDelete(NULL);
}

static bool gy25t_parse_and_send(gy25t_handle_t* handle, uint8_t* data, int len) {
    static uint8_t parse_buf[PARSER_BUFFER_SIZE];
    static int parse_len = 0;
    bool processed = false;

    if (parse_len + len > PARSER_BUFFER_SIZE) {
        parse_len = 0;
    }
    memcpy(parse_buf + parse_len, data, len);
    parse_len += len;

    while (parse_len >= GY25T_YAW_PACKET_SIZE) {
        int frame_offset = -1;
        for (int i = 0; i <= parse_len - GY25T_YAW_PACKET_SIZE; i++) {
            if (parse_buf[i] == 0xA4 && parse_buf[i+1] == 0x03 && parse_buf[i+2] == 0x18 && parse_buf[i+3] == 0x02) {
                frame_offset = i;
                break;
            }
        }

        if (frame_offset != -1) {
            uint8_t* packet = parse_buf + frame_offset;
            uint8_t checksum = gy25t_calculate_checksum(packet, 6);

            if (checksum == packet[6]) {
                int16_t raw_yaw_value = (int16_t)((packet[4] << 8) | packet[5]);
                float calculated_yaw = (float)raw_yaw_value / 100.0f;
                
                float filtered_yaw = handle->last_yaw;
                apply_low_pass_filter(handle, calculated_yaw, &filtered_yaw);

                // 正确的“滑动窗口”队列管理
                if (xQueueSend(handle->data_queue, &filtered_yaw, 0) != pdTRUE) {
                    float dummy;
                    xQueueReceive(handle->data_queue, &dummy, 0); // 队列满，移除最旧的
                    xQueueSend(handle->data_queue, &filtered_yaw, 0); // 发送最新的
                }
                handle->packets_received++;
                processed = true;
            } else {
                handle->packets_invalid++;
            }

            parse_len -= (frame_offset + GY25T_YAW_PACKET_SIZE);
            memmove(parse_buf, packet + GY25T_YAW_PACKET_SIZE, parse_len);
        } else {
            if (parse_len > GY25T_YAW_PACKET_SIZE) {
                memmove(parse_buf, parse_buf + parse_len - (GY25T_YAW_PACKET_SIZE - 1), GY25T_YAW_PACKET_SIZE - 1);
                parse_len = GY25T_YAW_PACKET_SIZE - 1;
            }
            break;
        }
    }
    return processed;
}

static uint8_t gy25t_calculate_checksum(uint8_t *data, int length) {
    uint16_t sum = 0;
    for (int i = 0; i < length; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

// ====================================================================================
// --- 附加功能接口实现 ---
// ====================================================================================

void gy25t_get_stats(gy25t_handle_t* handle, gy25t_stats_t* stats) {
    if (!handle || !stats) return;
    stats->packets_received = handle->packets_received;
    stats->packets_invalid = handle->packets_invalid;
}

bool gy25t_has_new_data(gy25t_handle_t* handle) {
    if (!handle) return false;
    return (uxQueueMessagesWaiting(handle->data_queue) > 0);
}

void gy25t_clear_new_data_flag(gy25t_handle_t* handle) {
    if (!handle) return;
    xQueueReset(handle->data_queue);
}
