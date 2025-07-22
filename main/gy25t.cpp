#include "gy25t.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "hal/uart_ll.h"
#include "esp_intr_alloc.h"

// 配置
#define DATA_QUEUE_SIZE         50          // YAW角数据队列大小
#define PARSER_BUFFER_SIZE      128         // 解析任务内部的静态缓冲区大小
static const char *TAG = "GY25T_YAW";

// 全局变量：用于中断处理中访问句柄
static gy25t_handle_t* g_interrupt_handle = NULL;

// 静态函数声明
static void apply_low_pass_filter(gy25t_handle_t* handle, float raw_yaw, float* filtered_yaw);
static void gy25t_parse_task(void *pvParameters);
static bool gy25t_parse_packet(gy25t_handle_t* handle, const uint8_t* packet);
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

    // 创建原始数据包队列（7字节hex数据，最多2组）
    handle->raw_queue = xQueueCreate(GY25T_RAW_QUEUE_SIZE, sizeof(gy25t_raw_packet_t));
    if (handle->raw_queue == NULL) {
        ESP_LOGE(TAG, "原始数据包队列创建失败！");
        free(handle);
        return NULL;
    }

    // 创建解析后数据队列
    handle->data_queue = xQueueCreate(DATA_QUEUE_SIZE, sizeof(float));
    if (handle->data_queue == NULL) {
        ESP_LOGE(TAG, "数据消息队列创建失败！");
        vQueueDelete(handle->raw_queue);
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
    
    // 安装UART驱动（使用事件队列）
    QueueHandle_t uart_queue;
    ESP_ERROR_CHECK(uart_driver_install(config->uart_port, config->buf_size, 0, 10, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(config->uart_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(config->uart_port, config->txd_pin, config->rxd_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // 设置全局句柄
    g_interrupt_handle = handle;

    // 创建数据接收和解析任务
    if (xTaskCreate(gy25t_parse_task, "gy25t_parse_task", 4096, handle, 5, &handle->read_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "GY-25T解析任务创建失败！");
        vQueueDelete(handle->raw_queue);
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
    if (handle->raw_queue) vQueueDelete(handle->raw_queue);
    if (handle->data_queue) vQueueDelete(handle->data_queue);
    uart_driver_delete(handle->config.uart_port);
    g_interrupt_handle = NULL; // 清除全局句柄
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
    const float ALPHA_FILTER = 0.5f;  // 提高响应速度：新值占50%，历史值占50%
    if (handle->filter.first_read) {
        *filtered_yaw = raw_yaw;
        handle->filter.first_read = false;
    } else {
        *filtered_yaw = ALPHA_FILTER * raw_yaw + (1.0f - ALPHA_FILTER) * (*filtered_yaw);
    }
}

// 数据接收和解析任务
static void gy25t_parse_task(void *pvParameters) {
    gy25t_handle_t* handle = (gy25t_handle_t*)pvParameters;
    uint8_t* recv_buf = (uint8_t*)malloc(PARSER_BUFFER_SIZE);
    static uint8_t parse_buf[PARSER_BUFFER_SIZE];
    static int parse_len = 0;

    if (!recv_buf) {
        ESP_LOGE(TAG, "接收缓冲区内存分配失败！");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "GY-25T数据接收解析任务已启动");

    // 主动查询计时器
    TickType_t last_query_time = xTaskGetTickCount();
    const TickType_t query_interval = pdMS_TO_TICKS(10); // 10ms查询一次，100Hz

    while (1) {
        // 主动发送查询指令（GY25T可能需要主动查询而不是自动发送）
        TickType_t current_time = xTaskGetTickCount();
        if (current_time - last_query_time >= query_interval) {
            // 发送YAW角查询指令：A4 03 18 02 C1
            uint8_t query_cmd[] = {0xA4, 0x03, 0x18, 0x02, 0xC1};
            uart_write_bytes(handle->config.uart_port, query_cmd, sizeof(query_cmd));
            last_query_time = current_time;
            
            static uint32_t query_count = 0;
            if (++query_count % 1000 == 0) {
                ESP_LOGI(TAG, "已发送 %lu 个查询指令", query_count);
            }
        }

        // 高频读取UART数据（1ms超时，提高响应速度）
        int len = uart_read_bytes(handle->config.uart_port, recv_buf, PARSER_BUFFER_SIZE, pdMS_TO_TICKS(1));
        if (len > 0) {
            // 调试：打印接收到的原始数据
            static uint32_t debug_count = 0;
            if (++debug_count % 1000 == 0) { // 每1000次打印一次调试信息
                ESP_LOGI(TAG, "接收到 %d 字节数据", len);
                for (int i = 0; i < len && i < 20; i++) {
                    printf("%02X ", recv_buf[i]);
                }
                printf("\n");
            }
            // 添加到解析缓冲区
            if (parse_len + len > PARSER_BUFFER_SIZE) {
                parse_len = 0; // 重置缓冲区
            }
            memcpy(parse_buf + parse_len, recv_buf, len);
            parse_len += len;

            // 查找完整的7字节数据包
            while (parse_len >= GY25T_YAW_PACKET_SIZE) {
                int frame_offset = -1;
                for (int i = 0; i <= parse_len - GY25T_YAW_PACKET_SIZE; i++) {
                    if (parse_buf[i] == 0xA4 && parse_buf[i+1] == 0x03 && 
                        parse_buf[i+2] == 0x18 && parse_buf[i+3] == 0x02) {
                        frame_offset = i;
                        break;
                    }
                }

                if (frame_offset != -1) {
                    // 找到数据包，发送到原始队列
                    gy25t_raw_packet_t packet;
                    memcpy(packet.data, &parse_buf[frame_offset], GY25T_YAW_PACKET_SIZE);
                    
                    // 如果队列满，移除最旧的数据包
                    if (xQueueSend(handle->raw_queue, &packet, 0) != pdTRUE) {
                        gy25t_raw_packet_t dummy;
                        xQueueReceive(handle->raw_queue, &dummy, 0);
                        xQueueSend(handle->raw_queue, &packet, 0);
                    }

                    // 解析数据包
                    if (gy25t_parse_packet(handle, packet.data)) {
                        // 调试：打印解析成功的数据包
                        static uint32_t parse_count = 0;
                        if (++parse_count % 100 == 0) { // 每100个包打印一次
                            ESP_LOGI(TAG, "成功解析第 %lu 个数据包", parse_count);
                        }
                    }

                    // 移除已处理的数据
                    parse_len -= (frame_offset + GY25T_YAW_PACKET_SIZE);
                    memmove(parse_buf, &parse_buf[frame_offset + GY25T_YAW_PACKET_SIZE], parse_len);
                } else {
                    // 没找到完整包，保留最后几字节继续查找
                    if (parse_len > GY25T_YAW_PACKET_SIZE) {
                        memmove(parse_buf, parse_buf + parse_len - (GY25T_YAW_PACKET_SIZE - 1), GY25T_YAW_PACKET_SIZE - 1);
                        parse_len = GY25T_YAW_PACKET_SIZE - 1;
                    }
                    break;
                }
            }
        }
    }

    free(recv_buf);
    vTaskDelete(NULL);
}

// 解析单个数据包
static bool gy25t_parse_packet(gy25t_handle_t* handle, const uint8_t* packet) {
    if (!handle || !packet) return false;

    // 校验数据包
    uint8_t checksum = gy25t_calculate_checksum((uint8_t*)packet, 6);
    if (checksum != packet[6]) {
        handle->packets_invalid++;
        return false;
    }

    // 解析YAW角度数据
    int16_t raw_yaw_value = (int16_t)((packet[4] << 8) | packet[5]);
    float calculated_yaw = (float)raw_yaw_value / 100.0f;
    
    // 直接使用原始角度数据，不进行滤波处理
    // 正确的"滑动窗口"队列管理
    if (xQueueSend(handle->data_queue, &calculated_yaw, 0) != pdTRUE) {
        float dummy;
        xQueueReceive(handle->data_queue, &dummy, 0); // 队列满，移除最旧的
        xQueueSend(handle->data_queue, &calculated_yaw, 0); // 发送最新的
    }
    
    handle->packets_received++;
    return true;
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
