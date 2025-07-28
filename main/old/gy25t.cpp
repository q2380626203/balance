#include "gy25t.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_intr_alloc.h"
#include "soc/uart_reg.h"
#include "hal/uart_hal.h"
#include "esp_private/periph_ctrl.h"
#include "soc/uart_periph.h"

// 配置
static const char *TAG = "GY25T_YAW";

// 全局变量
static gy25t_handle_t* g_interrupt_handle = NULL;
volatile float g_last_yaw = 0.0f;  // 全局YAW角度值

// 静态函数声明
static void gy25t_main_task(void *pvParameters);
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

    // 不需要队列，直接处理

    // 初始化全局YAW变量
    g_last_yaw = 0.0f;

    // 初始化UART
    uart_config_t uart_config = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {0}
    };
    
    // 优化的UART配置，增大接收缓冲区
    ESP_ERROR_CHECK(uart_param_config(config->uart_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(config->uart_port, config->txd_pin, config->rxd_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(config->uart_port, 1024, 0, 0, NULL, 0));  // 增大接收缓冲区到1024字节

    // 设置全局句柄
    g_interrupt_handle = handle;

    // 创建单个主任务（最高优先级，10Hz频率）
    if (xTaskCreate(gy25t_main_task, "gy25t_main_task", 4096, handle, 10, &handle->main_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "GY-25T主任务创建失败！");
        uart_driver_delete(config->uart_port);
        free(handle);
        return NULL;
    }

    ESP_LOGI(TAG, "GY-25T YAW角模块在 UART%d 上初始化完成 (高速模式)", config->uart_port);
    return handle;
}

void gy25t_deinit(gy25t_handle_t* handle) {
    if (!handle) return;
    if (handle->main_task_handle) vTaskDelete(handle->main_task_handle);
    uart_driver_delete(handle->config.uart_port);
    g_interrupt_handle = NULL; // 清除全局句柄
    free(handle);
    ESP_LOGI(TAG, "GY-25T YAW角模块已销毁");
}




// ====================================================================================
// --- 简化的主任务实现 ---
// ====================================================================================

// 主任务：高效处理串口数据并解析
static void gy25t_main_task(void *pvParameters) {
    gy25t_handle_t* handle = (gy25t_handle_t*)pvParameters;
    uint8_t buffer[64];  // 适中的缓冲区
    uint8_t parse_buffer[128];  // 解析缓冲区
    int parse_index = 0;
    
    ESP_LOGI(TAG, "GY-25T主任务已启动 (优化模式)");
    
    while (1) {
        // 适当超时读取串口数据，200Hz对应5ms周期
        int length = uart_read_bytes(handle->config.uart_port, buffer, sizeof(buffer), 10 / portTICK_PERIOD_MS);
        
        if (length > 0) {
            // 调试信息：显示接收到的原始数据
            // if (length <= 16) {  // 只显示前16个字节
            //     printf("[接收] ");
            //     for (int j = 0; j < length; j++) {
            //         printf("%02X ", buffer[j]);
            //     }
            //     printf("\n");
            // }
            
            // 添加接收到的数据到解析缓冲区
            for (int i = 0; i < length && parse_index < sizeof(parse_buffer); i++) {
                parse_buffer[parse_index++] = buffer[i];
            }
            
            // 持续处理缓冲区中的数据包
            bool processed_any = true;
            while (processed_any && parse_index >= GY25T_YAW_PACKET_SIZE) {
                processed_any = false;
                
                // 从缓冲区开始搜索完整数据包
                for (int i = 0; i <= parse_index - GY25T_YAW_PACKET_SIZE; i++) {
                    if (parse_buffer[i] == 0xA4 && parse_buffer[i+1] == 0x03 && 
                        parse_buffer[i+2] == 0x18 && parse_buffer[i+3] == 0x02) {
                        
                        // 找到完整数据包，显示调试信息
                        // printf("[解析] 找到数据包: ");
                        // for (int j = 0; j < GY25T_YAW_PACKET_SIZE; j++) {
                        //     printf("%02X ", parse_buffer[i + j]);
                        // }
                        // printf("\n");
                        
                        if (gy25t_parse_packet(handle, &parse_buffer[i])) {
                            // 成功解析，显示yaw值并移除已处理的数据
                            // printf("[成功] YAW角度更新: %.2f°\n", g_last_yaw);
                            int consumed = i + GY25T_YAW_PACKET_SIZE;
                            int remaining = parse_index - consumed;
                            if (remaining > 0) {
                                memmove(parse_buffer, &parse_buffer[consumed], remaining);
                                parse_index = remaining;
                            } else {
                                parse_index = 0;
                            }
                            processed_any = true;
                            break;
                        }
                    }
                }
                
                // 如果没有处理任何包且缓冲区满了，清理最旧的数据
                if (!processed_any && parse_index >= sizeof(parse_buffer)) {
                    memmove(parse_buffer, &parse_buffer[1], parse_index - 1);
                    parse_index--;
                }
            }
            
            // 如果缓冲区仍然满了，重置
            if (parse_index >= sizeof(parse_buffer)) {
                parse_index = 0;
                ESP_LOGW(TAG, "解析缓冲区重置");
            }
        }
        
        // 给其他任务让出时间，但频率仍然很高
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    
    vTaskDelete(NULL);
}

// 解析单个数据包并更新last_yaw
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
    
    // 直接更新全局变量 g_last_yaw
    g_last_yaw = calculated_yaw;
    
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

