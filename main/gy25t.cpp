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

// 配置
static const char *TAG = "GY25T_YAW";

// 全局变量
static gy25t_handle_t* g_interrupt_handle = NULL;
volatile float g_last_yaw = 0.0f;  // 全局YAW角度值

// 静态函数声明
static void IRAM_ATTR uart_rx_intr_handler(void* arg);
static void gy25t_parse_task(void *pvParameters);
static bool gy25t_parse_packet(gy25t_handle_t* handle, const uint8_t* packet);
static uint8_t gy25t_calculate_checksum(uint8_t *data, int length);

// 原始字节数据结构，用于中断中传递单个字节
typedef struct {
    uint8_t byte;
} gy25t_raw_byte_t;

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

    // 创建原始字节队列（用于接收单个字节数据，最多32个字节）
    handle->raw_queue = xQueueCreate(32, sizeof(gy25t_raw_byte_t));
    if (handle->raw_queue == NULL) {
        ESP_LOGE(TAG, "原始数据包队列创建失败！");
        free(handle);
        return NULL;
    }

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
    
    // 安装UART驱动（不使用事件队列）
    ESP_ERROR_CHECK(uart_driver_install(config->uart_port, 0, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(config->uart_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(config->uart_port, config->txd_pin, config->rxd_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // 注册自定义串口中断处理函数
    ESP_ERROR_CHECK(esp_intr_alloc(ETS_UART1_INTR_SOURCE, ESP_INTR_FLAG_IRAM, uart_rx_intr_handler, handle, NULL));
    
    // 启用UART接收中断
    uart_enable_intr_mask(config->uart_port, UART_RXFIFO_FULL_INT_ENA | UART_RXFIFO_TOUT_INT_ENA);

    // 设置全局句柄
    g_interrupt_handle = handle;

    // 创建数据解析任务
    if (xTaskCreate(gy25t_parse_task, "gy25t_parse_task", 4096, handle, 5, &handle->parse_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "GY-25T解析任务创建失败！");
        vQueueDelete(handle->raw_queue);
        uart_driver_delete(config->uart_port);
        free(handle);
        return NULL;
    }

    ESP_LOGI(TAG, "GY-25T YAW角模块在 UART%d 上初始化完成 (硬件中断模式)", config->uart_port);
    return handle;
}

void gy25t_deinit(gy25t_handle_t* handle) {
    if (!handle) return;
    if (handle->parse_task_handle) vTaskDelete(handle->parse_task_handle);
    if (handle->raw_queue) vQueueDelete(handle->raw_queue);
    uart_driver_delete(handle->config.uart_port);
    g_interrupt_handle = NULL; // 清除全局句柄
    free(handle);
    ESP_LOGI(TAG, "GY-25T YAW角模块已销毁");
}




// ====================================================================================
// --- 中断处理程序和解析任务 ---
// ====================================================================================

// 极简的UART接收中断处理函数，只负责接收字节并入队
static void IRAM_ATTR uart_rx_intr_handler(void* arg) {
    gy25t_handle_t* handle = (gy25t_handle_t*)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // 获取中断状态
    uint32_t uart_intr_status = REG_READ(UART_INT_ST_REG(UART_NUM_1));
    
    // 处理接收中断
    if (uart_intr_status & (UART_RXFIFO_FULL_INT_ST | UART_RXFIFO_TOUT_INT_ST)) {
        // 读取FIFO中的所有数据，每个字节直接入队
        while (REG_GET_FIELD(UART_STATUS_REG(UART_NUM_1), UART_RXFIFO_CNT) > 0) {
            gy25t_raw_byte_t raw_byte;
            raw_byte.byte = REG_READ(UART_FIFO_REG(UART_NUM_1)) & 0xFF;
            
            // 非阻塞发送到队列
            if (xQueueSendFromISR(handle->raw_queue, &raw_byte, &xHigherPriorityTaskWoken) != pdTRUE) {
                // 队列满时，丢弃最旧的字节
                gy25t_raw_byte_t dummy;
                xQueueReceiveFromISR(handle->raw_queue, &dummy, &xHigherPriorityTaskWoken);
                xQueueSendFromISR(handle->raw_queue, &raw_byte, &xHigherPriorityTaskWoken);
            }
        }
        
        // 清除接收中断标志
        REG_WRITE(UART_INT_CLR_REG(UART_NUM_1), UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR);
    }
    
    // 如果有更高优先级任务被唤醒，请求任务切换
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

// 数据解析任务（从字节队列中组装数据包并解析）
static void gy25t_parse_task(void *pvParameters) {
    gy25t_handle_t* handle = (gy25t_handle_t*)pvParameters;
    gy25t_raw_byte_t raw_byte;
    uint8_t parse_buffer[32];
    int parse_index = 0;
    
    ESP_LOGI(TAG, "GY-25T数据解析任务已启动");
    
    while (1) {
        // 从字节队列中获取数据
        if (xQueueReceive(handle->raw_queue, &raw_byte, portMAX_DELAY) == pdTRUE) {
            // 将字节添加到解析缓冲区
            parse_buffer[parse_index++] = raw_byte.byte;
            
            // 缓冲区溢出保护
            if (parse_index >= sizeof(parse_buffer)) {
                parse_index = 0;
            }
            
            // 检查是否有完整的GY25T数据包
            if (parse_index >= GY25T_YAW_PACKET_SIZE) {
                // 搜索帧头
                for (int i = 0; i <= parse_index - GY25T_YAW_PACKET_SIZE; i++) {
                    if (parse_buffer[i] == 0xA4 && parse_buffer[i+1] == 0x03 && 
                        parse_buffer[i+2] == 0x18 && parse_buffer[i+3] == 0x02) {
                        
                        // 找到完整数据包，解析并更新last_yaw
                        gy25t_parse_packet(handle, &parse_buffer[i]);
                        
                        // 移除已处理的数据
                        int remaining = parse_index - (i + GY25T_YAW_PACKET_SIZE);
                        if (remaining > 0) {
                            memmove(parse_buffer, &parse_buffer[i + GY25T_YAW_PACKET_SIZE], remaining);
                            parse_index = remaining;
                        } else {
                            parse_index = 0;
                        }
                        break;
                    }
                }
            }
        }
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
    
    // 调试信息：每100个包打印一次
    static uint32_t parse_count = 0;
    if (++parse_count % 100 == 0) {
        ESP_LOGI(TAG, "解析第%lu个数据包，YAW=%.2f°", parse_count, calculated_yaw);
    }
    
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

