#ifndef GY25T_H
#define GY25T_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"

#ifdef __cplusplus
extern "C" {
#endif

// === YAW角数据结构定义 ===

// YAW角原始数据
typedef struct {
    int16_t yaw;        // 航向角原始值
    bool new_data;      // 新数据标志
    uint64_t timestamp; // 时间戳 (微秒)
} gy25t_sensor_data_t;


// 陀螺仪初始化配置
typedef struct {
    uart_port_t uart_port;          // UART端口号
    gpio_num_t txd_pin;             // TXD引脚
    gpio_num_t rxd_pin;             // RXD引脚
    int baud_rate;                  // 波特率
    int buf_size;                   // 缓冲区大小
} gy25t_config_t;

// === YAW角数据解析配置 ===

#define GY25T_YAW_PACKET_SIZE     7         // YAW角数据包大小：帧头(4字节) + 数据(2字节) + 校验和(1字节)
#define GY25T_YAW_SCALE_FACTOR    100.0f    // YAW角缩放因子：原始值除以100得到度数
#define GY25T_RESET_ERROR_COUNT   100       // 连续错误重置阈值
#define GY25T_RAW_QUEUE_SIZE      2         // 原始数据包队列大小（最多2组）

// 原始数据包结构
typedef struct {
    uint8_t data[GY25T_YAW_PACKET_SIZE];    // 7字节原始数据包
} gy25t_raw_packet_t;


// 统计信息
typedef struct {
    uint32_t packets_received;    // 接收的数据包总数
    uint32_t packets_invalid;     // 无效数据包数
} gy25t_stats_t;

// 陀螺仪句柄结构
typedef struct {
    gy25t_config_t config;              // 配置信息
    TaskHandle_t main_task_handle;      // 主任务句柄

    // 统计信息
    volatile uint32_t packets_received;
    volatile uint32_t packets_invalid;
} gy25t_handle_t;

// 全局YAW角度变量
extern volatile float g_last_yaw;

// ====================================================================================
// --- GY-25T 陀螺仪模块接口函数 ---
// ====================================================================================

/**
 * @brief 初始化GY-25T陀螺仪模块
 * @param config 陀螺仪配置参数
 * @return 陀螺仪句柄指针，失败返回NULL
 */
gy25t_handle_t* gy25t_init(const gy25t_config_t* config);

/**
 * @brief 销毁GY-25T陀螺仪模块
 * @param handle 陀螺仪句柄
 */
void gy25t_deinit(gy25t_handle_t* handle);


// === 附加功能接口 ===

/**
 * @brief 获取统计信息
 * @param handle 陀螺仪句柄
 * @param[out] stats 指向统计数据结构体的指针
 */
void gy25t_get_stats(gy25t_handle_t* handle, gy25t_stats_t* stats);


#ifdef __cplusplus
}
#endif

#endif // GY25T_H
