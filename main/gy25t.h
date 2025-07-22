#ifndef GY25T_H
#define GY25T_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"

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

// YAW角滤波配置
typedef struct {
    float zero_angle;               // 零位角度 (度)
    bool first_read;                // 首次读取标志
} gy25t_filter_t;

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

// 输出频率枚举
typedef enum {
    GYRO_RATE_100HZ = 0x01, // 100Hz 输出频率
    GYRO_RATE_200HZ = 0x02  // 200Hz 输出频率
} gy25t_output_rate_t;

// 统计信息
typedef struct {
    uint32_t packets_received;    // 接收的数据包总数
    uint32_t packets_invalid;     // 无效数据包数
} gy25t_stats_t;

// 陀螺仪句柄结构
typedef struct {
    gy25t_config_t config;          // 配置信息
    gy25t_filter_t filter;          // 滤波器
    TaskHandle_t read_task_handle;  // 读取任务句柄
    
    // 原始数据包队列（7字节hex数据，最多2组）
    QueueHandle_t raw_queue;        // 用于传递原始7字节数据包的队列
    QueueHandle_t data_queue;       // 用于传递已解析YAW角数据的队列
    volatile float last_yaw;        // 保存从队列中获取的最后一个YAW角

    // 统计信息
    volatile uint32_t packets_received;
    volatile uint32_t packets_invalid;
} gy25t_handle_t;

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

/**
 * @brief 获取当前原始YAW角
 * @param handle 陀螺仪句柄
 * @return 原始YAW角度值
 */
float gy25t_get_raw_yaw(gy25t_handle_t* handle);

/**
 * @brief 获取当前滤波后YAW角
 * @param handle 陀螺仪句柄
 * @return 滤波后YAW角度值
 */
float gy25t_get_filtered_yaw(gy25t_handle_t* handle);

/**
 * @brief 校准零位角度
 * @param handle 陀螺仪句柄
 */
void gy25t_calibrate_zero_position(gy25t_handle_t* handle);

/**
 * @brief 设置GY-25T模块的输出频率
 * @param handle 陀螺仪句柄
 * @param rate 输出频率
 */
void gy25t_set_output_rate(gy25t_handle_t* handle, gy25t_output_rate_t rate);

// === 附加功能接口 ===

/**
 * @brief 获取统计信息
 * @param handle 陀螺仪句柄
 * @param[out] stats 指向统计数据结构体的指针
 */
void gy25t_get_stats(gy25t_handle_t* handle, gy25t_stats_t* stats);

/**
 * @brief 检查是否有新YAW数据
 * @param handle 陀螺仪句柄
 * @return true表示有新数据
 */
bool gy25t_has_new_data(gy25t_handle_t* handle);

/**
 * @brief 清除新数据标志
 * @param handle 陀螺仪句柄
 */
void gy25t_clear_new_data_flag(gy25t_handle_t* handle);

#ifdef __cplusplus
}
#endif

#endif // GY25T_H
