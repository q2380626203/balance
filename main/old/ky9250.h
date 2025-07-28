#ifndef KY9250_H
#define KY9250_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float gx, gy, gz;        // 角速度 (°/s)
    float ax, ay, az;        // 加速度 (g)
    float mx, my, mz;        // 磁力计
    float roll, pitch, yaw;  // 欧拉角 (°)
    float q0, q1, q2, q3;    // 四元数
    bool is_valid;           // 数据有效性
    uint32_t packet_count;   // 包计数器
    uint32_t timestamp;      // 时间戳
} ky9250_data_t;

typedef struct {
    uart_port_t uart_port;
    int rxd_pin;
    int txd_pin;
    int baud_rate;
    SemaphoreHandle_t data_mutex;
    ky9250_data_t sensor_data;
    uint32_t total_bytes_received;
    bool initialized;
} ky9250_handle_t;

// 初始化KY9250模块
ky9250_handle_t* ky9250_init(uart_port_t uart_port, int rxd_pin, int txd_pin, int baud_rate);

// 销毁KY9250模块
void ky9250_destroy(ky9250_handle_t* handle);

// 获取最新传感器数据（线程安全）
bool ky9250_get_data(ky9250_handle_t* handle, ky9250_data_t* data);

// 获取特定角度值（线程安全）
float ky9250_get_pitch(ky9250_handle_t* handle);
float ky9250_get_roll(ky9250_handle_t* handle);
float ky9250_get_yaw(ky9250_handle_t* handle);

// 数据解析任务（内部使用）
void ky9250_parse_task(void* pvParameters);

// 获取统计信息
uint32_t ky9250_get_bytes_received(ky9250_handle_t* handle);
uint32_t ky9250_get_packet_count(ky9250_handle_t* handle);

#ifdef __cplusplus
}
#endif

#endif // KY9250_H