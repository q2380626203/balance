#ifndef JY901S_H
#define JY901S_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define JY901S_PACKET_HEADER1 0x55
#define JY901S_PACKET_HEADER2 0x53
#define JY901S_PACKET_LENGTH 11

typedef struct {
    float roll;             // 滚转角X (°)
    float pitch;            // 俯仰角Y (°) 
    float yaw;              // 偏航角Z (°)
    uint16_t version;       // 版本号
    bool is_valid;          // 数据有效性
    uint32_t packet_count;  // 包计数器
    uint32_t timestamp;     // 时间戳
} jy901s_data_t;

typedef struct {
    uart_port_t uart_port;
    int rxd_pin;
    int txd_pin;
    int baud_rate;
    SemaphoreHandle_t data_mutex;
    jy901s_data_t sensor_data;
    uint32_t total_bytes_received;
    bool initialized;
} jy901s_handle_t;

// 初始化JY901S模块
jy901s_handle_t* jy901s_init(uart_port_t uart_port, int rxd_pin, int txd_pin, int baud_rate);

// 销毁JY901S模块
void jy901s_destroy(jy901s_handle_t* handle);

// 获取最新传感器数据（线程安全）
bool jy901s_get_data(jy901s_handle_t* handle, jy901s_data_t* data);

// 获取特定角度值（线程安全）
float jy901s_get_pitch(jy901s_handle_t* handle);
float jy901s_get_roll(jy901s_handle_t* handle);
float jy901s_get_yaw(jy901s_handle_t* handle);

// 数据解析任务（内部使用）
void jy901s_parse_task(void* pvParameters);

// 获取统计信息
uint32_t jy901s_get_bytes_received(jy901s_handle_t* handle);
uint32_t jy901s_get_packet_count(jy901s_handle_t* handle);

#ifdef __cplusplus
}
#endif

#endif // JY901S_H