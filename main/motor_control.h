#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

// ====================================================================================
// --- 电机控制模块数据结构定义 ---
// ====================================================================================

// 电机驱动配置结构
typedef struct {
    uart_port_t uart_port;          // UART端口号
    gpio_num_t txd_pin;             // TXD引脚
    gpio_num_t rxd_pin;             // RXD引脚
    int baud_rate;                  // 波特率
    int buf_size;                   // 缓冲区大小
} motor_driver_config_t;

// 电机错误状态结构
typedef struct {
    // 异常状态 (0x0023)
    uint32_t motor_error;          // 电机异常码
    uint32_t encoder_error;        // 编码器异常码
    uint32_t controller_error;     // 控制器异常码
    uint32_t system_error;         // 系统异常码
    
    bool error_data_valid;         // 错误数据有效性标志
    uint32_t last_error_query_time; // 最后错误查询时间戳
} motor_error_status_t;

// 电机控制器主结构
typedef struct {
    motor_driver_config_t driver_config;   // 驱动配置
    bool motor_enabled;                    // 电机使能状态
    float velocity_limit;                  // 电机最大速度限制
    motor_error_status_t error_status;     // 错误状态
} motor_controller_t;

// ====================================================================================
// --- 电机控制模块接口函数 ---
// ====================================================================================

/**
 * @brief 初始化电机控制器
 * @param driver_config 驱动配置
 * @param velocity_limit 速度限制
 * @return 电机控制器句柄，失败返回NULL
 */
motor_controller_t* motor_control_init(const motor_driver_config_t* driver_config, 
                                      float velocity_limit);

/**
 * @brief 销毁电机控制器
 * @param controller 电机控制器句柄
 */
void motor_control_deinit(motor_controller_t* controller);

/**
 * @brief 使能/失能电机
 * @param controller 电机控制器句柄
 * @param enable true为使能，false为失能
 */
void motor_control_enable(motor_controller_t* controller, bool enable);

/**
 * @brief 直接设置电机速度
 * @param controller 电机控制器句柄
 * @param velocity 目标速度 (r/s)
 */
void motor_control_set_velocity(motor_controller_t* controller, float velocity);

/**
 * @brief 清除电机错误和异常
 * @param controller 电机控制器句柄
 */
void motor_control_clear_errors(motor_controller_t* controller);

/**
 * @brief 获取电机使能状态
 * @param controller 电机控制器句柄
 * @return 使能状态
 */
bool motor_control_is_enabled(motor_controller_t* controller);

/**
 * @brief 查询电机异常状态
 * @param controller 电机控制器句柄
 * @param exception_type 异常类型 (0:电机异常, 1:编码器异常, 3:控制器异常, 4:系统异常)
 */
void motor_control_query_errors(motor_controller_t* controller, int exception_type);

/**
 * @brief 获取电机错误状态
 * @param controller 电机控制器句柄
 * @return 错误状态结构体指针
 */
motor_error_status_t* motor_control_get_error_status(motor_controller_t* controller);

/**
 * @brief 检查是否有任何错误
 * @param controller 电机控制器句柄
 * @return true表示有错误，false表示无错误
 */
bool motor_control_has_errors(motor_controller_t* controller);

/**
 * @brief 根据异常码获取异常描述
 * @param error_code 异常码
 * @param error_type 异常类型 (0:电机, 1:编码器, 3:控制器, 4:系统)
 * @return 异常描述字符串
 */
const char* motor_control_get_error_description(uint32_t error_code, uint8_t error_type);

/**
 * @brief 解析复合错误码，返回所有匹配的错误描述
 * @param error_code 异常码
 * @param error_type 异常类型 (0:电机, 1:编码器, 3:控制器, 4:系统)
 * @param result_buffer 结果缓冲区
 * @param buffer_size 缓冲区大小
 * @return 错误数量
 */
int motor_control_parse_error_bits(uint32_t error_code, uint8_t error_type, char* result_buffer, size_t buffer_size);

/**
 * @brief 处理电机错误查询的CAN响应数据
 * @param controller 电机控制器句柄
 * @param can_id CAN帧ID
 * @param data CAN帧数据
 * @param length 数据长度
 * @return true表示成功处理错误查询响应，false表示不是错误查询响应或处理失败
 */
bool motor_control_process_error_response(motor_controller_t* controller, 
                                        uint32_t can_id, 
                                        const uint8_t* data, 
                                        size_t length);

/**
 * @brief 重启电机（高级接口）
 * @param controller 电机控制器句柄
 */
void motor_control_restart(motor_controller_t* controller);

/**
 * @brief 获取串口读写互斥锁
 * @return 互斥锁句柄
 */
SemaphoreHandle_t motor_control_get_uart_mutex(void);

// ====================================================================================
// --- 低级别电机驱动函数 ---
// ====================================================================================

/**
 * @brief 设置电机为速度直接模式
 * @param uart_port UART端口
 */
void set_motor_velocity_mode(uart_port_t uart_port);

/**
 * @brief 使能电机
 * @param uart_port UART端口
 */
void enable_motor(uart_port_t uart_port);

/**
 * @brief 失能电机
 * @param uart_port UART端口
 */
void disable_motor(uart_port_t uart_port);

/**
 * @brief 移动电机到指定速度
 * @param uart_port UART端口
 * @param velocity 目标速度 (r/s)
 * @param velocity_limit 速度限制
 */
void move_motor_to_velocity(uart_port_t uart_port, float velocity, float velocity_limit);

/**
 * @brief 清除电机错误和异常
 * @param uart_port UART端口
 */
void clear_motor_errors(uart_port_t uart_port);

/**
 * @brief 查询电机异常信息
 * @param uart_port UART端口
 * @param exception_type 异常类型 (0:电机异常, 1:编码器异常, 3:控制器异常, 4:系统异常)
 */
void query_motor_exceptions(uart_port_t uart_port, int exception_type);

/**
 * @brief 重启电机
 * @param uart_port UART端口
 */
void restart_motor(uart_port_t uart_port);

/**
 * @brief 解析异常反馈数据
 * @param data 8字节CAN数据
 * @param error_type 异常类型 (0:电机 1:编码器 3:控制器 4:系统)
 * @param error_status 错误状态结构体指针
 */
void parse_error_data(const uint8_t *data, uint8_t error_type, motor_error_status_t *error_status);

/**
 * @brief 将4字节小端序数据转换为int32
 * @param bytes 4字节数据指针
 * @return int32值
 */
int32_t bytes_to_int32(const uint8_t *bytes);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_CONTROL_H