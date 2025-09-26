#include "motor_control.h"
#include "uart_monitor.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// ====================================================================================
// --- 常量定义 ---
// ====================================================================================

// CAN 指令 ID
#define ENABLE_ID           0x0027
#define VEL_MODE_ID         0x002B      // 设置速度模式的 CAN ID
#define TARGET_VEL_ID       0x002D      // 发送目标速度的 CAN ID
#define CLEAR_ERROR_ID      0x0038      // 清除错误和异常的 CAN ID
#define QUERY_EXCEPTION_ID  0x0023      // 查询电机异常的 CAN ID
#define RESTART_MOTOR_ID    0x0036      // 重启电机的 CAN ID

// CAN 指令数据
static const uint8_t ENABLE_DATA[]      = {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 致能马达
static const uint8_t DISABLE_DATA[]     = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 失能马达
static const uint8_t VEL_DIRECT_MODE_DATA[] = {0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}; // 速度直接模式数据
static const uint8_t CLEAR_ERROR_DATA[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 清除错误和异常数据
static const uint8_t RESTART_MOTOR_DATA[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 重启电机数据

// 记录最后查询的异常类型
static int g_last_exception_query_type = -1;
static const char* TAG = "MOTOR_CONTROL";

// UART监听相关
static uart_monitor_t* g_uart_monitor = NULL;

// 串口读写互斥锁 - 已禁用
// static SemaphoreHandle_t g_uart_rw_mutex = NULL;

// 内部函数声明
static void send_serial_can_frame(uart_port_t uart_port, const char* cmd_name, 
                                 uint32_t id, const uint8_t *data, uint8_t len);

// ====================================================================================
// --- 电机控制器主要接口实现 ---
// ====================================================================================

motor_controller_t* motor_control_init(const motor_driver_config_t* driver_config, 
                                      float velocity_limit) {
    if (!driver_config) {
        printf("[错误] 电机控制器配置参数为空！\n");
        return NULL;
    }

    // 分配控制器内存
    motor_controller_t* controller = (motor_controller_t*)malloc(sizeof(motor_controller_t));
    if (!controller) {
        printf("[错误] 电机控制器内存分配失败！\n");
        return NULL;
    }

    // 复制配置
    memcpy(&controller->driver_config, driver_config, sizeof(motor_driver_config_t));
    controller->velocity_limit = velocity_limit;

    // 初始化电机状态
    controller->motor_enabled = false;
    
    // 初始化错误状态
    memset(&controller->error_status, 0, sizeof(motor_error_status_t));

    // 初始化UART
    uart_config_t uart_config = {
        .baud_rate = driver_config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    
    uart_driver_install(driver_config->uart_port, driver_config->buf_size * 2, 0, 0, NULL, 0);
    uart_param_config(driver_config->uart_port, &uart_config);
    uart_set_pin(driver_config->uart_port, driver_config->txd_pin, driver_config->rxd_pin, 
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // 串口读写互斥锁已禁用

    // 初始化电机为速度模式
    set_motor_velocity_mode(driver_config->uart_port);

    // 启动UART监听任务
    if (!g_uart_monitor) {
        uart_monitor_config_t monitor_config = {
            .uart_port = driver_config->uart_port,
            .buf_size = driver_config->buf_size,
            .tag = "MOTOR_UART",
            .init_uart = false  // 复用已有的UART
        };
        
        g_uart_monitor = uart_monitor_init(&monitor_config);
        if (g_uart_monitor) {
            uart_monitor_set_motor_controller(controller);
            if (uart_monitor_start(g_uart_monitor)) {
                ESP_LOGI(TAG, "UART监听器启动成功");
            } else {
                ESP_LOGE(TAG, "UART监听器启动失败");
                uart_monitor_deinit(g_uart_monitor);
                g_uart_monitor = NULL;
            }
        } else {
            ESP_LOGE(TAG, "UART监听器初始化失败");
        }
    }

    printf("[信息] 电机控制器在 UART%d 上初始化完成，速度限制: %.2f r/s\n", 
           driver_config->uart_port, velocity_limit);
    return controller;
}

void motor_control_deinit(motor_controller_t* controller) {
    if (!controller) return;

    // 失能电机
    motor_control_enable(controller, false);

    // 停止UART监听任务
    if (g_uart_monitor) {
        uart_monitor_deinit(g_uart_monitor);
        g_uart_monitor = NULL;
        ESP_LOGI(TAG, "UART监听器已停止");
    }

    // 串口读写互斥锁已禁用

    // 删除UART驱动
    uart_driver_delete(controller->driver_config.uart_port);

    // 释放内存
    free(controller);
    
    printf("[信息] 电机控制器已销毁\n");
}

void motor_control_enable(motor_controller_t* controller, bool enable) {
    if (!controller) return;

    if (enable && !controller->motor_enabled) {
        enable_motor(controller->driver_config.uart_port);
        controller->motor_enabled = true;
        printf("[信息] 电机已使能\n");
    } else if (!enable && controller->motor_enabled) {
        disable_motor(controller->driver_config.uart_port);
        controller->motor_enabled = false;
        printf("[信息] 电机已失能\n");
    }
}

void motor_control_set_velocity(motor_controller_t* controller, float velocity) {
    if (!controller) return;

    move_motor_to_velocity(controller->driver_config.uart_port, 
                          velocity, controller->velocity_limit);
}

void motor_control_clear_errors(motor_controller_t* controller) {
    if (!controller) return;

    clear_motor_errors(controller->driver_config.uart_port);
    printf("[信息] 电机错误和异常已清除\n");
}

bool motor_control_is_enabled(motor_controller_t* controller) {
    if (!controller) return false;
    return controller->motor_enabled;
}

// ====================================================================================
// --- 低级别电机驱动函数实现 ---
// ====================================================================================

static void send_serial_can_frame(uart_port_t uart_port, const char* cmd_name, 
                                 uint32_t id, const uint8_t *data, uint8_t len) {
    uint8_t tx_buffer[10];
    tx_buffer[0] = (id >> 8) & 0xFF; // CAN ID high byte
    tx_buffer[1] = id & 0xFF;        // CAN ID low byte
    memcpy(&tx_buffer[2], data, len); // Copy data
    uart_write_bytes(uart_port, tx_buffer, sizeof(tx_buffer)); // Send data
    ESP_LOGD(TAG, "串口发送: %s (ID:0x%04X)", cmd_name, id);
}

void set_motor_velocity_mode(uart_port_t uart_port) {
    send_serial_can_frame(uart_port, "设置速度模式", VEL_MODE_ID, VEL_DIRECT_MODE_DATA, sizeof(VEL_DIRECT_MODE_DATA));
}

void enable_motor(uart_port_t uart_port) {
    send_serial_can_frame(uart_port, "致能马达", ENABLE_ID, ENABLE_DATA, sizeof(ENABLE_DATA));
}

void disable_motor(uart_port_t uart_port) {
    send_serial_can_frame(uart_port, "失能马达", ENABLE_ID, DISABLE_DATA, sizeof(DISABLE_DATA));
}

void move_motor_to_velocity(uart_port_t uart_port, float velocity, float velocity_limit) {
    // 限制电机速度在设定范围内
    if (velocity > velocity_limit) velocity = velocity_limit;
    if (velocity < -velocity_limit) velocity = -velocity_limit;

    uint8_t can_data[8] = {0};
    memcpy(can_data, &velocity, sizeof(velocity)); // Copy float velocity to CAN data
    send_serial_can_frame(uart_port, "MoveToVelocity", TARGET_VEL_ID, can_data, sizeof(can_data));
}

void clear_motor_errors(uart_port_t uart_port) {
    send_serial_can_frame(uart_port, "清除错误和异常", CLEAR_ERROR_ID, CLEAR_ERROR_DATA, sizeof(CLEAR_ERROR_DATA));
}

void query_motor_exceptions(uart_port_t uart_port, int exception_type) {
    uint8_t exception_data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (exception_type >= 0 && exception_type <= 4) {
        exception_data[0] = exception_type;
        // 记录查询的异常类型，用于后续解析
        g_last_exception_query_type = exception_type;
        printf("[MOTOR_CONTROL] 设置异常查询类型为: %d\n", exception_type);
    }
    send_serial_can_frame(uart_port, "查询电机异常", QUERY_EXCEPTION_ID, exception_data, sizeof(exception_data));
}

void restart_motor(uart_port_t uart_port) {
    send_serial_can_frame(uart_port, "重启电机", RESTART_MOTOR_ID, RESTART_MOTOR_DATA, sizeof(RESTART_MOTOR_DATA));
}

int32_t bytes_to_int32(const uint8_t *bytes) {
    return (int32_t)bytes[0] | 
           ((int32_t)bytes[1] << 8) | 
           ((int32_t)bytes[2] << 16) | 
           ((int32_t)bytes[3] << 24);
}

void parse_error_data(const uint8_t *data, uint8_t error_type, motor_error_status_t *error_status) {
    if (!data || !error_status) return;
    
    // 取前4字节作为异常码（小端序）
    uint32_t error_code = (uint32_t)data[0] | 
                          ((uint32_t)data[1] << 8) | 
                          ((uint32_t)data[2] << 16) | 
                          ((uint32_t)data[3] << 24);
    
    // 根据异常类型更新对应的错误字段
    switch (error_type) {
        case 0:  // 电机异常
            error_status->motor_error = error_code;
            break;
        case 1:  // 编码器异常
            error_status->encoder_error = error_code;
            break;
        case 3:  // 控制器异常
            error_status->controller_error = error_code;
            break;
        case 4:  // 系统异常
            error_status->system_error = error_code;
            break;
        default:
            printf("[错误] 未知异常类型: %d\n", error_type);
            return;
    }
    
    error_status->error_data_valid = true;
    error_status->last_error_query_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    printf("[MOTOR_CONTROL] 解析异常数据 - 类型:%d, 错误码:0x%08lX\n", error_type, (unsigned long)error_code);
}

// ====================================================================================
// --- 错误描述功能 ---
// ====================================================================================

// 异常码描述表
typedef struct {
    uint32_t code;
    const char* description;
} error_desc_t;

// 电机异常码表
static const error_desc_t motor_errors[] = {
    {0x00000001, "相间电阻超出正常范围"},
    {0x00000002, "相间电感超出正常范围"}, 
    {0x00000010, "FOC频率太高"},
    {0x00000080, "SVM调制异常"},
    {0x00000400, "相间电流饱和"},
    {0x00001000, "电机电流过大"},
    {0x00020000, "电机温度过高"},
    {0x00040000, "驱动器温度过高"},
    {0x00080000, "FOC处理不及时"},
    {0x00100000, "相间电流采样失效"},
    {0x00200000, "控制器异常"},
    {0x00400000, "母线电压超限"},
    {0x00800000, "刹车电阻驱动异常"},
    {0x01000000, "系统级异常"},
    {0x02000000, "相间电流采样不及时"},
    {0x04000000, "电机位置未知"},
    {0x08000000, "电机速度未知"},
    {0x10000000, "力矩未知"},
    {0x20000000, "力矩控制未知"},
    {0x40000000, "电流采样值未知"},
    {0, NULL}
};

// 编码器异常码表
static const error_desc_t encoder_errors[] = {
    {0x00000001, "编码器带宽过高"},
    {0x00000002, "CPR和极对数不匹配"},
    {0x00000004, "编码器无响应"},
    {0x00000400, "第二编码器通信错误"},
    {0, NULL}
};

// 控制器异常码表  
static const error_desc_t controller_errors[] = {
    {0x00000001, "速度过高"},
    {0x00000002, "控制输入模式不正确"},
    {0x00000004, "锁相环增益不稳"},
    {0x00000020, "位置/速度不稳定"},
    {0x00000080, "机械功率和电气功率不匹配(编码器校准不正确,或磁钢不稳)"},
    {0, NULL}
};

// 系统异常码表
static const error_desc_t system_errors[] = {
    {0x00000002, "电源电压过低"},
    {0x00000004, "电源电压过高"},
    {0x00000008, "电源反向（充电）电流过高"},
    {0x00000010, "电源正向（放电）电流过高"},
    {0, NULL}
};

const char* motor_control_get_error_description(uint32_t error_code, uint8_t error_type) {
    if (error_code == 0) {
        return "正常";
    }
    
    const error_desc_t* error_table = NULL;
    
    switch (error_type) {
        case 0:  // 电机异常
            error_table = motor_errors;
            break;
        case 1:  // 编码器异常
            error_table = encoder_errors;
            break;
        case 3:  // 控制器异常
            error_table = controller_errors;
            break;
        case 4:  // 系统异常
            error_table = system_errors;
            break;
        default:
            return "未知异常类型";
    }
    
    // 先尝试精确匹配
    for (int i = 0; error_table[i].description != NULL; i++) {
        if (error_table[i].code == error_code) {
            return error_table[i].description;
        }
    }
    
    return "复合异常";
}

// 解析复合错误码，返回所有匹配的错误描述
int motor_control_parse_error_bits(uint32_t error_code, uint8_t error_type, char* result_buffer, size_t buffer_size) {
    if (error_code == 0) {
        strncpy(result_buffer, "正常", buffer_size - 1);
        result_buffer[buffer_size - 1] = '\0';
        return 1;
    }
    
    const error_desc_t* error_table = NULL;
    
    switch (error_type) {
        case 0:  // 电机异常
            error_table = motor_errors;
            break;
        case 1:  // 编码器异常
            error_table = encoder_errors;
            break;
        case 3:  // 控制器异常
            error_table = controller_errors;
            break;
        case 4:  // 系统异常
            error_table = system_errors;
            break;
        default:
            strncpy(result_buffer, "未知异常类型", buffer_size - 1);
            result_buffer[buffer_size - 1] = '\0';
            return 0;
    }
    
    int error_count = 0;
    result_buffer[0] = '\0';
    
    // 位操作解析复合错误码
    for (int i = 0; error_table[i].description != NULL; i++) {
        if (error_code & error_table[i].code) {  // 位与操作检查错误位
            if (error_count > 0) {
                strncat(result_buffer, "; ", buffer_size - strlen(result_buffer) - 1);
            }
            strncat(result_buffer, error_table[i].description, buffer_size - strlen(result_buffer) - 1);
            error_count++;
        }
    }
    
    if (error_count == 0) {
        strncpy(result_buffer, "未知异常码", buffer_size - 1);
        result_buffer[buffer_size - 1] = '\0';
    }
    
    return error_count;
}

// ====================================================================================
// --- 高级别错误查询接口 ---
// ====================================================================================

void motor_control_query_errors(motor_controller_t* controller, int exception_type) {
    if (!controller) {
        printf("[错误] 电机控制器句柄为空！\n");
        return;
    }
    
    if (exception_type < 0 || (exception_type > 1 && exception_type != 3 && exception_type != 4)) {
        printf("[错误] 无效的异常类型: %d，有效值为 0, 1, 3, 4\n", exception_type);
        return;
    }
    
    query_motor_exceptions(controller->driver_config.uart_port, exception_type);
    
    const char* type_names[] = {"电机", "编码器", "", "控制器", "系统"};
    printf("[信息] 查询%s异常状态\n", type_names[exception_type]);
}

motor_error_status_t* motor_control_get_error_status(motor_controller_t* controller) {
    if (!controller) {
        return NULL;
    }
    return &controller->error_status;
}

bool motor_control_has_errors(motor_controller_t* controller) {
    if (!controller) {
        return false;
    }
    
    motor_error_status_t* status = &controller->error_status;
    
    // 检查是否有任何非零的错误码
    return (status->motor_error != 0 || 
            status->encoder_error != 0 || 
            status->controller_error != 0 || 
            status->system_error != 0);
}

void motor_control_restart(motor_controller_t* controller) {
    if (!controller) {
        printf("[错误] 电机控制器句柄为空！\n");
        return;
    }
    
    restart_motor(controller->driver_config.uart_port);
    printf("[信息] 电机重启指令已发送\n");
}

SemaphoreHandle_t motor_control_get_uart_mutex(void) {
    return NULL; // 互斥锁已禁用
}

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
                                        size_t length) {
    if (!controller || !data || length < 8) {
        return false;
    }
    
    // 检查是否是错误查询响应 (0x0023)
    if (can_id != QUERY_EXCEPTION_ID) {
        return false;
    }
    
    // 使用记录的异常查询类型来解析数据
    if (g_last_exception_query_type >= 0) {
        parse_error_data(data, g_last_exception_query_type, &controller->error_status);
        
        // 获取错误描述并打印
        uint32_t error_code = (uint32_t)data[0] | 
                              ((uint32_t)data[1] << 8) | 
                              ((uint32_t)data[2] << 16) | 
                              ((uint32_t)data[3] << 24);
        
        const char* description = motor_control_get_error_description(error_code, g_last_exception_query_type);
        const char* type_names[] = {"电机", "编码器", "", "控制器", "系统"};
        
        printf("[信息] %s异常状态: 0x%08lX - %s\n", 
               type_names[g_last_exception_query_type], 
               (unsigned long)error_code, 
               description);
        
        return true;
    }
    
    return false;
}