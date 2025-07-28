#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_task_wdt.h"
#include "motor_control.h"
#include "ky9250.h"

// =====================================================================================
// --- 硬件与软件配置区 (Hardware & Software Configuration) ---
// =====================================================================================

#define MOTOR_UART_PORT     UART_NUM_2
#define MOTOR_UART_TXD      GPIO_NUM_13
#define MOTOR_UART_RXD      GPIO_NUM_12

#define KY9250_UART_PORT    UART_NUM_1
#define KY9250_UART_RXD     GPIO_NUM_11
#define KY9250_UART_TXD     GPIO_NUM_10

#define UART_BUF_SIZE       (1024)

// --- Pitch角度平衡控制参数 ---
const float TARGET_PITCH_ANGLE = 0.0f;    // 目标pitch角度 (度) - 平衡位置
const float PITCH_TOLERANCE = 3.0f;       // pitch角度容差 (±3度) - 恢复你的原始参数
const float MOTOR_FIXED_SPEED = 15.0f;    // 电机固定转动速度 - 恢复你的原始参数

// --- 全局模块句柄 (Global Module Handles) ---
static motor_controller_t* g_motor_controller = NULL;
static ky9250_handle_t* g_ky9250_handle = NULL;


// =====================================================================================
// --- KY9250传感器数据处理 (KY9250 Sensor Data Processing) ---
// =====================================================================================

// 简化的数据处理，不再需要复杂的回调函数

// =====================================================================================
// --- 实时显示任务 (Real-time Display Task) ---
// =====================================================================================

// 固定显示的实时数据任务 (减少打印频率避免看门狗超时)
static void realtime_display_task(void *pvParameters) {
    const TickType_t refresh_rate = pdMS_TO_TICKS(500); // 2Hz刷新频率，避免看门狗超时
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        vTaskDelayUntil(&last_wake_time, refresh_rate);
        
        // 获取KY9250数据
        ky9250_data_t sensor_data;
        bool data_valid = ky9250_get_data(g_ky9250_handle, &sensor_data);
        
        printf("\033[2J\033[H"); // 注释掉清屏，减少打印时间
        printf("=== ESP32 KY9250 平衡控制系统 ===\n");
        printf("UART1: 接收%lu字节\n", ky9250_get_bytes_received(g_ky9250_handle));
        
        if (data_valid) {
            // 显示姿态数据，重点关注Pitch角
            printf("**PITCH: %6.2f°**  Roll: %6.2f°  Yaw: %6.2f°\n",
                   sensor_data.pitch, sensor_data.roll, sensor_data.yaw);
            printf("Acc: %.2f %.2f %.2f | 包#%lu\n",
                   sensor_data.ax, sensor_data.ay, sensor_data.az, sensor_data.packet_count);
            
            printf("平衡控制: PITCH=%.2f° (目标%.1f°±%.1f°)\n", 
                   sensor_data.pitch, TARGET_PITCH_ANGLE, PITCH_TOLERANCE);
        } else {
            printf("等待KY9250数据... (检查UART1连接)\n");
        }
        
        fflush(stdout);
    }
}

// 串口2数据监控任务（电机控制数据）
static void uart2_monitor_task(void *pvParameters) {
    uint8_t* data = (uint8_t*) malloc(UART_BUF_SIZE);
    printf("[信息] 串口2数据监控任务已启动\n");
    
    while (1) {
        int len = uart_read_bytes(MOTOR_UART_PORT, data, UART_BUF_SIZE, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            // printf("[串口2接收] 长度:%d 数据: ", len);
            // for (int i = 0; i < len; i++) {
            //     printf("%02X ", data[i]);
            // }
            // printf("\n");
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    free(data);
}

// =====================================================================================
// --- 平衡控制任务 (Balance Control Task) ---
// =====================================================================================


// 基于Pitch角度的平衡控制任务
static void balance_control_task(void *pvParameters) {
    // 控制周期设置
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 5ms, 200Hz
    const TickType_t safeFrequency = (xFrequency > 0) ? xFrequency : 1;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // 控制状态变量
    float current_pitch = 0.0f;
    float pitch_error = 0.0f;
    bool motor_should_run = false;
    bool last_in_tolerance = true;  // 上一次是否在容差区内
    TickType_t out_of_tolerance_start_time = 0;  // 离开容差区的时间
    const TickType_t enable_delay = pdMS_TO_TICKS(500);  // 500ms延时 - 恢复你的原始参数
    
    // PITCH角变化检测变量
    float pitch_at_check_start = 0.0f;  // 开始检测时的pitch值
    TickType_t pitch_check_start_time = 0;  // 开始检测的时间
    TickType_t last_motor_restart_time = 0;  // 上次电机重启的时间
    TickType_t last_restart_velocity_time = 0;  // 重启后速度指令发送时间
    const float pitch_significant_change = 1.0f;  // pitch角显著变化阈值 (1度) - 恢复你的原始参数
    const TickType_t pitch_check_timeout = pdMS_TO_TICKS(500);  // 500ms检查超时 - 恢复你的原始参数
    const TickType_t motor_restart_cooldown = pdMS_TO_TICKS(1500);  // 1.5s电机重启冷却时间 - 恢复你的原始参数
    const TickType_t restart_velocity_period = pdMS_TO_TICKS(500);  // 重启后500ms高频发送期 - 恢复你的原始参数
    const TickType_t restart_velocity_freq = pdMS_TO_TICKS(10);  // 重启后10ms发送频率 - 恢复你的原始参数
    
    // 速度指令发送计时器
    TickType_t xLastVelocityTime = xTaskGetTickCount();
    const TickType_t xVelocityFrequency = pdMS_TO_TICKS(10); // 10ms发送一次速度指令

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, safeFrequency); // 固定频率运行 (200Hz)

        // 1. 数据更新：通过KY9250模块获取pitch角度
        current_pitch = ky9250_get_pitch(g_ky9250_handle);

        // 初始化检测起始点（如果还未初始化）
        if (pitch_check_start_time == 0) {
            pitch_check_start_time = xTaskGetTickCount();
            pitch_at_check_start = current_pitch;
        }

        // 2. 控制逻辑：基于最新的PITCH角数据执行
        pitch_error = current_pitch - TARGET_PITCH_ANGLE;
        bool in_tolerance = (fabs(pitch_error) <= PITCH_TOLERANCE);
        
        if (in_tolerance) {
            // 在容差区内：立即失能并设置速度为0
            if (motor_control_is_enabled(g_motor_controller)) {
                // 连续发送三次失能指令确保发出
                for (int i = 0; i < 3; i++) {
                    motor_control_enable(g_motor_controller, false);
                    vTaskDelay(pdMS_TO_TICKS(10)); // 延时10ms
                }
                motor_control_set_velocity(g_motor_controller, 0.0f);
            }
            motor_should_run = false;
            last_in_tolerance = true;
        } else {
            // 在容差区外
            if (last_in_tolerance) {
                // 刚离开容差区，记录时间并重置pitch变化检测
                out_of_tolerance_start_time = xTaskGetTickCount();
                pitch_check_start_time = xTaskGetTickCount();
                pitch_at_check_start = current_pitch;
                last_in_tolerance = false;
                printf("[信息] PITCH离开平衡区(%.2f°)，开始计时\n", current_pitch);
            }
            
            TickType_t current_time = xTaskGetTickCount();
            
            // 检查在容差外是否超过500ms且pitch角变化不超过1度（同时检查冷却时间）
            if (current_time - pitch_check_start_time >= pitch_check_timeout && 
                current_time - last_motor_restart_time >= motor_restart_cooldown) {
                float pitch_change_amount = fabs(current_pitch - pitch_at_check_start);
                if (pitch_change_amount <= pitch_significant_change) {
                    printf("[警告] 平衡外超过500ms且PITCH角变化仅%.2f°(≤1°)，清除电机错误并立即重启\n", pitch_change_amount);
                    
                    // 清除电机错误
                    motor_control_clear_errors(g_motor_controller);
                    
                    // 重置状态，重新开始容差外规则
                    out_of_tolerance_start_time = current_time;
                    last_motor_restart_time = current_time;  // 记录重启时间
                    last_restart_velocity_time = current_time;  // 记录重启后速度发送时间
                    
                    // 立即使能电机发送速度指令
                    for (int i = 0; i < 3; i++) {
                        motor_control_enable(g_motor_controller, true);
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }
                    
                    // 计算并发送速度指令（pitch正值前倾，需要后退）
                    float motor_speed = (pitch_error > 0) ? -MOTOR_FIXED_SPEED : MOTOR_FIXED_SPEED;
                    motor_control_set_velocity(g_motor_controller, motor_speed);
                    
                    motor_should_run = true;
                } else {
                    printf("[信息] 500ms内PITCH角变化%.2f°(>1°)，重置检测计时器\n", pitch_change_amount);
                }
                
                // 重置检测计时器
                pitch_check_start_time = current_time;
                pitch_at_check_start = current_pitch;
            } else if (current_time - out_of_tolerance_start_time >= enable_delay) {
                // 延时500ms后，发送使能指令并设置速度
                if (!motor_control_is_enabled(g_motor_controller)) {
                    // 连续发送三次使能指令确保发出
                    for (int i = 0; i < 3; i++) {
                        motor_control_enable(g_motor_controller, true);
                        vTaskDelay(pdMS_TO_TICKS(10)); // 延时10ms
                    }
                }
                
                motor_should_run = true;
            } else {
                // 延时期间保持失能状态
                if (motor_control_is_enabled(g_motor_controller)) {
                    // 连续发送三次失能指令确保发出
                    for (int i = 0; i < 3; i++) {
                        motor_control_enable(g_motor_controller, false);
                        vTaskDelay(pdMS_TO_TICKS(10)); // 延时10ms
                    }
                    motor_control_set_velocity(g_motor_controller, 0.0f);
                }
                motor_should_run = false;
            }
        }
        
        // 3. 定期发送速度指令
        TickType_t now = xTaskGetTickCount();
        
        // 检查是否在重启后500ms内需要高频发送
        bool in_restart_period = (now - last_motor_restart_time <= restart_velocity_period);
        
        if (motor_should_run) {
            if (in_restart_period) {
                // 重启后500ms内，以10ms频率发送
                if (now - last_restart_velocity_time >= restart_velocity_freq) {
                    float motor_speed = (pitch_error > 0) ? -MOTOR_FIXED_SPEED : MOTOR_FIXED_SPEED;
                    motor_control_set_velocity(g_motor_controller, motor_speed);
                    last_restart_velocity_time = now;
                    printf("[重启期] 高频发送速度指令: %.1f\n", motor_speed);
                }
            } else {
                // 正常情况下，以10ms频率发送
                if (now - xLastVelocityTime >= xVelocityFrequency) {
                    float motor_speed = (pitch_error > 0) ? -MOTOR_FIXED_SPEED : MOTOR_FIXED_SPEED;
                    motor_control_set_velocity(g_motor_controller, motor_speed);
                    xLastVelocityTime = now;
                }
            }
        }
        
        // 4. 定期发送清理错误指令
        static TickType_t xLastClearErrorTime = 0;
        const TickType_t xClearErrorFrequency = pdMS_TO_TICKS(3000); // 3s发送一次清理指令 - 恢复你的原始参数
        if (xTaskGetTickCount() - xLastClearErrorTime >= xClearErrorFrequency) {
            motor_control_clear_errors(g_motor_controller);
            xLastClearErrorTime = xTaskGetTickCount();
        }
        
    }
}

// =====================================================================================
// --- 主程序入口 (Main Entry Point) ---
// =====================================================================================

extern "C" void app_main(void) {
    // 禁用看门狗
    esp_task_wdt_deinit();
    
    printf("--- 系统将在5秒后启动，请保持设备静止 ---\n");
    vTaskDelay(pdMS_TO_TICKS(5000)); // 通电后等待5秒
    printf("--- ESP32 KY9250 平衡控制系统 ---\n"); 
    
    // 初始化KY9250模块
    printf("[信息] 初始化KY9250模块...\n");
    g_ky9250_handle = ky9250_init(KY9250_UART_PORT, KY9250_UART_RXD, KY9250_UART_TXD, 115200);
    if (!g_ky9250_handle) {
        printf("[错误] KY9250模块初始化失败！\n");
        return;
    }
    printf("[成功] KY9250模块初始化成功\n");


    // 初始化电机控制模块
    motor_driver_config_t motor_driver_config = {
        .uart_port = MOTOR_UART_PORT,
        .txd_pin = MOTOR_UART_TXD,
        .rxd_pin = MOTOR_UART_RXD,
        .baud_rate = 115200,
        .buf_size = UART_BUF_SIZE
    };
    
    g_motor_controller = motor_control_init(&motor_driver_config, MOTOR_FIXED_SPEED);
    if (!g_motor_controller) {
        printf("[错误] 电机控制模块初始化失败！\n");
        ky9250_destroy(g_ky9250_handle);
        return;
    }


    // 创建实时显示任务
    xTaskCreate(realtime_display_task, "realtime_display", 4096, NULL, 3, NULL);
    xTaskCreate(uart2_monitor_task, "uart2_monitor", 2048, NULL, 3, NULL);

    // 创建集成平衡控制任务
    xTaskCreate(balance_control_task, "balance_control_task", 4096, NULL, 5, NULL);

    printf("[信息] KY9250平衡系统初始化完成，所有任务已启动\n");
    printf("[信息] 功能说明:\n");
    printf("  - KY9250传感器: 实时9轴数据采集和姿态解析(模块化)\n");
    printf("  - 平衡控制: 当PITCH角度超出%.1f°±%.1f°时，电机以%.1f速度调节\n", TARGET_PITCH_ANGLE, PITCH_TOLERANCE, MOTOR_FIXED_SPEED);
    printf("  - 数据显示: 2Hz实时传感器数据，重点关注PITCH角\n");
}
