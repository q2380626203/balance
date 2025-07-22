#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "gy25t.h"
#include "motor_control.h"

// --- 陀螺仪频率配置 ---
// 注意：这需要GY-25T模块支持频率切换指令
#define GYRO_TARGET_FREQ_200HZ

// =====================================================================================
// --- 硬件与软件配置区 (Hardware & Software Configuration) ---
// =====================================================================================

#define GYRO_UART_PORT      UART_NUM_1
#define GYRO_UART_TXD       GPIO_NUM_11
#define GYRO_UART_RXD       GPIO_NUM_10

#define MOTOR_UART_PORT     UART_NUM_2
#define MOTOR_UART_TXD      GPIO_NUM_13
#define MOTOR_UART_RXD      GPIO_NUM_12

#define UART_BUF_SIZE       (1024)

// --- 简化的Yaw角度控制参数 ---
const float TARGET_YAW_ANGLE = 0.0f;      // 目标yaw角度 (度)
const float YAW_TOLERANCE = 3.0f;       // yaw角度容差 (±5度)
const float MOTOR_FIXED_SPEED = 15.0f;    // 电机固定转动速度

// --- 全局模块句柄 (Global Module Handles) ---
static gy25t_handle_t* g_gyro_handle = NULL;
static motor_controller_t* g_motor_controller = NULL;

// =====================================================================================
// --- 实时显示任务 (Real-time Display Task) ---
// =====================================================================================

// 固定显示的实时数据任务
static void realtime_display_task(void *pvParameters) {
    const TickType_t refresh_rate = pdMS_TO_TICKS(100); // 10Hz刷新频率
    TickType_t last_wake_time = xTaskGetTickCount();
    
    // 数据监控变量
    static float last_recorded_yaw = 0.0f;
    static uint32_t total_parsed_count = 0;
    
    while (1) {
        vTaskDelayUntil(&last_wake_time, refresh_rate);
        
        // 获取当前数据
        float current_yaw = g_last_yaw;
        
        // 清屏并显示固定界面
        printf("\033[2J\033[H"); // 清屏并移动光标到左上角
        printf("╔════════════════════════════════════════════════════════════╗\n");
        printf("║                ESP32 GY25T 平衡控制系统                   ║\n");
        printf("╠════════════════════════════════════════════════════════════╣\n");
        printf("║ 实时数据显示                                             ║\n");
        printf("║                                                          ║\n");
        printf("╚════════════════════════════════════════════════════════════╝\n");
        printf("\n");
        
        // 检测数据变化并更新解析信息
        if (fabs(current_yaw - last_recorded_yaw) > 0.01f) { // yaw变化超过0.01度
            total_parsed_count++;
            last_recorded_yaw = current_yaw;
        }
        
        // 固定显示解析信息
        printf("[成功解析#%04lu] YAW角度更新: %.2f° -> %.2f° (变化%.2f°) - 数据已处理\n", 
               total_parsed_count, last_recorded_yaw, current_yaw, current_yaw - last_recorded_yaw);
        
        // 显示当前YAW数据
        printf("当前YAW角度: %.2f°\n", current_yaw);
        
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


// 简化的基于yaw角度的平衡控制任务
static void balance_control_task(void *pvParameters) {
    // 控制周期设置
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 5ms, 200Hz
    // 确保xFrequency至少为1，防止断言失败
    const TickType_t safeFrequency = (xFrequency > 0) ? xFrequency : 1;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // 删除了原有的打印变量，由实时显示任务负责

    // 控制状态变量
    float current_yaw = 0.0f;
    float yaw_error = 0.0f;
    bool motor_should_run = false;
    bool last_in_tolerance = true;  // 上一次是否在容差区内
    TickType_t out_of_tolerance_start_time = 0;  // 离开容差区的时间
    const TickType_t enable_delay = pdMS_TO_TICKS(500);  // 500ms延时
    
    // YAW角变化检测变量
    float last_yaw = 0.0f;  // 上一次记录的yaw值
    TickType_t last_yaw_change_time = 0;  // 上次yaw角发生变化的时间
    const float yaw_change_threshold = 0.01f;  // yaw角变化检测阈值 (度)
    const TickType_t yaw_no_change_timeout = pdMS_TO_TICKS(500);  // 500ms无yaw变化超时
    
    // 速度指令发送计时器
    TickType_t xLastVelocityTime = xTaskGetTickCount();
    const TickType_t xVelocityFrequency = pdMS_TO_TICKS(10); // 10ms发送一次速度指令
    

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, safeFrequency); // 固定频率运行 (200Hz)

        // 1. 数据更新：直接读取全局YAW变量
        current_yaw = g_last_yaw;

        // 检测YAW角是否发生变化
        if (fabs(current_yaw - last_yaw) > yaw_change_threshold) {
            last_yaw_change_time = xTaskGetTickCount();
            last_yaw = current_yaw;
        }

        // 2. 控制逻辑：始终基于最新的YAW角数据执行
        yaw_error = current_yaw - TARGET_YAW_ANGLE;
        bool in_tolerance = (fabs(yaw_error) <= YAW_TOLERANCE);
        
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
                // 刚离开容差区，记录时间并初始化yaw变化检测
                out_of_tolerance_start_time = xTaskGetTickCount();
                last_yaw_change_time = xTaskGetTickCount();
                last_yaw = current_yaw;
                last_in_tolerance = false;
                printf("[信息] 离开容差区，开始计时\n");
            }
            
            TickType_t current_time = xTaskGetTickCount();
            
            // 检查在容差外是否超过500ms没有yaw角变化
            if (current_time - last_yaw_change_time >= yaw_no_change_timeout) {
                printf("[警告] 容差外超过500ms无YAW角变化，清除电机错误并立即重启\n");
                
                // 清除电机错误
                motor_control_clear_errors(g_motor_controller);
                
                // 重置状态，重新开始容差外规则
                out_of_tolerance_start_time = current_time;
                last_yaw_change_time = current_time;
                
                // 立即使能电机三次并发送速度指令
                printf("[信息] 清理完成，立即使能电机并发送速度指令\n");
                for (int i = 0; i < 3; i++) {
                    motor_control_enable(g_motor_controller, true);
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                
                // 计算并发送速度指令（方向取反）
                float motor_speed = (yaw_error > 0) ? -MOTOR_FIXED_SPEED : MOTOR_FIXED_SPEED;
                motor_control_set_velocity(g_motor_controller, motor_speed);
                
                motor_should_run = true;
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
        
        // 3. 定期发送速度指令 (10ms频率)
        if (motor_should_run && (xTaskGetTickCount() - xLastVelocityTime >= xVelocityFrequency)) {
            float motor_speed = (yaw_error > 0) ? -MOTOR_FIXED_SPEED : MOTOR_FIXED_SPEED;
            motor_control_set_velocity(g_motor_controller, motor_speed);
            xLastVelocityTime = xTaskGetTickCount();
        }
        
    }
}

// =====================================================================================
// --- 主程序入口 (Main Entry Point) ---
// =====================================================================================

extern "C" void app_main(void) {
    printf("--- 系统将在5秒后启动，请保持设备静止 ---\n");
    vTaskDelay(pdMS_TO_TICKS(5000)); // 通电后等待5秒
    printf("--- 简化Yaw角度平衡控制系统 ---\n"); 
    
    // 初始化GY-25T陀螺仪模块
    gy25t_config_t gyro_config = {
        .uart_port = GYRO_UART_PORT,
        .txd_pin = GYRO_UART_TXD,
        .rxd_pin = GYRO_UART_RXD,
        .baud_rate = 115200,
        .buf_size = UART_BUF_SIZE
    };
    
    g_gyro_handle = gy25t_init(&gyro_config);
    if (!g_gyro_handle) {
        printf("[错误] GY-25T 陀螺仪模块初始化失败！\n");
        return;
    }


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
        gy25t_deinit(g_gyro_handle);
        return;
    }


    // 创建实时显示任务
    xTaskCreate(realtime_display_task, "realtime_display", 4096, NULL, 3, NULL);
    xTaskCreate(uart2_monitor_task, "uart2_monitor", 2048, NULL, 3, NULL);

    // 创建集成平衡控制任务
    xTaskCreate(balance_control_task, "balance_control_task", 4096, NULL, 5, NULL);

    printf("[信息] 简化Yaw角度平衡系统初始化完成，所有任务已启动\n");
    printf("[信息] 功能说明: 当yaw角度超出±%.1f°范围时，电机将以固定速度%.1f转动进行调节\n", YAW_TOLERANCE, MOTOR_FIXED_SPEED);
}
