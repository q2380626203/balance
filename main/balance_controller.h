#pragma once

#include "shared_data.h"

#ifdef __cplusplus
extern "C" {
#endif

// =====================================================================================
// --- PID控制器结构体 ---
// =====================================================================================

typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float integral;     // 积分项
    float prev_error;   // 上一次误差
    float output_min;   // 输出最小值
    float output_max;   // 输出最大值
    float integral_min; // 积分限幅最小值
    float integral_max; // 积分限幅最大值
} pid_controller_t;

// =====================================================================================
// --- 平衡控制器句柄 ---
// =====================================================================================

typedef struct balance_controller_handle balance_controller_handle_t;

// =====================================================================================
// --- PID控制器函数 ---
// =====================================================================================

/**
 * @brief 初始化PID控制器
 */
void pid_init(pid_controller_t* pid, float kp, float ki, float kd, float output_min, float output_max);

/**
 * @brief 更新PID控制器并计算输出
 * @param pid PID控制器句柄
 * @param error 当前误差
 * @param dt 时间间隔(秒)
 * @return PID输出值
 */
float pid_update(pid_controller_t* pid, float error, float dt);

/**
 * @brief 重置PID控制器状态
 */
void pid_reset(pid_controller_t* pid);

// =====================================================================================
// --- 公共接口函数 ---
// =====================================================================================

/**
 * @brief 初始化平衡控制器
 * @param shared_data 共享数据句柄
 * @return 平衡控制器句柄，失败返回NULL
 */
balance_controller_handle_t* balance_controller_init(shared_data_t* shared_data);

/**
 * @brief 启动平衡控制任务
 * @param handle 平衡控制器句柄
 * @return true成功，false失败
 */
bool balance_controller_start(balance_controller_handle_t* handle);

/**
 * @brief 停止平衡控制任务
 * @param handle 平衡控制器句柄
 * @return true成功，false失败
 */
bool balance_controller_stop(balance_controller_handle_t* handle);

/**
 * @brief 销毁平衡控制器
 * @param handle 平衡控制器句柄
 */
void balance_controller_destroy(balance_controller_handle_t* handle);

/**
 * @brief 检查平衡控制器是否正在运行
 * @param handle 平衡控制器句柄
 * @return true正在运行，false未运行
 */
bool balance_controller_is_running(balance_controller_handle_t* handle);

#ifdef __cplusplus
}
#endif