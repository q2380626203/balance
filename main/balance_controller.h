#pragma once

#include "shared_data.h"

#ifdef __cplusplus
extern "C" {
#endif

// =====================================================================================
// --- 平衡控制器句柄 ---
// =====================================================================================

typedef struct balance_controller_handle balance_controller_handle_t;

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