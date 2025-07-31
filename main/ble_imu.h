/**
 * @file ble_imu.h
 * @brief BLE IMU 连接器模块 - 替换JY901S的蓝牙IMU传感器接口
 * 
 * 提供与JY901S兼容的接口，通过BLE连接远程IMU设备获取欧拉角数据
 */

#ifndef BLE_IMU_H
#define BLE_IMU_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// IMU数据结构 - 与JY901S结构保持兼容
typedef struct {
    float roll;       // 滚转角X (°) - 对应angle_y
    float pitch;      // 俯仰角Y (°) - 对应angle_x  
    float yaw;        // 偏航角Z (°) - 对应angle_z
    
    // 扩展数据
    float acc_x, acc_y, acc_z;        // 加速度 (g)
    float gyro_x, gyro_y, gyro_z;     // 角速度 (°/s)
    float mag_x, mag_y, mag_z;        // 磁场
    float q0, q1, q2, q3;             // 四元数
    
    bool is_valid;                    // 数据有效性
    uint32_t packet_count;            // 包计数器
    uint32_t timestamp;               // 时间戳
} ble_imu_data_t;

// BLE IMU句柄类型
typedef struct ble_imu_handle ble_imu_handle_t;

/**
 * @brief 初始化BLE IMU模块
 * @return BLE IMU句柄，失败返回NULL
 */
ble_imu_handle_t* ble_imu_init(void);

/**
 * @brief 销毁BLE IMU模块
 * @param handle BLE IMU句柄
 */
void ble_imu_destroy(ble_imu_handle_t* handle);

/**
 * @brief 获取BLE IMU数据（线程安全）
 * @param handle BLE IMU句柄
 * @param data 输出数据结构
 * @return true 数据有效，false 数据无效
 */
bool ble_imu_get_data(ble_imu_handle_t* handle, ble_imu_data_t* data);

/**
 * @brief 获取ROLL角度（与JY901S兼容接口）
 * @param handle BLE IMU句柄
 * @return ROLL角度值（度）
 */
float ble_imu_get_roll(ble_imu_handle_t* handle);

/**
 * @brief 获取PITCH角度
 * @param handle BLE IMU句柄
 * @return PITCH角度值（度）
 */
float ble_imu_get_pitch(ble_imu_handle_t* handle);

/**
 * @brief 获取YAW角度
 * @param handle BLE IMU句柄
 * @return YAW角度值（度）
 */
float ble_imu_get_yaw(ble_imu_handle_t* handle);

/**
 * @brief 检查BLE连接状态
 * @param handle BLE IMU句柄
 * @return true 已连接，false 未连接
 */
bool ble_imu_is_connected(ble_imu_handle_t* handle);

/**
 * @brief 获取接收字节数统计
 * @param handle BLE IMU句柄
 * @return 累计接收字节数
 */
uint32_t ble_imu_get_bytes_received(ble_imu_handle_t* handle);

#ifdef __cplusplus
}
#endif

#endif // BLE_IMU_H