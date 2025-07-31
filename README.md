# ESP32平衡车项目（BLE IMU版本）

基于ESP32S3和BLE IMU传感器的智能平衡控制系统，实现基于PITCH角度的精确姿态控制。

## 项目概述

本项目是一个ESP-IDF平衡车控制系统，主要功能：
- 通过BLE连接IMU设备获取实时姿态数据
- 基于PITCH角度进行平衡控制（前后倾倒检测）
- 电机速度自动调节
- 实时状态监控和调试信息显示
- 模块化传感器架构，支持完整9轴数据

## 🔥 **BLE IMU数据处理方法**

### **当前实现架构（无线BLE连接）**

系统采用**BLE无线连接**架构，自动连接指定IMU设备，提供线程安全的数据访问：

#### 1. **BLE IMU模块** (`main/ble_imu.c/h`)

**模块化特点**：
```c
// BLE IMU传感器数据结构（与JY901S兼容）
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

// 线程安全的数据访问接口
bool ble_imu_get_data(ble_imu_handle_t* handle, ble_imu_data_t* data);
float ble_imu_get_roll(ble_imu_handle_t* handle);
float ble_imu_get_pitch(ble_imu_handle_t* handle);  // 主要控制轴
float ble_imu_get_yaw(ble_imu_handle_t* handle);
```

#### 2. **BLE连接任务**
- **目标设备**: MAC地址 `e8:cb:ed:5a:52:8e`
- **自动扫描**: 设备断开后自动重新扫描连接
- **完整数据**: 支持9轴传感器、磁场、四元数等完整IMU数据

#### 3. **平衡控制任务中的PITCH角读取**
位置：`main/main.cpp`
```c
// 🔥 核心：直接读取PITCH角度（平衡车的前后倾倒控制轴）
float current_pitch = ble_imu_get_pitch(g_ble_imu_handle);  // 线程安全获取
float pitch_error = current_pitch - TARGET_PITCH_ANGLE;    // 计算平衡误差
```

#### 4. **实时显示界面**
位置：`main/main.cpp`

**突出PITCH角显示**：
```c
static void realtime_display_task(void *pvParameters) {
    const TickType_t refresh_rate = pdMS_TO_TICKS(500); // 2Hz刷新频率
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        vTaskDelayUntil(&last_wake_time, refresh_rate);
        
        // 获取BLE IMU数据
        ble_imu_data_t sensor_data;
        bool data_valid = ble_imu_get_data(g_ble_imu_handle, &sensor_data);
        
        printf("\\033[2J\\033[H"); // 清屏
        printf("=== ESP32 BLE IMU 平衡控制系统 ===\\n");
        
        if (data_valid) {
            // 显示姿态数据，重点关注Pitch角
            printf("Roll: %6.2f°  **PITCH: %6.2f°**  Yaw: %6.2f°\\n",
                   sensor_data.roll, sensor_data.pitch, sensor_data.yaw);
            printf("平衡控制: PITCH=%.2f° (目标%.1f°±%.1f°)\\n", 
                   sensor_data.pitch, TARGET_PITCH_ANGLE, PITCH_TOLERANCE);
        }
        
        fflush(stdout);
    }
}
```

### **优化特点**

1. **📡 无线连接**：BLE自动连接，无需UART线缆
2. **🔒 线程安全**：使用互斥锁保护共享数据，避免数据竞争
3. **📐 正确控制轴**：使用PITCH角进行平衡控制，检测前后倾倒
4. **⚡ 高响应速度**：BLE实时数据传输
5. **📊 完整数据**：支持9轴传感器、磁场、四元数等完整IMU数据
6. **🔄 自动重连**：连接断开后自动扫描重连

### **数据流程图**
```
BLE IMU设备 → BLE连接 → 数据解析 → 互斥锁保护 → 平衡控制 → 实时显示
   高频输出    无线传输   完整数据    线程安全      PITCH角     2Hz刷新
    ↓           ↓          ↓           ↓            ↓           ↓
   发送       自动重连   9轴+四元数   共享数据      误差计算    可视化界面
```

## 硬件配置

### 设备信息
```c
// BLE IMU 目标设备地址
#define TARGET_DEVICE_MAC  "e8:cb:ed:5a:52:8e"

// 电机控制 (UART2)  
#define MOTOR_UART_TXD      GPIO_NUM_13  // ESP32 → 电机控制器
#define MOTOR_UART_RXD      GPIO_NUM_12  // ESP32 ← 电机控制器
```

### 通信参数
- **连接方式**: BLE无线连接
- **目标设备**: MAC地址 `e8:cb:ed:5a:52:8e`
- **数据内容**: 9轴传感器 + 磁场 + 四元数
- **电机通信**: UART2，115200波特率

## 软件架构

### 控制参数
```c
const float TARGET_PITCH_ANGLE = 0.0f;    // 目标pitch角度（平衡位置）
const float PITCH_TOLERANCE = 10.0f;      // pitch角度容差（±10度）
const float MOTOR_FIXED_SPEED = 30.0f;    // 电机固定转速
```

### 主要任务
1. **BLE连接任务**: BLE设备扫描和连接管理
2. **balance_control_task**: 平衡控制主循环（优先级5）
3. **realtime_display_task**: 实时显示界面任务（优先级3，2Hz）
4. **uart2_monitor_task**: 电机数据监控任务（优先级3）

### 文件结构
```
main/
├── main.cpp          # 主程序和平衡控制逻辑
├── ble_imu.c         # BLE IMU传感器模块实现
├── ble_imu.h         # BLE IMU传感器模块头文件
├── motor_control.cpp # 电机控制模块实现
├── motor_control.h   # 电机控制模块头文件
├── old/              # 旧版本传感器代码（已废弃）
│   ├── jy901s.c      # 旧JY901S UART传感器代码
│   ├── jy901s.h      
│   ├── gy25t.cpp     # 旧GY25T传感器代码
│   ├── gy25t.h       
│   ├── ky9250.c      # 旧KY9250传感器代码
│   └── ky9250.h      
└── CMakeLists.txt    # 构建配置

ble_imu_connector/    # 独立的BLE IMU连接器项目
├── main/
│   ├── main.c        # BLE连接器核心程序
│   └── CMakeLists.txt
├── README.md         # BLE连接器说明
└── 使用说明.md       # 详细使用文档
```

## 编译和运行

### 环境要求
- ESP-IDF v4.4+
- CMake 3.16+
- 支持ESP32S3的开发环境

### 编译步骤
```bash
# 1. 设置ESP-IDF环境
. $HOME/esp/esp-idf/export.sh

# 2. 配置项目
idf.py set-target esp32s3

# 3. 编译项目
idf.py build

# 4. 烧录到设备
idf.py -p /dev/ttyUSB0 flash

# 5. 监控串口输出
idf.py -p /dev/ttyUSB0 monitor
```

## 控制逻辑

### 平衡算法
- **核心原理**: 基于PITCH角进行平衡控制（前后倾倒检测）
- **平衡区间**: PITCH角在0°±10°范围内时，电机停止
- **失衡响应**: PITCH角超出容差范围时，启动电机调节
- **控制方向**: 
  - PITCH > 0°（前倾）：电机调节
  - PITCH < 0°（后倾）：电机反向调节

### BLE连接管理
- **自动扫描**: 系统启动时自动扫描目标设备
- **自动重连**: 连接断开后自动重新扫描连接
- **数据验证**: 确保接收数据的完整性和有效性

### 安全机制
- 系统启动延时，确保BLE连接稳定
- 连接状态监控和错误处理
- 电机使能/失能确认机制
- 看门狗禁用，避免调试中断

## 调试和监控

### 实时状态显示
程序运行时会实时显示：
- BLE连接状态
- **PITCH角度**（重点标记）
- Roll和Yaw角度
- 9轴传感器数据（加速度、角速度、磁场）
- 四元数数据
- 平衡控制状态
- BLE接收统计

### 串口输出示例
```
=== ESP32 BLE IMU 平衡控制系统 ===
BLE状态: 已连接 e8:cb:ed:5a:52:8e
Roll: -1.23°  **PITCH: 2.45°**  Yaw: 89.12°
加速度: X=0.012, Y=-0.045, Z=9.807 g
角速度: X=1.234, Y=-2.567, Z=0.890 °/s
平衡控制: PITCH=2.45° (目标0.0°±10.0°)
```

## 协议说明

### BLE IMU数据格式
- **连接方式**: BLE GATT
- **目标设备**: MAC `e8:cb:ed:5a:52:8e`
- **数据内容**: 
  - 3轴欧拉角（Roll、Pitch、Yaw）
  - 3轴加速度（acc_x、acc_y、acc_z）
  - 3轴角速度（gyro_x、gyro_y、gyro_z）
  - 3轴磁场（mag_x、mag_y、mag_z）
  - 四元数（q0、q1、q2、q3）

## 注意事项

1. **设备配对**: 确保IMU设备在可连接状态
2. **控制轴选择**: 平衡车使用PITCH角检测前后倾倒
3. **电源要求**: 确保ESP32S3和IMU设备供电稳定
4. **BLE距离**: 保持设备在BLE有效通信距离内
5. **角度范围**: PITCH角度范围为-180°到+180°
6. **连接稳定性**: BLE连接可能受环境干扰影响

## 更新日志

### 当前版本 (v2.0 - BLE版本)
- **传感器模块**: 改用BLE IMU传感器，MAC地址 `e8:cb:ed:5a:52:8e`
- **控制方式**: 基于PITCH角的平衡控制（从ROLL改为PITCH）
- **连接方式**: BLE无线连接，支持自动重连
- **数据完整性**: 支持9轴传感器、磁场、四元数等完整IMU数据
- **目标平台**: ESP32S3

### 历史版本
- **v1.x**: 使用JY901S UART连接，基于ROLL角控制（代码保存在old/目录）
- **v0.x**: 使用GY25T、KY9250等传感器的早期版本

## 许可证

本项目采用开源许可证，详情请参考项目根目录的LICENSE文件。

---

🤖 Generated with [Claude Code](https://claude.ai/code)