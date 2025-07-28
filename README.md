# ESP32平衡车项目

基于ESP32和KY9250九轴传感器的智能平衡控制系统，实现基于PITCH角度的精确姿态控制。

## 项目概述

本项目是一个ESP-IDF平衡车控制系统，主要功能：
- 通过KY9250九轴传感器获取实时姿态数据
- 基于PITCH角度进行平衡控制（平衡车的正确控制轴）
- 电机速度自动调节
- 实时状态监控和调试信息显示
- 模块化传感器架构，易于维护和扩展

## 🔥 **KY9250数据处理方法** 

### **当前实现架构（模块化设计）**

系统采用**模块化架构**，将传感器处理独立封装，提供线程安全的数据访问：

#### 1. **KY9250模块** (`main/ky9250.c/h`)

**模块化特点**：
```c
// 完整的传感器数据结构
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

// 线程安全的数据访问接口
bool ky9250_get_data(ky9250_handle_t* handle, ky9250_data_t* data);
float ky9250_get_pitch(ky9250_handle_t* handle);  // 直接获取PITCH角
float ky9250_get_roll(ky9250_handle_t* handle);
float ky9250_get_yaw(ky9250_handle_t* handle);
```

#### 2. **数据解析任务** (`ky9250_parse_task`)
位置：`main/ky9250.c:75-125`

**核心处理循环**：
```c
void ky9250_parse_task(void* pvParameters) {
    uint8_t ubuf[52];
    uint8_t ucRxCnt = 0;
    uint8_t temp_byte;
    
    while (1) {
        // 读取UART数据
        int len = uart_read_bytes(handle->uart_port, &temp_byte, 1, 10 / portTICK_PERIOD_MS);
        if (len > 0) {
            handle->total_bytes_received++;
            
            ubuf[ucRxCnt++] = temp_byte;
            
            // 检查包头 0x50
            if (ubuf[0] != 0x50) {
                ucRxCnt = 0;
                continue;
            }
            
            // 等待完整52字节数据包
            if (ucRxCnt < 52) continue;
            
            // 解析数据包并更新共享数据（线程安全）
            ky9250_data_t temp_data = {0};
            if (parse_ky9250_packet(ubuf, &temp_data)) {
                if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    handle->sensor_data = temp_data;
                    handle->sensor_data.packet_count++;
                    xSemaphoreGive(handle->data_mutex);
                }
            }
            
            ucRxCnt = 0;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
```

#### 3. **平衡控制任务中的PITCH角读取**
位置：`main/main.cpp:137-138`
```c
// 🔥 核心：直接读取PITCH角度（平衡车的正确控制轴）
current_pitch = ky9250_get_pitch(g_ky9250_handle);  // 线程安全获取
pitch_error = current_pitch - TARGET_PITCH_ANGLE;   // 计算平衡误差
```

#### 4. **实时显示界面**
位置：`main/main.cpp:48-78`

**突出PITCH角显示**：
```c
static void realtime_display_task(void *pvParameters) {
    const TickType_t refresh_rate = pdMS_TO_TICKS(500); // 2Hz刷新频率
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        vTaskDelayUntil(&last_wake_time, refresh_rate);
        
        // 获取KY9250数据
        ky9250_data_t sensor_data;
        bool data_valid = ky9250_get_data(g_ky9250_handle, &sensor_data);
        
        printf("\\033[2J\\033[H"); // 清屏
        printf("=== ESP32 KY9250 平衡控制系统 ===\\n");
        
        if (data_valid) {
            // 显示姿态数据，重点关注Pitch角
            printf("**PITCH: %6.2f°**  Roll: %6.2f°  Yaw: %6.2f°\\n",
                   sensor_data.pitch, sensor_data.roll, sensor_data.yaw);
            printf("平衡控制: PITCH=%.2f° (目标%.1f°±%.1f°)\\n", 
                   sensor_data.pitch, TARGET_PITCH_ANGLE, PITCH_TOLERANCE);
        }
        
        fflush(stdout);
    }
}
```

### **优化特点**

1. **🏗️ 模块化架构**：传感器功能独立封装，便于维护和扩展
2. **🔒 线程安全**：使用互斥锁保护共享数据，避免数据竞争
3. **📐 正确控制轴**：使用PITCH角进行平衡控制，而非YAW角
4. **⚡ 高响应速度**：200Hz控制频率，1ms数据处理周期
5. **📊 完整数据**：支持9轴数据和四元数，可扩展更复杂算法
6. **🛡️ 数据验证**：校验位检查，确保数据完整性

### **数据流程图**
```
KY9250传感器 → UART接收 → 解析任务 → 互斥锁保护 → 平衡控制 → 实时显示
   200Hz        1ms      独立任务     线程安全      PITCH角     2Hz刷新
    ↓           ↓          ↓           ↓            ↓           ↓
   发送       快速接收   52字节包    共享数据      误差计算    可视化界面
```

## 硬件配置

### 引脚连接
```c
// KY9250传感器连接 (UART1)
#define KY9250_UART_RXD     GPIO_NUM_11  // ESP32 ← KY9250 TX
#define KY9250_UART_TXD     GPIO_NUM_10  // ESP32 → KY9250 RX

// 电机控制 (UART2)  
#define MOTOR_UART_TXD      GPIO_NUM_13  // ESP32 → 电机控制器
#define MOTOR_UART_RXD      GPIO_NUM_12  // ESP32 ← 电机控制器
```

### 通信参数
- **波特率**: 115200 bps
- **数据位**: 8位
- **停止位**: 1位
- **校验位**: 无
- **传感器输出频率**: 200Hz（固定）
- **数据包格式**: 52字节，包含9轴数据和四元数

## 软件架构

### 控制参数
```c
const float TARGET_PITCH_ANGLE = 0.0f;    // 目标pitch角度（平衡位置）
const float PITCH_TOLERANCE = 3.0f;       // pitch角度容差（±3度）
const float MOTOR_FIXED_SPEED = 15.0f;    // 电机固定转速
```

### 主要任务
1. **ky9250_parse_task**: KY9250数据解析任务（优先级4）
2. **balance_control_task**: 平衡控制主循环（优先级5，200Hz）
3. **realtime_display_task**: 实时显示界面任务（优先级3，2Hz）
4. **uart2_monitor_task**: 电机数据监控任务（优先级3）

### 文件结构
```
main/
├── main.cpp          # 主程序和平衡控制逻辑
├── ky9250.c          # KY9250传感器模块实现
├── ky9250.h          # KY9250传感器模块头文件
├── motor_control.cpp # 电机控制模块实现
├── motor_control.h   # 电机控制模块头文件
└── CMakeLists.txt    # 构建配置
```

## 编译和运行

### 环境要求
- ESP-IDF v4.4+
- CMake 3.16+
- 支持C++11的编译器

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
- **平衡区间**: PITCH角在0°±3°范围内时，电机停止
- **失衡响应**: PITCH角超出容差范围时，延时500ms后启动电机调节
- **控制方向**: 
  - PITCH > 0°（前倾）：电机后退（负速度）
  - PITCH < 0°（后倾）：电机前进（正速度）

### 智能重启机制
- **变化检测**: 500ms内PITCH角变化≤1°时触发电机重启
- **冷却保护**: 1.5秒重启冷却时间，防止频繁操作
- **高频恢复**: 重启后500ms内以10ms频率高频发送控制指令
- **错误清理**: 每3秒自动清理电机错误状态

### 安全机制
- 系统启动5秒延时，确保传感器稳定
- 连续错误检测和重置机制
- 电机使能/失能三次确认发送
- 看门狗禁用，避免调试中断

## 调试和监控

### 实时状态显示
程序运行时会实时显示：
- **PITCH角度**（重点标记）
- Roll和Yaw角度
- 9轴加速度数据
- 传感器包计数
- 平衡控制状态
- UART接收统计

### 串口输出示例
```
=== ESP32 KY9250 平衡控制系统 ===
UART1: 接收1542字节
**PITCH: -1.23°**  Roll: 0.45°  Yaw: 2.78°
Acc: 0.02 -0.98 0.15 | 包#42
平衡控制: PITCH=-1.23° (目标0.0°±3.0°)
```

## 协议说明

### KY9250数据包格式
- **包头**: 0x50
- **数据长度**: 52字节
- **校验**: 第49-51字节校验值为128
- **数据内容**: 
  - 字节1-9: 陀螺仪数据 (gx,gy,gz)
  - 字节10-18: 加速度数据 (ax,ay,az)
  - 字节19-27: 磁力计数据 (mx,my,mz)
  - 字节28-36: 欧拉角 (roll,pitch,yaw)
  - 字节37-48: 四元数 (q0,q1,q2,q3)

## 注意事项

1. **启动顺序**: 确保在系统启动的5秒内保持设备静止
2. **控制轴选择**: 平衡车必须使用PITCH角，不是YAW角
3. **电源要求**: 确保ESP32和传感器供电稳定
4. **串口冲突**: 避免多个程序同时占用串口
5. **角度范围**: PITCH角度范围为-180°到+180°
6. **实时性能**: 控制循环运行在200Hz，确保实时响应

## 更新日志

### 2025-07-28 （当前版本）
- **🔄 重构平衡控制系统**: 实现KY9250模块化并改用PITCH角控制
  - **传感器模块化**: 创建独立的ky9250.c/h模块，提供线程安全接口
    - 位置：`main/ky9250.c`, `main/ky9250.h`
    - 功能：完整的9轴数据解析，互斥锁保护，统计信息
  - **控制轴修正**: 从YAW角改为PITCH角进行平衡控制
    - 原因：平衡车应控制前后倾倒（PITCH），而非左右旋转（YAW）
    - 位置：`main/main.cpp:137-148`
  - **参数保持**: 保留所有原始调试参数（3°容差，15速度，500ms延时等）
    - 确保已调试好的控制特性不变
  - **显示优化**: 界面突出显示PITCH角，便于调试监控
    - 位置：`main/main.cpp:65`

### 2025-07-25
- **平衡控制算法重大优化**: 重构YAW角变化检测和电机重启机制
  - **智能变化检测**: 改为基于时间窗口(500ms)和变化幅度(1°)的综合判断
  - **电机重启冷却机制**: 加入1.5秒冷却时间，防止频繁重启
  - **重启后高频控制**: 重启后500ms内以10ms频率高频发送速度指令
  - **定期错误清理**: 新增每3秒自动清理电机错误的机制

### 2025-07-22
- **平衡控制优化**: 将电机调整方向取反，改善平衡响应效果
  - 控制逻辑：PITCH误差正值时电机反向转动，误差负值时正向转动

## 许可证

本项目采用开源许可证，详情请参考项目根目录的LICENSE文件。

---

🤖 Generated with [Claude Code](https://claude.ai/code)