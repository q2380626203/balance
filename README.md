# ESP32平衡车项目

基于ESP32和JY901S九轴传感器的智能平衡控制系统，实现基于ROLL角度的精确姿态控制。

## 项目概述

本项目是一个ESP-IDF平衡车控制系统，主要功能：
- 通过JY901S九轴传感器获取实时姿态数据
- 基于ROLL角度进行平衡控制（左右倾倒检测）
- 电机速度自动调节
- 实时状态监控和调试信息显示
- 模块化传感器架构，易于维护和扩展

## 🔥 **JY901S数据处理方法** 

### **当前实现架构（模块化设计）**

系统采用**模块化架构**，将传感器处理独立封装，提供线程安全的数据访问：

#### 1. **JY901S模块** (`main/jy901s.c/h`)

**模块化特点**：
```c
// JY901S传感器数据结构
typedef struct {
    float roll;             // 滚转角X (°)
    float pitch;            // 俯仰角Y (°) 
    float yaw;              // 偏航角Z (°)
    uint16_t version;       // 版本号
    bool is_valid;          // 数据有效性
    uint32_t packet_count;  // 包计数器
    uint32_t timestamp;     // 时间戳
} jy901s_data_t;

// 线程安全的数据访问接口
bool jy901s_get_data(jy901s_handle_t* handle, jy901s_data_t* data);
float jy901s_get_roll(jy901s_handle_t* handle);   // 直接获取ROLL角
float jy901s_get_pitch(jy901s_handle_t* handle);
float jy901s_get_yaw(jy901s_handle_t* handle);
```

#### 2. **数据解析任务** (`jy901s_parse_task`)
位置：`main/jy901s.c`

**核心处理循环**：
```c
// JY901S数据包格式：
// 包头: 0x55 0x53
// 数据长度: 11字节
// 数据内容: ROLL(2字节) + PITCH(2字节) + YAW(2字节) + VERSION(2字节) + 校验和(1字节)
void jy901s_parse_task(void* pvParameters) {
    uint8_t packet[JY901S_PACKET_LENGTH];
    uint8_t buffer_index = 0;
    uint8_t temp_byte;
    
    while (1) {
        // 读取UART数据
        int len = uart_read_bytes(handle->uart_port, &temp_byte, 1, 10 / portTICK_PERIOD_MS);
        if (len > 0) {
            handle->total_bytes_received++;
            
            // 状态机解析数据包
            if (buffer_index == 0 && temp_byte == JY901S_PACKET_HEADER1) {
                packet[buffer_index++] = temp_byte;
            } else if (buffer_index == 1 && temp_byte == JY901S_PACKET_HEADER2) {
                packet[buffer_index++] = temp_byte;
            } else if (buffer_index >= 2 && buffer_index < JY901S_PACKET_LENGTH) {
                packet[buffer_index++] = temp_byte;
                
                // 完整数据包接收完成
                if (buffer_index == JY901S_PACKET_LENGTH) {
                    jy901s_data_t temp_data = {0};
                    if (parse_jy901s_packet(packet, &temp_data)) {
                        // 线程安全更新共享数据
                        if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                            handle->sensor_data = temp_data;
                            handle->sensor_data.packet_count++;
                            xSemaphoreGive(handle->data_mutex);
                        }
                    }
                    buffer_index = 0;
                }
            } else {
                buffer_index = 0;  // 重置状态机
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
```

#### 3. **平衡控制任务中的ROLL角读取**
位置：`main/main.cpp`
```c
// 🔥 核心：直接读取ROLL角度（平衡车的左右倾倒控制轴）
float current_roll = jy901s_get_roll(g_jy901s_handle);  // 线程安全获取
float roll_error = current_roll - TARGET_ROLL_ANGLE;    // 计算平衡误差
```

#### 4. **实时显示界面**
位置：`main/main.cpp`

**突出ROLL角显示**：
```c
static void realtime_display_task(void *pvParameters) {
    const TickType_t refresh_rate = pdMS_TO_TICKS(500); // 2Hz刷新频率
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        vTaskDelayUntil(&last_wake_time, refresh_rate);
        
        // 获取JY901S数据
        jy901s_data_t sensor_data;
        bool data_valid = jy901s_get_data(g_jy901s_handle, &sensor_data);
        
        printf("\\033[2J\\033[H"); // 清屏
        printf("=== ESP32 JY901S 平衡控制系统 ===\\n");
        
        if (data_valid) {
            // 显示姿态数据，重点关注Roll角
            printf("**ROLL: %6.2f°**  Pitch: %6.2f°  Yaw: %6.2f°\\n",
                   sensor_data.roll, sensor_data.pitch, sensor_data.yaw);
            printf("平衡控制: ROLL=%.2f° (目标%.1f°±%.1f°)\\n", 
                   sensor_data.roll, TARGET_ROLL_ANGLE, ROLL_TOLERANCE);
        }
        
        fflush(stdout);
    }
}
```

### **优化特点**

1. **🏗️ 模块化架构**：传感器功能独立封装，便于维护和扩展
2. **🔒 线程安全**：使用互斥锁保护共享数据，避免数据竞争
3. **📐 正确控制轴**：使用ROLL角进行平衡控制，检测左右倾倒
4. **⚡ 高响应速度**：快速数据处理，1ms解析周期
5. **📊 完整数据**：支持三轴姿态角度，可扩展更复杂算法
6. **🛡️ 数据验证**：校验和检查，确保数据完整性

### **数据流程图**
```
JY901S传感器 → UART接收 → 解析任务 → 互斥锁保护 → 平衡控制 → 实时显示
   高频输出     快速接收   11字节包    线程安全      ROLL角     2Hz刷新
    ↓           ↓          ↓           ↓            ↓           ↓
   发送       状态机     0x55 0x53    共享数据      误差计算    可视化界面
```

## 硬件配置

### 引脚连接
```c
// JY901S传感器连接 (UART1)
#define JY901S_UART_RXD     GPIO_NUM_11  // ESP32 ← JY901S TX
#define JY901S_UART_TXD     GPIO_NUM_10  // ESP32 → JY901S RX

// 电机控制 (UART2)  
#define MOTOR_UART_TXD      GPIO_NUM_13  // ESP32 → 电机控制器
#define MOTOR_UART_RXD      GPIO_NUM_12  // ESP32 ← 电机控制器
```

### 通信参数
- **波特率**: 115200 bps
- **数据位**: 8位
- **停止位**: 1位
- **校验位**: 无
- **数据包格式**: 11字节，包含三轴角度数据
- **包头**: 0x55 0x53

## 软件架构

### 控制参数
```c
const float TARGET_ROLL_ANGLE = 0.0f;    // 目标roll角度（平衡位置）
const float ROLL_TOLERANCE = 3.0f;       // roll角度容差（±3度）
const float MOTOR_FIXED_SPEED = 15.0f;   // 电机固定转速
```

### 主要任务
1. **jy901s_parse_task**: JY901S数据解析任务（优先级4）
2. **balance_control_task**: 平衡控制主循环（优先级5）
3. **realtime_display_task**: 实时显示界面任务（优先级3，2Hz）
4. **uart2_monitor_task**: 电机数据监控任务（优先级3）

### 文件结构
```
main/
├── main.cpp          # 主程序和平衡控制逻辑
├── jy901s.c          # JY901S传感器模块实现
├── jy901s.h          # JY901S传感器模块头文件
├── motor_control.cpp # 电机控制模块实现
├── motor_control.h   # 电机控制模块头文件
├── old/              # 旧版本传感器代码（已废弃）
│   ├── gy25t.cpp     # 旧GY25T传感器代码
│   ├── gy25t.h       
│   ├── ky9250.c      # 旧KY9250传感器代码
│   └── ky9250.h      
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
- **核心原理**: 基于ROLL角进行平衡控制（左右倾倒检测）
- **平衡区间**: ROLL角在0°±3°范围内时，电机停止
- **失衡响应**: ROLL角超出容差范围时，延时后启动电机调节
- **控制方向**: 
  - ROLL > 0°（右倾）：电机调节（具体方向根据机械结构）
  - ROLL < 0°（左倾）：电机反向调节

### 智能重启机制
- **变化检测**: 监测ROLL角变化幅度
- **冷却保护**: 防止频繁操作的保护机制
- **高频恢复**: 重启后高频控制指令发送
- **错误清理**: 定期自动清理电机错误状态

### 安全机制
- 系统启动延时，确保传感器稳定
- 连续错误检测和重置机制
- 电机使能/失能确认机制
- 看门狗禁用，避免调试中断

## 调试和监控

### 实时状态显示
程序运行时会实时显示：
- **ROLL角度**（重点标记）
- Pitch和Yaw角度
- 传感器版本信息
- 传感器包计数
- 平衡控制状态
- UART接收统计

### 串口输出示例
```
=== ESP32 JY901S 平衡控制系统 ===
UART1: 接收1542字节
**ROLL: -1.23°**  Pitch: 0.45°  Yaw: 2.78°
版本: 0x1234 | 包#42
平衡控制: ROLL=-1.23° (目标0.0°±3.0°)
```

## 协议说明

### JY901S数据包格式
- **包头**: 0x55 0x53
- **数据长度**: 11字节
- **数据内容**: 
  - 字节2-3: ROLL角度数据 (int16_t)
  - 字节4-5: PITCH角度数据 (int16_t)
  - 字节6-7: YAW角度数据 (int16_t)
  - 字节8-9: 版本号 (uint16_t)
  - 字节10: 校验和
- **角度转换**: 角度 = 原始值 / 32768 * 180 (度)

## 注意事项

1. **启动顺序**: 确保在系统启动时保持设备静止
2. **控制轴选择**: 平衡车使用ROLL角检测左右倾倒
3. **电源要求**: 确保ESP32和传感器供电稳定
4. **串口冲突**: 避免多个程序同时占用串口
5. **角度范围**: ROLL角度范围为-180°到+180°
6. **实时性能**: 系统具备高实时响应能力

## 更新日志

### 当前版本
- **传感器模块**: 使用JY901S九轴传感器模块
- **控制方式**: 基于ROLL角的平衡控制
- **模块化设计**: 独立的传感器模块，线程安全接口
- **实时监控**: 2Hz刷新频率的状态显示界面

### 历史版本
- 项目曾使用GY25T和KY9250传感器（代码保存在old/目录）
- 经过多次优化改进，最终采用JY901S传感器方案

## 许可证

本项目采用开源许可证，详情请参考项目根目录的LICENSE文件。

---

🤖 Generated with [Claude Code](https://claude.ai/code)