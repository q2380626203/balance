# ESP32平衡车项目

基于ESP32和GY25T陀螺仪的智能平衡控制系统，实现基于YAW角度的精确姿态控制。

## 项目概述

本项目是一个ESP-IDF平衡车控制系统，主要功能：
- 通过GY25T陀螺仪获取实时姿态数据
- 基于YAW角度进行平衡控制
- 电机速度自动调节
- 实时状态监控和调试信息显示

## 🔥 **GY25T数据处理方法** 

### **当前实现架构（简化单任务模式）**

系统采用**单任务高效处理**架构，简化数据流并提升响应速度：

#### 1. **主处理任务** (`gy25t_main_task`)
位置：`main/gy25t.cpp:100-175`

**核心处理循环**：
```c
static void gy25t_main_task(void *pvParameters) {
    uint8_t buffer[64];          // 接收缓冲区
    uint8_t parse_buffer[128];   // 解析缓冲区  
    int parse_index = 0;
    
    while (1) {
        // 适当超时读取串口数据，200Hz对应5ms周期
        int length = uart_read_bytes(handle->config.uart_port, buffer, 
                                   sizeof(buffer), 10 / portTICK_PERIOD_MS);
        
        if (length > 0) {
            // 添加数据到解析缓冲区
            for (int i = 0; i < length && parse_index < sizeof(parse_buffer); i++) {
                parse_buffer[parse_index++] = buffer[i];
            }
            
            // 搜索并处理完整数据包
            bool processed_any = true;
            while (processed_any && parse_index >= GY25T_YAW_PACKET_SIZE) {
                processed_any = false;
                
                // 搜索帧头 0xA4 0x03 0x18 0x02
                for (int i = 0; i <= parse_index - GY25T_YAW_PACKET_SIZE; i++) {
                    if (parse_buffer[i] == 0xA4 && parse_buffer[i+1] == 0x03 && 
                        parse_buffer[i+2] == 0x18 && parse_buffer[i+3] == 0x02) {
                        
                        // 解析并更新全局yaw值
                        if (gy25t_parse_packet(handle, &parse_buffer[i])) {
                            // 移除已处理数据，继续处理剩余数据
                            int consumed = i + GY25T_YAW_PACKET_SIZE;
                            int remaining = parse_index - consumed;
                            if (remaining > 0) {
                                memmove(parse_buffer, &parse_buffer[consumed], remaining);
                                parse_index = remaining;
                            } else {
                                parse_index = 0;
                            }
                            processed_any = true;
                            break;
                        }
                    }
                }
            }
        }
        
        // 让出CPU时间给其他任务
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}
```

#### 2. **全局变量直接更新**
位置：`main/gy25t.cpp:175` & `main/gy25t.h:71`
```c
// 全局YAW角度变量 - 实时更新
volatile float g_last_yaw = 0.0f;

// 解析成功后直接更新全局变量
static bool gy25t_parse_packet(gy25t_handle_t* handle, const uint8_t* packet) {
    // ... 数据校验和解析 ...
    
    // 直接更新全局变量 
    g_last_yaw = calculated_yaw;
    return true;
}
```

#### 3. **控制任务中的实时读取**
位置：`main/main.cpp:141-142`
```c
// 🔥 核心：直接读取全局YAW变量
current_yaw = g_last_yaw;  // 零延迟获取最新角度
// YAW误差计算在控制逻辑中进行
```

#### 4. **实时显示界面**
位置：`main/main.cpp:44-83`

**全屏实时刷新界面**：
```c
// 10Hz刷新频率的实时显示
static void realtime_display_task(void *pvParameters) {
    const TickType_t refresh_rate = pdMS_TO_TICKS(100); // 10Hz刷新频率
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        vTaskDelayUntil(&last_wake_time, refresh_rate);
        
        float current_yaw = g_last_yaw;  // 实时读取
        
        // 清屏并显示固定界面
        printf("\033[2J\033[H"); // 清屏并移动光标到左上角
        printf("╔════════════════════════════════════════════════════════════╗\n");
        printf("║                ESP32 GY25T 平衡控制系统                   ║\n");
        printf("╠════════════════════════════════════════════════════════════╣\n");
        printf("║ 实时数据显示                                             ║\n");
        printf("║                                                          ║\n");
        printf("╚════════════════════════════════════════════════════════════╝\n");
        
        // 显示当前YAW数据
        printf("当前YAW角度: %.2f°\n", current_yaw);
        
        fflush(stdout);
    }
}
```

### **优化特点**

1. **🚀 零拷贝架构**：数据直接更新到全局变量，无队列开销
2. **⚡ 高响应速度**：单任务处理，减少上下文切换延迟
3. **🔄 连续处理**：while循环确保处理缓冲区中的所有数据包
4. **📊 实时监控**：显示界面实时显示数据处理频率
5. **🛡️ 缓冲区保护**：智能的溢出处理，避免数据丢失

### **数据流程图**
```
GY25T陀螺仪 → UART接收 → 单任务解析 → 全局变量更新 → 控制任务读取 → 实时显示
   200Hz       10ms      连续处理      直接写入        直接读取       10Hz刷新
    ↓           ↓           ↓             ↓              ↓             ↓
   发送       快速接收    帧头搜索      g_last_yaw    零延迟获取    可视化界面
```

## 硬件配置

### 引脚连接
```c
// 陀螺仪连接 (UART1)
#define GYRO_UART_TXD       GPIO_NUM_17  // ESP32 → GY25T RX
#define GYRO_UART_RXD       GPIO_NUM_16  // ESP32 ← GY25T TX

// 电机控制 (UART2)  
#define MOTOR_UART_TXD      GPIO_NUM_19  // ESP32 → 电机控制器
#define MOTOR_UART_RXD      GPIO_NUM_18  // ESP32 ← 电机控制器
```

### 通信参数
- **波特率**: 115200 bps
- **数据位**: 8位
- **停止位**: 1位
- **校验位**: 无
- **陀螺仪输出频率**: 200Hz（可配置）

## 软件架构

### 控制参数
```c
const float TARGET_YAW_ANGLE = 0.0f;      // 目标yaw角度
const float YAW_TOLERANCE = 0.5f;         // yaw角度容差（±0.5度）
const float MOTOR_FIXED_SPEED = 2.0f;     // 电机固定转速
```

### 主要任务
1. **gy25t_main_task**: GY25T数据高效解析任务（最高优先级10）
2. **balance_control_task**: 平衡控制主循环（优先级5）
3. **realtime_display_task**: 实时显示界面任务（优先级3）
4. **uart2_monitor_task**: 电机数据监控任务（优先级3）

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
idf.py set-target esp32

# 3. 编译项目
idf.py build

# 4. 烧录到设备
idf.py -p /dev/ttyUSB0 flash

# 5. 监控串口输出
idf.py -p /dev/ttyUSB0 monitor
```

## 控制逻辑

### 平衡算法
- 当YAW角度在目标值±0.5°范围内时，电机停止
- 当YAW角度超出容差范围时，延时500ms后启动电机调节
- 电机运行方向根据YAW误差符号决定

### 安全机制
- 系统启动5秒延时，确保传感器稳定
- 连续错误检测和重置机制
- 电机使能/失能三次确认发送

## 调试和监控

### 实时状态显示
程序运行时会实时显示：
- 当前YAW角度
- 目标YAW角度  
- YAW角度误差
- 电机运行状态
- 设定转速值
- 陀螺仪统计信息

### 串口输出示例
```
╔════════════════════════════════════════════════════════════╗
║                ESP32 GY25T 平衡控制系统                   ║
╠════════════════════════════════════════════════════════════╣
║ 实时数据显示                                             ║
║                                                          ║
╚════════════════════════════════════════════════════════════╝

[成功解析#0042] YAW角度更新: 1.20° -> 1.23° (变化0.03°) - 数据已处理
当前YAW角度: 1.23°
```

## 文件结构
```
平衡/
├── main/
│   ├── main.cpp          # 主程序入口和控制逻辑
│   ├── gy25t.cpp         # GY25T陀螺仪驱动实现
│   ├── gy25t.h           # GY25T陀螺仪驱动头文件
│   ├── motor_control.cpp # 电机控制模块实现
│   ├── motor_control.h   # 电机控制模块头文件
│   └── CMakeLists.txt    # 主程序构建配置
├── build/                # 编译输出目录
├── CMakeLists.txt        # 项目根配置
├── sdkconfig             # ESP-IDF配置
├── sdkconfig.ci          # CI配置文件
├── sdkconfig.old         # 配置备份文件
├── partitions.csv        # 分区表配置
├── pytest_hello_world.py # 测试脚本
├── gy25t协议.md          # GY25T通信协议说明
└── README.md            # 项目说明文档
```

## 协议说明

详细的GY25T陀螺仪通信协议请参考 [`gy25t协议.md`](gy25t协议.md)

## 注意事项

1. **启动顺序**: 确保在系统启动的5秒内保持设备静止
2. **电源要求**: 确保ESP32和陀螺仪供电稳定
3. **串口冲突**: 避免多个程序同时占用串口
4. **角度范围**: YAW角度范围为-180°到+180°
5. **实时性能**: 控制循环运行在200Hz，确保实时响应

## 许可证

本项目采用开源许可证，详情请参考项目根目录的LICENSE文件。