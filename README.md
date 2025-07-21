# ESP32平衡车项目

基于ESP32和GY25T陀螺仪的智能平衡控制系统，实现基于YAW角度的精确姿态控制。

## 项目概述

本项目是一个ESP-IDF平衡车控制系统，主要功能：
- 通过GY25T陀螺仪获取实时姿态数据
- 基于YAW角度进行平衡控制
- 电机速度自动调节
- 实时状态监控和调试信息显示

## 🔥 **实时数据获取方法** 

### **核心实现原理**

系统采用**多任务事件驱动**架构获取实时数据：

#### 1. **数据采集任务** (`gy25t_event_task`)
位置：`main/gy25t.cpp:121-141`
```c
// 200Hz高频率采集GY25T陀螺仪数据
while (1) {
    int len = uart_read_bytes(handle->config.uart_port, dtmp, PARSER_BUFFER_SIZE, pdMS_TO_TICKS(10));
    if (len > 0) {
        gy25t_parse_and_send(handle, dtmp, len);  // 实时解析并发送到队列
    }
}
```

#### 2. **实时队列机制** (`data_queue`)
位置：`main/gy25t.h:64`
```c
QueueHandle_t data_queue;  // 用于传递已解析YAW角数据的队列
```

**滑动窗口队列管理**（`gy25t.cpp:174-179`）:
```c
// 队列满时自动丢弃旧数据，保证最新数据优先级
if (xQueueSend(handle->data_queue, &filtered_yaw, 0) != pdTRUE) {
    float dummy;
    xQueueReceive(handle->data_queue, &dummy, 0);  // 移除最旧数据
    xQueueSend(handle->data_queue, &filtered_yaw, 0);  // 发送最新数据
}
```

#### 3. **实时控制循环** (`balance_control_task`)
位置：`main/main.cpp:87-192`

**关键实时数据获取代码**（`main.cpp:111-117`）：
```c
// 🔥 核心：获取最新YAW角数据的实时方法
float new_yaw;
// 循环读取队列直到为空，确保使用最新数据
while (xQueueReceive(g_gyro_handle->data_queue, &new_yaw, 0) == pdTRUE) {
    current_yaw = new_yaw;  // 更新为最新的YAW角度
}
// 如果队列为空，current_yaw保持上一次的有效值
```

**实时显示**（`main.cpp:177-189`）：
```c
printf("YAW角    : %8.2f °\n", current_yaw);      // 🔥 实时YAW角度
printf("YAW误差  : %8.2f °\n", yaw_error);       // 🔥 实时控制误差
printf("电机状态 : %-4s\n", motor_should_run ? "运行" : "停止");  // 🔥 实时电机状态
```

### **数据流程图**
```
GY25T陀螺仪 → UART1接收 → 数据解析任务 → 滤波处理 → 队列缓存 → 控制任务 → 实时显示
   200Hz         10ms         实时解析      低通滤波     50深度     200Hz      1Hz更新
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
1. **uart1_monitor_task**: 陀螺仪数据监控
2. **uart2_monitor_task**: 电机数据监控  
3. **balance_control_task**: 平衡控制主循环
4. **gy25t_event_task**: GY25T数据解析任务

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
--- 实时状态 ---
YAW角    :     1.23 °
目标YAW  :     0.00 °  
YAW误差  :     1.23 °
电机状态 : 运行
设定速度 :      2.0
--- 诊断信息 ---
陀螺仪无效包: 5
------------------
```

## 文件结构
```
平衡/
├── main/
│   ├── main.cpp          # 主程序入口和控制逻辑
│   ├── gy25t.cpp/.h      # GY25T陀螺仪驱动
│   ├── motor_control.cpp/.h  # 电机控制模块
│   └── CMakeLists.txt    # 主程序构建配置
├── CMakeLists.txt        # 项目根配置
├── sdkconfig            # ESP-IDF配置
├── partitions.csv       # 分区表配置
├── gy25t协议.md         # GY25T通信协议说明
└── README.md           # 项目说明文档
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