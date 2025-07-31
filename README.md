# ESP32平衡车双协议通信系统

基于ESP32S3的模块化平衡车控制系统，集成BLE IMU传感器、WiFi AP热点和Web配置界面，实现本地控制与远程监控的双重功能。

## 项目概述

这是一个完全模块化的ESP-IDF平衡车控制系统，主要特性：

### 🔄 双协议通信架构
- **BLE协议**: 连接IMU传感器获取实时姿态数据  
- **WiFi协议**: 提供AP热点和Web配置界面

### ⚖️ 智能平衡控制
- 基于PITCH角度的精确平衡控制算法
- 自适应电机速度调节系统
- 实时故障检测和自动恢复机制

### 🌐 远程监控管理
- WiFi AP热点：`ESP32_Balance_Config` (密码: `balance123`)
- Web管理界面：`http://192.168.4.1`
- 实时数据可视化和参数调节

### 🧩 模块化系统架构
- 共享数据管理器（线程安全）
- 独立的功能模块（BLE、WiFi、Web、控制器）
- 统一的状态管理和错误处理

## 🏗️ 系统架构设计

### 核心模块架构图
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   BLE IMU模块   │    │   WiFi AP模块   │    │  Web服务器模块  │
│                 │    │                 │    │                 │
│ • 姿态数据采集  │    │ • 热点管理      │    │ • Web界面       │
│ • 自动重连      │    │ • 客户端管理    │    │ • REST API      │
│ • 线程安全      │    │ • 网络配置      │    │ • 实时数据      │
└─────────┬───────┘    └─────────┬───────┘    └─────────┬───────┘
          │                      │                      │
          └──────────────────────┼──────────────────────┘
                                 │
                    ┌─────────────▼─────────────┐
                    │     共享数据管理器        │
                    │                           │
                    │ • 全局状态管理            │
                    │ • 配置参数存储            │
                    │ • 线程安全访问            │
                    │ • 模块间通信             │
                    └─────────────┬─────────────┘
                                 │
                    ┌─────────────▼─────────────┐
                    │    平衡控制器模块         │
                    │                           │
                    │ • PITCH角度控制           │
                    │ • 电机速度调节            │
                    │ • 自动故障恢复            │
                    │ • 实时控制算法            │
                    └───────────────────────────┘
```

## 📡 BLE IMU数据处理

### BLE IMU模块 (`main/ble_imu.c/h`)
```c
// 完整IMU数据结构
typedef struct {
    float roll, pitch, yaw;                    // 欧拉角 (°)
    float acc_x, acc_y, acc_z;                // 加速度 (g)
    float gyro_x, gyro_y, gyro_z;             // 角速度 (°/s) 
    float mag_x, mag_y, mag_z;                // 磁场强度
    float q0, q1, q2, q3;                     // 四元数
    bool is_valid;                            // 数据有效性
    uint32_t packet_count, timestamp;         // 统计信息
} ble_imu_data_t;

// 线程安全数据访问
float ble_imu_get_pitch(ble_imu_handle_t* handle);  // 平衡控制关键轴
```

### 关键特性
- **目标设备**: MAC地址 `e8:cb:ed:5a:52:8e`
- **自动管理**: 扫描、连接、重连全自动
- **数据完整**: 9轴传感器 + 磁场 + 四元数
- **线程安全**: 互斥锁保护，支持多任务访问

### WiFi AP模块 (`main/wifi_ap.c/h`)
- **热点名称**: `ESP32_Balance_Config`
- **连接密码**: `balance123` 
- **IP地址**: `192.168.4.1`
- **最大连接**: 4个客户端
- **自动管理**: 客户端连接/断开事件处理

### Web服务器模块 (`main/web_server.c/h`)
- **服务端口**: HTTP 80
- **访问地址**: `http://192.168.4.1`
- **功能特性**:
  - 实时系统状态显示
  - 平衡参数在线调节
  - IMU数据可视化图表
  - 系统配置管理界面

## 🔧 硬件配置

### 通信接口
```c
// BLE IMU传感器
#define TARGET_DEVICE_MAC  "e8:cb:ed:5a:52:8e"  // 目标IMU设备

// 电机控制 (UART2)  
#define MOTOR_UART_TXD      GPIO_NUM_13         // ESP32 → 电机控制器
#define MOTOR_UART_RXD      GPIO_NUM_12         // ESP32 ← 电机控制器
#define MOTOR_UART_BAUD     115200              // 通信波特率
```

### 系统资源配置
- **Flash容量**: 4MB (支持大型Web界面和OTA升级)
- **分区表**: 自定义分区 (app: 3600KB, spiffs: 352KB)  
- **FreeRTOS频率**: 1000Hz (高精度控制)
- **双协议支持**: BLE + WiFi 并发运行

## ⚙️ 系统配置与控制

### 共享数据管理器 (`main/shared_data.c/h`)
```c
// 平衡控制配置参数
typedef struct {
    float target_pitch_angle;    // 目标pitch角度 (默认: 0.0°)
    float pitch_tolerance;       // pitch角度容差 (默认: ±10.0°)
    float motor_fixed_speed;     // 电机固定转速 (默认: 30.0)
    float enable_delay_ms;       // 启动延时 (毫秒)
    bool  auto_restart_enabled;  // 自动重启使能
    float restart_threshold;     // 重启阈值
} balance_config_t;

// 系统状态监控
typedef struct {
    bool ble_connected;          // BLE连接状态
    bool motor_enabled;          // 电机使能状态  
    float pitch_error;           // 当前pitch误差
    bool in_tolerance;           // 是否在平衡区间
    uint32_t uptime_ms;          // 系统运行时间
    uint32_t control_loop_count; // 控制循环计数
} system_status_t;
```

### 模块化任务架构
1. **平衡控制器任务**: 独立的PITCH角度控制循环 (`balance_controller.c`)
2. **BLE连接管理任务**: 自动扫描、连接、数据接收 (`ble_imu.c`)
3. **WiFi AP服务任务**: 热点管理和客户端处理 (`wifi_ap.c`)
4. **Web服务器任务**: HTTP请求处理和API响应 (`web_server.c`)

### 项目文件结构
```
main/
├── main.cpp              # 系统初始化和模块协调
├── shared_data.c/h       # 共享数据管理器（线程安全）
├── balance_controller.c/h # 平衡控制器模块
├── ble_imu.c/h          # BLE IMU传感器模块
├── motor_control.cpp/h   # 电机控制驱动
├── wifi_ap.c/h          # WiFi AP热点模块
├── web_server.c/h       # Web服务器模块
├── CMakeLists.txt       # ESP-IDF构建配置
├── partitions.csv       # 自定义分区表
└── old/                 # 历史版本传感器代码
    ├── jy901s.c/h       # JY901S UART传感器
    ├── gy25t.cpp/h      # GY25T传感器
    └── ky9250.c/h       # KY9250传感器

配置文件：
├── sdkconfig            # ESP-IDF项目配置
├── partitions.csv       # Flash分区表
└── plan.md             # 开发计划文档
```

## 🚀 编译部署

### 开发环境要求
- **ESP-IDF**: v4.4+ (推荐v5.0+)
- **工具链**: CMake 3.16+, Python 3.8+
- **硬件平台**: ESP32S3 (支持BLE+WiFi双协议)
- **Flash需求**: 4MB Flash (支持Web界面和OTA)

### 快速开始
```bash
# 1. 设置ESP-IDF环境
. $HOME/esp/esp-idf/export.sh

# 2. 配置目标芯片
idf.py set-target esp32s3

# 3. 编译完整项目 
idf.py build

# 4. 烧录固件到设备
idf.py -p /dev/ttyUSB0 flash

# 5. 监控串口日志
idf.py -p /dev/ttyUSB0 monitor
```

### 部署后访问
1. **串口监控**: 查看详细系统日志和调试信息
2. **连接WiFi**: 手机/电脑连接 `ESP32_Balance_Config` (密码: `balance123`)
3. **Web界面**: 浏览器访问 `http://192.168.4.1`

## 🎯 平衡控制算法

### 核心控制逻辑
```c
// 基于PITCH角的平衡控制算法
float current_pitch = shared_data_get_pitch(shared_data);
float pitch_error = current_pitch - config.target_pitch_angle;

if (abs(pitch_error) > config.pitch_tolerance) {
    // 超出平衡区间：启动电机调节
    float motor_speed = (pitch_error > 0) ? config.motor_fixed_speed : -config.motor_fixed_speed;
    motor_control_set_velocity(motor_controller, motor_speed);
} else {
    // 在平衡区间内：停止电机
    motor_control_set_velocity(motor_controller, 0.0f);
}
```

### 智能特性
- **自适应启动**: 延时启动机制，避免初始化冲击
- **故障恢复**: 自动检测并恢复电机错误状态
- **平滑控制**: 渐进式速度调节，避免突变
- **安全保护**: 多重安全检查，防止失控

## 📊 实时监控

### 双路监控显示
1. **串口终端**: 详细的系统状态和调试信息
2. **Web界面**: 直观的图形化监控面板

### 监控数据内容
- **姿态信息**: Roll/Pitch/Yaw角度实时显示
- **传感器数据**: 9轴IMU完整数据
- **控制状态**: 电机状态、平衡误差、控制输出
- **通信状态**: BLE连接、WiFi客户端、数据统计
- **系统信息**: CPU使用率、内存状态、运行时间

### Web界面功能
- 📈 **实时图表**: 姿态角度变化曲线
- ⚙️ **参数调节**: 在线修改平衡控制参数
- 📋 **状态面板**: 系统各模块运行状态
- 🔧 **配置管理**: 保存/恢复系统配置

## 📡 通信协议

### BLE IMU通信
- **协议**: BLE GATT通信协议
- **目标设备**: MAC地址 `e8:cb:ed:5a:52:8e`
- **数据频率**: 高频实时传输 (100Hz+)
- **数据内容**: 完整9轴IMU + 四元数 + 磁场数据

### WiFi Web通信
- **协议**: HTTP REST API
- **数据格式**: JSON格式数据交换
- **实时更新**: WebSocket长连接支持(可选)
- **API端点**: 
  - `GET /api/status` - 获取系统状态
  - `POST /api/config` - 更新配置参数
  - `GET /api/data` - 获取实时传感器数据

## ⚠️ 部署注意事项

### 🔧 硬件准备
1. **ESP32S3芯片**: 确认支持BLE+WiFi双协议并发
2. **Flash容量**: 必须使用4MB Flash，2MB无法支持Web界面
3. **电源稳定**: 双协议运行功耗较高，确保供电充足
4. **IMU设备**: 确保BLE IMU设备 (`e8:cb:ed:5a:52:8e`) 处于可连接状态

### 📡 通信环境
1. **BLE距离**: 保持IMU设备在3-10米有效通信距离内
2. **WiFi频段**: 使用2.4GHz频段，避免与其他设备冲突
3. **双协议干扰**: BLE和WiFi可能存在频段干扰，监控连接稳定性

### 🖥️ 系统资源
1. **内存使用**: 双协议+Web服务器消耗较多RAM，监控堆栈使用
2. **CPU负载**: 多任务并发运行，确保控制循环不被阻塞
3. **看门狗**: 系统已禁用看门狗，生产环境需重新评估

### 🔒 安全考虑
1. **WiFi密码**: 默认密码 `balance123` 较弱，生产环境应修改
2. **Web访问**: 无身份验证，仅适用于受控环境
3. **API安全**: REST API未加密，避免在公网环境使用

## 🔄 版本演进历史

### 🚀 当前版本 (v3.0 - 双协议通信系统)
- **架构重构**: 完全模块化设计，采用共享数据管理器
- **双协议支持**: BLE IMU数据采集 + WiFi Web配置界面
- **Web管理**: 实时监控界面和参数在线调节功能
- **资源升级**: 4MB Flash + 自定义分区表 + 高频FreeRTOS
- **平台优化**: 专为ESP32S3双协议并发优化

### 📝 历史版本对比
| 版本 | 传感器 | 通信方式 | 控制轴 | 主要特性 |
|------|-------|----------|--------|----------|
| **v3.0** | BLE IMU | BLE+WiFi双协议 | PITCH | 模块化架构+Web界面 |
| v2.0 | BLE IMU | BLE单协议 | PITCH | 无线连接+线程安全 |
| v1.x | JY901S | UART有线 | ROLL | 基础平衡控制 |
| v0.x | GY25T/KY9250 | UART有线 | ROLL | 早期原型 |

### 🗂️ 历史代码保留
- **old/目录**: 保存v1.x和v0.x版本的传感器驱动代码
- **向下兼容**: 可快速回退到UART传感器方案
- **学习参考**: 展示系统架构演进过程

## 📋 开发路线图

### 🎯 下一版本计划 (v3.1)
- **OTA升级**: 基于Web界面的固件在线升级
- **数据存储**: 历史数据记录和分析功能
- **高级控制**: PID控制算法和参数自整定
- **多设备**: 支持多个BLE IMU设备并发

### 🔮 未来扩展方向
- **云端连接**: 数据上传到云平台进行分析
- **机器学习**: 基于历史数据的智能平衡算法
- **移动APP**: 专用手机应用程序
- **传感器融合**: 整合更多传感器类型

## 📄 许可证声明

本项目基于开源许可证发布，允许学习、修改和分发。详细条款请参考项目根目录的LICENSE文件。

### 🙏 致谢
感谢ESP-IDF开发团队提供的优秀框架，以及开源社区的贡献和支持。

---

**🤖 本文档由 Claude Code 协助生成和维护**  
**📅 最后更新**: $(date)