# ESP32 + STM32 智能安全帽系统

基于小智ESP32 AI语音助手和STM32多传感器的智能安全帽项目，通过MCP协议实现语音控制的多功能安全监测系统。

## 项目简介

本项目结合了ESP32的AI语音交互能力和STM32的多传感器采集能力，打造了一款可以通过语音命令控制的智能安全帽。用户可以通过自然语言与AI对话，实时查询和控制安全帽上的各种传感器。

### 系统架构

```
┌──────────────────────────────────────┐
│      ESP32 小智AI语音助手             │
│  • 离线语音唤醒 (ESP-SR)              │
│  • 流式 ASR + LLM + TTS               │
│  • MCP协议设备控制                    │
│  • WiFi/4G云端连接                    │
│  • OLED/LCD显示                       │
└──────────────┬───────────────────────┘
               │ UART通信
               │ 115200bps
               │ GPIO17(TX) / GPIO18(RX)
               ↓
┌──────────────────────────────────────┐
│    STM32F103 多传感器控制板           │
│  • MAX30102 心率血氧传感器            │
│  • MPU6050 六轴姿态传感器             │
│  • MQ2 可燃气体/烟雾传感器            │
│  • DHT11 温湿度传感器                 │
│  • GPS 定位模块                       │
│  • OLED 0.96寸显示屏                  │
└──────────────────────────────────────┘
```

## 主要功能

### STM32传感器端
- ✅ **心率血氧监测** - MAX30102传感器实时检测心率和血氧饱和度
- ✅ **姿态检测** - MPU6050 DMP运动检测，支持跌倒和碰撞识别
- ✅ **环境监测** - 温湿度实时采集
- ✅ **危险预警** - 烟雾/可燃气体浓度检测
- ✅ **GPS定位** - 实时位置追踪
- ✅ **事件报警** - 自动检测跌倒、碰撞、烟雾事件并上报

### ESP32 AI控制端
- ✅ **语音交互** - 通过小智AI进行自然语言对话
- ✅ **MCP协议控制** - 所有传感器通过MCP工具暴露给大模型
- ✅ **智能告警** - 接收STM32事件并语音播报
- ✅ **数据可视化** - 在显示屏上展示传感器数据
- ✅ **云端连接** - 支持接入xiaozhi.me或自建服务器

## 通信协议

STM32与ESP32之间通过UART串口通信，采用简单的JSON格式协议：

### 命令格式（ESP32 → STM32）
```
{CMD:HR_START}       - 启动心率检测
{CMD:HR_STOP}        - 停止心率检测
{CMD:TEMP_START}     - 启动温度检测
{CMD:MPU_START}      - 启动姿态检测
{CMD:GPS_START}      - 启动GPS定位
{CMD:MQ2_START}      - 启动烟雾检测
```

### 数据格式（STM32 → ESP32）
```
{DATA:HR:75,SPO2:98}           - 心率75bpm，血氧98%
{DATA:TEMP:25,35}              - 温度25°C，湿度35%
{DATA:MPU:10.5,-5.2,180.0}     - 俯仰角/横滚角/航向角
{DATA:GPS:39.908722,116.397496,8} - 纬度/经度/卫星数
{DATA:MQ2:150.5,2}             - 烟雾PPM值/报警级别
```

### 事件格式（STM32 → ESP32）
```
{EVENT:FALL}         - 检测到跌倒
{EVENT:COLLISION}    - 检测到碰撞
{EVENT:SMOKE}        - 检测到烟雾超标
```

### 状态响应
```
{STATUS:OK}          - 命令执行成功
{STATUS:ERROR}       - 命令执行失败
```

## MCP工具接口

ESP32通过MCP协议将所有STM32传感器控制接口暴露给大模型，用户可以通过语音命令控制：

### 心率血氧
- `self.stm32.heart_rate.start` - 启动心率血氧监测
- `self.stm32.heart_rate.stop` - 停止监测
- `self.stm32.heart_rate.get_state` - 获取当前心率和血氧值

### 温湿度
- `self.stm32.temperature.start` - 启动温湿度监测
- `self.stm32.temperature.stop` - 停止监测
- `self.stm32.temperature.get_state` - 获取当前温湿度值

### 姿态检测
- `self.stm32.mpu6050.start` - 启动姿态监测
- `self.stm32.mpu6050.stop` - 停止监测
- `self.stm32.mpu6050.get_state` - 获取俯仰角/横滚角/航向角

### 烟雾检测
- `self.stm32.mq2.start` - 启动烟雾检测
- `self.stm32.mq2.stop` - 停止检测
- `self.stm32.mq2.get_state` - 获取烟雾浓度和报警级别

### GPS定位
- `self.stm32.gps.start` - 启动GPS定位
- `self.stm32.gps.stop` - 停止定位
- `self.stm32.gps.get_state` - 获取经纬度和卫星数

## 使用示例

通过语音与AI对话：

```
用户: "小智小智，帮我检测一下心率"
小智: "好的，正在启动心率检测..."
     [调用 self.stm32.heart_rate.start]
     "您的心率是75次每分钟，血氧饱和度98%"

用户: "检测一下环境温度"
小智: "当前环境温度25度，湿度35%"

用户: "我的位置在哪里"
小智: "正在获取GPS定位..."
     "您当前位于北纬39.908722，东经116.397496"
```

## 硬件清单

### ESP32部分
- ESP32开发板（支持小智AI的任意硬件）
  - 立创实战派 ESP32-S3
  - 乐鑫 ESP32-S3-BOX3
  - M5Stack CoreS3
  - 或其他支持的70+款开源硬件
- 麦克风、扬声器（语音交互）
- OLED/LCD显示屏（可选）

### STM32部分
- STM32F103C8T6开发板
- MAX30102 心率血氧传感器
- MPU6050 六轴陀螺仪
- MQ2 烟雾传感器
- DHT11 温湿度传感器
- GPS定位模块
- OLED 0.96寸显示屏
- 面包板和杜邦线

### 连接方式
```
ESP32 GPIO17 (TX)  →  STM32 PC11 (USART3_RX)
ESP32 GPIO18 (RX)  ←  STM32 PC10 (USART3_TX)
GND                ←→ GND
```

## 项目结构

```
.
├── STM32_Class_Project/          # STM32传感器控制端
│   ├── APP/                      # 应用层代码
│   │   ├── esp32_comm.c/h        # ESP32通信模块
│   │   ├── max30102_app.c/h      # 心率血氧
│   │   ├── mpu_app.c/h           # 姿态检测
│   │   ├── mq2_app.c/h           # 烟雾检测
│   │   ├── dht11_app.c/h         # 温湿度
│   │   ├── gps_app.c/h           # GPS定位
│   │   ├── oled.c/h              # OLED显示
│   │   └── scheduler.c/h         # 任务调度器
│   ├── Core/                     # STM32 HAL库核心
│   ├── Drivers/                  # STM32 HAL驱动
│   └── CMakeLists.txt            # CMake构建文件
│
└── xiaozhi-esp32-clean/          # ESP32 AI语音控制端
    ├── main/
    │   ├── application.cc        # 主应用逻辑
    │   ├── mcp_server.cc/h       # MCP协议服务器
    │   ├── boards/common/
    │   │   └── stm32_controller.h # STM32控制器（MCP工具定义）
    │   ├── audio/                # 音频处理
    │   ├── display/              # 显示驱动
    │   └── protocols/            # 通信协议
    ├── managed_components/       # ESP32依赖组件
    └── CMakeLists.txt
```

## 编译与烧录

### STM32端

**环境要求**
- CMake 3.22+
- ARM GCC工具链
- CLion / VS Code + CMake插件

**编译步骤**
```bash
cd STM32_Class_Project/STM32_Class_Project
mkdir build && cd build
cmake ..
make
```

**烧录**
使用STM32 CubeProgrammer或OpenOCD烧录生成的.bin文件

### ESP32端

**环境要求**
- ESP-IDF 5.4+
- VS Code + ESP-IDF插件（推荐）

**编译步骤**
```bash
cd xiaozhi-esp32-clean
idf.py build
idf.py flash monitor
```

**配置**
首次使用需要配置WiFi和服务器信息，可通过Web配置页面或串口配置。

## 快速开始

### 1. 烧录固件
- STM32: 烧录传感器控制程序
- ESP32: 烧录小智AI固件

### 2. 硬件连接
按照上述接线图连接ESP32和STM32

### 3. 配置ESP32
- 连接到ESP32的WiFi热点
- 访问配置页面设置网络
- 选择大模型服务（官方/自建）

### 4. 开始使用
- 语音唤醒："小智小智"
- 语音命令："帮我检测一下心率"
- 查看显示屏上的传感器数据

## 开发文档

详细的开发文档请参考：
- [STM32传感器开发说明](STM32_Class_Project/README.md)
- [ESP32小智开发指南](xiaozhi-esp32-clean/README.md)
- [通信协议详解](docs/protocol.md)

## 应用场景

- 🏗️ **建筑工地** - 实时监测工人健康状态和安全
- 🏭 **工厂车间** - 检测有害气体，预防工伤事故
- 🚧 **矿井作业** - GPS定位+跌倒检测，保障井下安全
- 🏥 **医疗看护** - 老年人跌倒检测和健康监护
- 🔥 **消防救援** - 高温高危环境下的生命体征监测

## 技术特点

1. **模块化设计** - STM32和ESP32分工明确，易于扩展
2. **AI语音控制** - 通过自然语言操作，无需手动按键
3. **MCP协议** - 所有传感器统一接口，大模型直接调用
4. **事件驱动** - 自动检测危险并主动告警
5. **低功耗** - 任务调度器优化，传感器按需启动

## 许可证

本项目基于以下开源项目：
- [xiaozhi-esp32](https://github.com/78/xiaozhi-esp32) - MIT License
- STM32 HAL库 - ST License

本项目代码以 MIT License 发布，可自由用于个人或商业用途。

## 致谢

感谢虾哥（78）开源的小智ESP32项目，为本项目提供了强大的AI语音交互能力。

## 联系方式

如有问题或建议，欢迎提Issue或加入讨论群。

---

**⚠️ 注意事项**
- 本项目为教育学习用途，实际工业应用需要进一步测试和认证
- 传感器数据仅供参考，不能替代专业医疗设备
- 使用时请注意用电安全和传感器使用规范
