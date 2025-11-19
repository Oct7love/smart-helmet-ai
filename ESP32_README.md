# ESP32 小智AI语音控制端

基于小智ESP32开源项目的AI语音助手，通过MCP协议控制STM32传感器。

## 项目简介

本项目是[xiaozhi-esp32](https://github.com/78/xiaozhi-esp32)的应用实例，通过添加STM32控制器模块，实现了对智能安全帽所有传感器的语音控制。

## 核心功能

- ✅ **离线语音唤醒** - 基于ESP-SR引擎
- ✅ **流式对话** - ASR + LLM + TTS
- ✅ **MCP协议** - 设备端工具调用
- ✅ **UART通信** - 与STM32实时数据交互
- ✅ **事件响应** - 自动播报跌倒/碰撞/烟雾警告

## STM32控制器集成

### 文件位置
```
main/boards/common/stm32_controller.h
```

### MCP工具定义

控制器自动注册以下MCP工具到大模型：

#### 心率血氧
```cpp
self.stm32.heart_rate.start      // 启动监测
self.stm32.heart_rate.stop       // 停止监测
self.stm32.heart_rate.get_state  // 获取数据
// 返回: {"active": true, "heart_rate": 75, "spo2": 98}
```

#### 温湿度
```cpp
self.stm32.temperature.start
self.stm32.temperature.stop
self.stm32.temperature.get_state
// 返回: {"active": true, "temperature": 25.0, "humidity": 35}
```

#### 姿态检测
```cpp
self.stm32.mpu6050.start
self.stm32.mpu6050.stop
self.stm32.mpu6050.get_state
// 返回: {"active": true, "pitch": 10.5, "roll": -5.2, "yaw": 180.0}
```

#### 烟雾检测
```cpp
self.stm32.mq2.start
self.stm32.mq2.stop
self.stm32.mq2.get_state
// 返回: {"active": true, "ppm": 150.5, "alarm_level": 2}
```

#### GPS定位
```cpp
self.stm32.gps.start
self.stm32.gps.stop
self.stm32.gps.get_state
// 返回: {"active": true, "latitude": 39.908722, "longitude": 116.397496, "satellites": 8}
```

## UART通信配置

```cpp
#define STM32_UART_NUM UART_NUM_1
#define STM32_TXD_PIN  GPIO_NUM_17
#define STM32_RXD_PIN  GPIO_NUM_18
#define STM32_UART_BAUD_RATE 115200
```

### 接线方式
```
ESP32 GPIO17 (TX) → STM32 PC11 (USART3_RX)
ESP32 GPIO18 (RX) ← STM32 PC10 (USART3_TX)
GND              ↔  GND
```

## 事件处理

STM32检测到异常事件时会主动上报，ESP32自动响应：

### 跌倒事件
```cpp
收到: {EVENT:FALL}
响应: 显示警告 + 播放警报音
```

### 碰撞事件
```cpp
收到: {EVENT:COLLISION}
响应: 显示警告 + 播放警报音
```

### 烟雾超标
```cpp
收到: {EVENT:SMOKE}
响应: 显示警告 + 播放警报音
```

## 编译与烧录

### 环境搭建
```bash
# 安装ESP-IDF 5.4+
git clone -b v5.4 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh
. ./export.sh
```

### 编译烧录
```bash
cd ESP32_XiaoZhi
idf.py set-target esp32s3  # 根据你的芯片选择
idf.py build
idf.py flash monitor
```

### 配置网络
首次启动后：
1. 手机连接ESP32的WiFi热点
2. 访问配置页面设置网络
3. 选择大模型服务（官方/自建）

## 硬件支持

支持小智ESP32的所有硬件平台（70+款），包括：
- 立创实战派 ESP32-S3
- 乐鑫 ESP32-S3-BOX3
- M5Stack CoreS3
- M5Stack AtomS3R + Echo Base
- LILYGO T-Circle-S3
- 虾哥 Mini C3
- 更多...

详见：[支持的硬件列表](https://github.com/78/xiaozhi-esp32)

## 使用示例

### 语音交互示例

```
用户: "小智小智，帮我测一下心率"
小智: [调用 self.stm32.heart_rate.start]
      "好的，正在启动心率检测..."
      [收到数据 {DATA:HR:75,SPO2:98}]
      "您的心率是每分钟75次，血氧饱和度98%"

用户: "现在的温度是多少"
小智: [调用 self.stm32.temperature.get_state]
      "当前环境温度25度，湿度35%"

用户: "我在哪里"
小智: [调用 self.stm32.gps.start]
      "正在获取GPS定位..."
      [收到数据 {DATA:GPS:39.908722,116.397496,8}]
      "您当前位于北纬39.908722，东经116.397496，已连接8颗卫星"

系统: [收到 {EVENT:FALL}]
小智: "警告！检测到摔倒，请注意安全！" [播放警报音]
```

## 自定义开发

### 添加新传感器

1. 在STM32端添加传感器驱动和通信协议
2. 在 `stm32_controller.h` 中添加MCP工具定义：

```cpp
mcp_server.AddTool("self.stm32.new_sensor.start",
    "Start new sensor monitoring",
    PropertyList(),
    [this](const PropertyList& properties) -> ReturnValue {
        SendCommand("{CMD:NEW_SENSOR_START}\r\n");
        return true;
    });
```

3. 添加数据接收处理：

```cpp
else if (strstr(response, "{DATA:NEW_SENSOR:")) {
    // 解析数据并更新
}
```

## 服务器选择

### 官方服务器（推荐新手）
- 访问 [xiaozhi.me](https://xiaozhi.me)
- 注册账号免费使用Qwen实时模型
- 无需搭建服务器

### 自建服务器
参考第三方开源项目：
- [Python服务器](https://github.com/xinnan-tech/xiaozhi-esp32-server)
- [Java服务器](https://github.com/joey-zhou/xiaozhi-esp32-server-java)
- [Golang服务器](https://github.com/AnimeAIChat/xiaozhi-server-go)

## 开发文档

- [自定义开发板指南](docs/custom-board.md)
- [MCP协议使用说明](docs/mcp-usage.md)
- [WebSocket通信协议](docs/websocket.md)
- [完整功能文档](https://ccnphfhqs21z.feishu.cn/wiki/F5krwD16viZoF0kKkvDcrZNYnhb)

## 致谢

本项目基于 [xiaozhi-esp32](https://github.com/78/xiaozhi-esp32) 开源项目，感谢虾哥（78）的贡献。

## 许可证

遵循原项目的 MIT License
