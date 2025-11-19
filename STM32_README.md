# STM32 多传感器控制端

STM32F103智能安全帽传感器控制系统，负责采集多种传感器数据并通过UART与ESP32通信。

## 硬件配置

- **主控**: STM32F103C8T6
- **传感器**:
  - MAX30102: 心率和血氧检测
  - MPU6050: 六轴姿态传感器（陀螺仪+加速度计）
  - MQ2: 可燃气体/烟雾传感器
  - DHT11: 温湿度传感器
  - GPS模块: NEO-6M/7M/8M
- **显示**: OLED 0.96寸 (SSD1306)
- **通信**: USART3 (115200bps) 连接ESP32

## 功能特性

### 1. 心率血氧监测 (MAX30102)
- 实时采集心率和血氧饱和度
- 优化的信号处理算法
- 自动波动抑制和抗丢失机制

### 2. 姿态检测 (MPU6050 DMP)
- 实时姿态解算（俯仰角/横滚角/航向角）
- 跌倒检测算法
- 碰撞检测算法

### 3. 环境监测
- 温湿度实时采集 (DHT11)
- 烟雾/可燃气体浓度检测 (MQ2)
- 多级报警机制

### 4. GPS定位
- 实时位置追踪
- 卫星数量显示
- NMEA协议解析

## 项目结构

```
STM32_Project/
├── APP/                        # 应用层代码
│   ├── esp32_comm.c/h          # ESP32通信模块
│   ├── max30102_app.c/h        # 心率血氧应用
│   ├── mpu_app.c/h             # 姿态检测应用
│   ├── mq2_app.c/h             # 烟雾检测应用
│   ├── dht11_app.c/h           # 温湿度应用
│   ├── gps_app.c/h             # GPS应用
│   ├── oled.c/h                # OLED显示驱动
│   ├── scheduler.c/h           # 任务调度器
│   └── eMPL_MPU/               # MPU6050 DMP库
├── Core/                       # STM32 HAL核心
│   ├── Inc/                    # 头文件
│   └── Src/                    # 源文件
├── Drivers/                    # STM32 HAL驱动
│   ├── STM32F1xx_HAL_Driver/
│   └── CMSIS/
└── CMakeLists.txt              # CMake构建文件
```

## 通信协议

### ESP32发送的命令
```c
{CMD:HR_START}       // 启动心率检测
{CMD:HR_STOP}        // 停止心率检测
{CMD:TEMP_START}     // 启动温度检测
{CMD:MPU_START}      // 启动姿态检测
{CMD:GPS_START}      // 启动GPS
{CMD:MQ2_START}      // 启动烟雾检测
```

### STM32返回的数据
```c
{DATA:HR:75,SPO2:98}            // 心率和血氧
{DATA:TEMP:25,35}               // 温度和湿度
{DATA:MPU:10.5,-5.2,180.0}      // 姿态角度
{DATA:GPS:39.908722,116.397496,8} // GPS坐标
{DATA:MQ2:150.5,2}              // 烟雾PPM和报警级别
{STATUS:OK}                     // 命令响应
{EVENT:FALL}                    // 跌倒事件
{EVENT:COLLISION}               // 碰撞事件
{EVENT:SMOKE}                   // 烟雾报警
```

## 任务调度

系统使用轻量级任务调度器，各传感器按不同周期执行：

```c
static task_t scheduler_task[] = {
    {mq2_task, 500, 0},        // 烟雾检测 500ms
    {dht11_task, 1000, 0},     // 温湿度 1000ms
    {mpu6050_task, 100, 0},    // 姿态检测 100ms
    {max30102_task, 100, 0},   // 心率血氧 100ms
    {gps_task, 500, 0},        // GPS定位 500ms
};
```

## 编译与烧录

### 环境要求
- CMake 3.22+
- ARM GCC工具链
- STM32 CubeProgrammer / OpenOCD

### 编译步骤
```bash
cd STM32_Project
mkdir build && cd build
cmake ..
make
```

### 烧录
```bash
# 使用STM32 CubeProgrammer
# 或使用OpenOCD
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program STM32_Class_Project.elf verify reset exit"
```

## 硬件连接

### UART3 (ESP32通信)
- PC10: TX → ESP32 GPIO18 (RX)
- PC11: RX ← ESP32 GPIO17 (TX)

### I2C1 (传感器总线)
- PB6: SCL
- PB7: SDA
- 连接: MAX30102, MPU6050

### I2C2 (OLED显示)
- PB10: SCL
- PB11: SDA

### ADC1 (模拟传感器)
- PA0: MQ2模拟输出

### GPIO
- PA1: DHT11数据引脚
- USART2: GPS模块

## 传感器校准

### MQ2烟雾传感器
预热时间：20秒
基准值：在清洁空气中测量
报警阈值可在 `mq2_app.h` 中配置

### MAX30102心率传感器
需要手指紧密接触
首次检测需要5-10秒稳定时间

## 开发文档

详细文档请参考 `md文档/` 目录：
- MAX30102配置说明.md
- OLED显示心率血氧_使用说明.md
- printf重定向使用说明.md

## 许可证

基于STM32 HAL库开发，遵循ST License
应用层代码采用MIT License
