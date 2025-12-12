//
// Created by 13615 on 2025/11/1.
//

#ifndef STM32_CLASS_PROJECT_BSP_SYSTEM_H
#define STM32_CLASS_PROJECT_BSP_SYSTEM_H

#include "main.h"
#include "stdio.h"
#include "string.h"
#include "stdarg.h"
#include "stdint.h"
#include "stdlib.h"
#include "math.h"
#include "delay.h"

/*系统生成*/
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "scheduler.h"
#include "oledfont.h"
#include "oled.h"
#include "ringbuffer/ringbuffer.h"
// 传感器驱动
#include "mq2_app.h"    // 烟雾传感器驱动头文件
#include "dht11_app.h"  // 温湿度传感器驱动头文件
#include "mpu_app.h"    // MPU6050传感器驱动头文件
#include "max30102_app.h"
#include "gps_app.h"    // ATGM336H GPS定位模块驱动头文件
#include "system_state.h"
// ESP32通信模块
#include "esp32_comm.h"  // ESP32串口通信模块

#endif //STM32_CLASS_PROJECT_BSP_SYSTEM_H