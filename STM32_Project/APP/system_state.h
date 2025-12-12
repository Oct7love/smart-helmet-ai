/**
  ******************************************************************************
  * @file           : system_state.h
  * @brief          : 系统状态管理头文件
  * @author         : AI Assistant
  * @date           : 2025/01/19
  * @version        : v1.0
  ******************************************************************************
  * @attention
  *
  * 功能：
  * - 定义传感器选择枚举
  * - 管理当前选中的传感器状态
  *
  ******************************************************************************
  */

#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <stdint.h>
#include "bsp_system.h"
/**
 * @brief 传感器类型枚举
 */
typedef enum {
    SENSOR_HR = 0,      // 心率血氧 (MAX30102)
    SENSOR_TEMP,        // 温湿度 (DHT11)
    SENSOR_MPU,         // 姿态检测 (MPU6050)
    SENSOR_MQ2,         // 烟雾检测 (MQ2)
    SENSOR_GPS,         // GPS定位
    SENSOR_MAX          // 传感器数量
} SelectedSensor_t;

/**
 * @brief 当前选中的传感器（用于按键控制和OLED显示）
 */
 extern volatile SelectedSensor_t current_selected_sensor;

#endif // SYSTEM_STATE_H
