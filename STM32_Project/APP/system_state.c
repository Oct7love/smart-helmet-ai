/**
  ******************************************************************************
  * @file           : system_state.c
  * @brief          : 系统状态管理实现
  * @author         : AI Assistant
  * @date           : 2025/01/19
  * @version        : v1.0
  ******************************************************************************
  */

#include "system_state.h"

/**
 * @brief 当前选中的传感器
 * @note  默认选中心率传感器
 */
volatile SelectedSensor_t current_selected_sensor = SENSOR_HR;
