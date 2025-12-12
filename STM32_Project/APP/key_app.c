/**
  ******************************************************************************
  * @file           : key_app.c
  * @brief          : 按键处理模块实现
  * @author         : AI Assistant
  * @date           : 2025/01/19
  * @version        : v1.0
  ******************************************************************************
  */

#include "key_app.h"
#include "system_state.h"
#include "esp32_comm.h"
#include "main.h"
#include <stdio.h>

/* 按键去抖时间 (毫秒) */
#define KEY_DEBOUNCE_MS 20

/* 防止快速重复按键的保护时间 (毫秒) */
#define KEY_REPEAT_PROTECT_MS 200

/* 按键状态跟踪 */
static uint8_t key1_last = 1;       // 上一次读取的电平
static uint8_t key2_last = 1;
static uint32_t key1_last_tick = 0; // 上一次电平变化的时间
static uint32_t key2_last_tick = 0;

/**
 * @brief KEY1按下事件处理（启动/停止当前选中的传感器）
 * @note  通过调用ESP32_COMM_ProcessCommand来复用现有逻辑
 */
static void on_key1_pressed(void)
{
    // 防止快速重复按键
    static uint32_t last_press_time = 0;
    uint32_t now = HAL_GetTick();

    if (now - last_press_time < KEY_REPEAT_PROTECT_MS) {
        return;
    }
    last_press_time = now;

    printf("[KEY] KEY1 pressed, sensor=%d\r\n", current_selected_sensor);

    // 根据当前选中的传感器执行启动/停止操作
    switch (current_selected_sensor) {
        case SENSOR_HR:
            // 读取当前状态并切换
            if (heart_rate_enabled) {
                printf("[KEY] Stopping HR sensor\r\n");
                ESP32_COMM_ProcessCommand("{CMD:HR_STOP}");
            } else {
                printf("[KEY] Starting HR sensor\r\n");
                ESP32_COMM_ProcessCommand("{CMD:HR_START}");
            }
            break;

        case SENSOR_TEMP:
            if (temperature_enabled) {
                printf("[KEY] Stopping TEMP sensor\r\n");
                ESP32_COMM_ProcessCommand("{CMD:TEMP_STOP}");
            } else {
                printf("[KEY] Starting TEMP sensor\r\n");
                ESP32_COMM_ProcessCommand("{CMD:TEMP_START}");
            }
            break;

        case SENSOR_MPU:
            if (mpu6050_enabled) {
                printf("[KEY] Stopping MPU sensor\r\n");
                ESP32_COMM_ProcessCommand("{CMD:MPU_STOP}");
            } else {
                printf("[KEY] Starting MPU sensor\r\n");
                ESP32_COMM_ProcessCommand("{CMD:MPU_START}");
            }
            break;

        case SENSOR_MQ2:
            if (mq2_enabled) {
                printf("[KEY] Stopping MQ2 sensor\r\n");
                ESP32_COMM_ProcessCommand("{CMD:MQ2_STOP}");
            } else {
                printf("[KEY] Starting MQ2 sensor\r\n");
                ESP32_COMM_ProcessCommand("{CMD:MQ2_START}");
            }
            break;

        case SENSOR_GPS:
            if (gps_enabled) {
                printf("[KEY] Stopping GPS sensor\r\n");
                ESP32_COMM_ProcessCommand("{CMD:GPS_STOP}");
            } else {
                printf("[KEY] Starting GPS sensor\r\n");
                ESP32_COMM_ProcessCommand("{CMD:GPS_START}");
            }
            break;

        default:
            break;
    }
}

/**
 * @brief KEY2按下事件处理（切换选中的传感器）
 */
static void on_key2_pressed(void)
{
    // 防止快速重复按键
    static uint32_t last_press_time = 0;
    uint32_t now = HAL_GetTick();

    if (now - last_press_time < KEY_REPEAT_PROTECT_MS) {
        return;
    }
    last_press_time = now;

    // 切换到下一个传感器
    current_selected_sensor = (current_selected_sensor + 1) % SENSOR_MAX;

    printf("[KEY] KEY2 pressed, switched to sensor=%d\r\n", current_selected_sensor);

    // 可选：发送切换事件到ESP32
    const char* sensor_names[] = {"HR", "TEMP", "MPU", "MQ2", "GPS"};
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "{EVENT:SELECT:%s}", sensor_names[current_selected_sensor]);
    ESP32_COMM_SendString(buffer);
    ESP32_COMM_SendString("\r\n");
}

/**
 * @brief 按键扫描任务（带去抖动和边沿检测）
 * @note  应被添加到scheduler中，建议20ms周期调用
 */
void key_task(void)
{
    uint32_t now = HAL_GetTick();

    // 读取按键当前电平（假设：按下=低电平，松开=高电平）
    uint8_t k1 = HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);
    uint8_t k2 = HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin);

    // ==================== KEY1 去抖和边沿检测 ====================
    if (k1 != key1_last) {
        // 电平变化，重置计时器
        key1_last = k1;
        key1_last_tick = now;
    } else if ((now - key1_last_tick) > KEY_DEBOUNCE_MS) {
        // 电平稳定超过去抖时间
        static uint8_t key1_stable = 1;  // 上一次稳定的电平

        if (k1 != key1_stable) {
            // 稳定电平发生变化，检测到边沿
            key1_stable = k1;

            if (k1 == GPIO_PIN_RESET) {
                // 下降沿：按键按下
                on_key1_pressed();
            }
            // 上升沿（按键松开）不处理
        }
    }

    // ==================== KEY2 去抖和边沿检测 ====================
    if (k2 != key2_last) {
        // 电平变化，重置计时器
        key2_last = k2;
        key2_last_tick = now;
    } else if ((now - key2_last_tick) > KEY_DEBOUNCE_MS) {
        // 电平稳定超过去抖时间
        static uint8_t key2_stable = 1;  // 上一次稳定的电平

        if (k2 != key2_stable) {
            // 稳定电平发生变化，检测到边沿
            key2_stable = k2;

            if (k2 == GPIO_PIN_RESET) {
                // 下降沿：按键按下
                on_key2_pressed();
            }
            // 上升沿（按键松开）不处理
        }
    }
}
