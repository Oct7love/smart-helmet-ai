/**
  ******************************************************************************
  * @file           : esp32_comm.h
  * @brief          : ESP32通信模块头文件
  * @author         : AI Assistant
  * @date           : 2025/01/18
  * @version        : v1.0
  ******************************************************************************
  * @attention
  *
  * 功能特点:
  * - 基于USART3通信（115200波特率）
  * - DMA接收+空闲中断
  * - 简单的命令协议 {CMD:xxx}\r\n
  * - 数据返回格式 {DATA:type:value}\r\n
  *
  * 硬件连接:
  * - ESP32 TX (GPIO17) --> STM32 RX (PC11/USART3_RX)
  * - ESP32 RX (GPIO18) <-- STM32 TX (PC10/USART3_TX)
  * - GND <--> GND
  *
  ******************************************************************************
  */

#ifndef ESP32_COMM_H
#define ESP32_COMM_H

#include "bsp_system.h"

/* 命令定义 ----------------------------------------------------------------*/
#define CMD_HR_START        "{CMD:HR_START}"        // 开始心率检测
#define CMD_HR_STOP         "{CMD:HR_STOP}"         // 停止心率检测
#define CMD_TEMP_START      "{CMD:TEMP_START}"      // 开始温度检测
#define CMD_TEMP_STOP       "{CMD:TEMP_STOP}"       // 停止温度检测
#define CMD_MPU_START       "{CMD:MPU_START}"       // 开始姿态检测
#define CMD_MPU_STOP        "{CMD:MPU_STOP}"        // 停止姿态检测
#define CMD_GPS_START       "{CMD:GPS_START}"       // 开始GPS定位
#define CMD_GPS_STOP        "{CMD:GPS_STOP}"        // 停止GPS定位
#define CMD_MQ2_START       "{CMD:MQ2_START}"       // 开始烟雾检测
#define CMD_MQ2_STOP        "{CMD:MQ2_STOP}"        // 停止烟雾检测
#define CMD_TEST_FALL       "{CMD:TEST_FALL}"       // 测试：模拟摔倒事件
#define CMD_TEST_COLLISION  "{CMD:TEST_COLLISION}"  // 测试：模拟碰撞事件

/* 传感器控制标志 ----------------------------------------------------------*/
extern volatile uint8_t heart_rate_enabled;
extern volatile uint8_t temperature_enabled;
extern volatile uint8_t mpu6050_enabled;
extern volatile uint8_t gps_enabled;
extern volatile uint8_t mq2_enabled;

/* 函数声明 ----------------------------------------------------------------*/
void ESP32_COMM_Init(void);
void ESP32_COMM_ProcessCommand(const char* cmd);
void ESP32_COMM_SendHeartRate(uint8_t hr, uint8_t spo2);
void ESP32_COMM_SendTemperature(float temp);
void ESP32_COMM_SendMPU6050(float pitch, float roll, float yaw);
void ESP32_COMM_SendGPS(float lat, float lon, uint8_t satellites);
void ESP32_COMM_SendStatus(const char* status);
void ESP32_COMM_SendString(const char* str);
void ESP32_COMM_SendFallEvent(void);
void ESP32_COMM_SendCollisionEvent(void);
void ESP32_COMM_SendSmokeEvent(void);

#endif // ESP32_COMM_H
