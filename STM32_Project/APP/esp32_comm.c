/**
  ******************************************************************************
  * @file           : esp32_comm.c
  * @brief          : ESP32通信模块实现
  * @author         : AI Assistant
  * @date           : 2025/01/18
  * @version        : v1.0
  ******************************************************************************
  */

/* 头文件包含 ----------------------------------------------------------------*/
#include "esp32_comm.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

/* 控制标志 ------------------------------------------------------------------*/
volatile uint8_t heart_rate_enabled = 0;
volatile uint8_t temperature_enabled = 0;
volatile uint8_t mpu6050_enabled = 0;
volatile uint8_t gps_enabled = 0;
volatile uint8_t mq2_enabled = 0;

/* 私有变量 ------------------------------------------------------------------*/
extern UART_HandleTypeDef huart3;  // USART3句柄（在usart.c中定义）
extern uint8_t uart3_tx_buffer[256];

/**
 * @brief  初始化ESP32通信模块
 * @retval None
 */
void ESP32_COMM_Init(void)
{
    printf("[ESP32_COMM] Communication module initialized\r\n");
    printf("[ESP32_COMM] USART3: 115200-8-N-1, PC10(TX), PC11(RX)\r\n");
    printf("[ESP32_COMM] heart_rate_enabled 地址: %p, 初始值: %d\r\n",
           (void*)&heart_rate_enabled, heart_rate_enabled);

    // 启动USART3 DMA接收（在这里启动，避免初始化期间触发中断）
    extern uint8_t uart3_rx_buffer[256];
    extern DMA_HandleTypeDef hdma_usart3_rx;

    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart3_rx_buffer, sizeof(uart3_rx_buffer));
    if (status == HAL_OK) {
        printf("[ESP32_COMM] DMA接收启动成功\r\n");
    } else {
        printf("[ESP32_COMM] DMA接收启动失败! Status=%d\r\n", status);
    }
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT); // 禁用半传输中断

    // 发送测试消息
    HAL_Delay(100);  // 等待稳定
    const char* test_msg = "[STM32] USART3 Ready!\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t*)test_msg, strlen(test_msg), 100);
    printf("[ESP32_COMM] 发送测试消息: %s", test_msg);

    // 发送启动消息给ESP32
    ESP32_COMM_SendStatus("STM32_READY");
    printf("[ESP32_COMM] Init Complete\r\n");
}

/**
 * @brief  处理来自ESP32的命令
 * @param  cmd: 命令字符串
 * @retval None
 */
void ESP32_COMM_ProcessCommand(const char* cmd)
{
    printf("[ESP32_COMM] ========== 命令处理开始 ==========\r\n");
    printf("[ESP32_COMM] RX: %s\r\n", cmd);
    printf("[ESP32_COMM] 命令长度: %d\r\n", strlen(cmd));

    // 打印十六进制
    printf("[ESP32_COMM] HEX: ");
    for(int i = 0; i < strlen(cmd) && i < 50; i++) {
        printf("%02X ", (uint8_t)cmd[i]);
    }
    printf("\r\n");

    // 心率检测命令
    printf("[ESP32_COMM] 检查 CMD_HR_START = \"%s\"\r\n", CMD_HR_START);
    char* found = strstr(cmd, CMD_HR_START);
    printf("[ESP32_COMM] strstr() 返回: %p\r\n", found);

    if (found) {
        printf("[ESP32_COMM] ✓ 匹配到心率启动命令!\r\n");
        printf("[ESP32_COMM] 设置前 heart_rate_enabled = %d\r\n", heart_rate_enabled);
        heart_rate_enabled = 1;
        printf("[ESP32_COMM] 设置后 heart_rate_enabled = %d\r\n", heart_rate_enabled);
        ESP32_COMM_SendStatus("OK");
        printf("[ESP32_COMM] Heart rate detection started\r\n");
    }
    else if (strstr(cmd, CMD_HR_STOP)) {
        heart_rate_enabled = 0;
        ESP32_COMM_SendStatus("OK");
        printf("[ESP32_COMM] Heart rate detection stopped\r\n");
    }
    // 温度检测命令
    else if (strstr(cmd, CMD_TEMP_START)) {
        temperature_enabled = 1;
        ESP32_COMM_SendStatus("OK");
        printf("[ESP32_COMM] Temperature detection started\r\n");
    }
    else if (strstr(cmd, CMD_TEMP_STOP)) {
        temperature_enabled = 0;
        ESP32_COMM_SendStatus("OK");
        printf("[ESP32_COMM] Temperature detection stopped\r\n");
    }
    // 姿态检测命令
    else if (strstr(cmd, CMD_MPU_START)) {
        mpu6050_enabled = 1;
        ESP32_COMM_SendStatus("OK");
        printf("[ESP32_COMM] MPU6050 detection started\r\n");
    }
    else if (strstr(cmd, CMD_MPU_STOP)) {
        mpu6050_enabled = 0;
        ESP32_COMM_SendStatus("OK");
        printf("[ESP32_COMM] MPU6050 detection stopped\r\n");
    }
    // GPS定位命令
    else if (strstr(cmd, CMD_GPS_START)) {
        gps_enabled = 1;
        ESP32_COMM_SendStatus("OK");
        printf("[ESP32_COMM] GPS positioning started\r\n");
    }
    else if (strstr(cmd, CMD_GPS_STOP)) {
        gps_enabled = 0;
        ESP32_COMM_SendStatus("OK");
        printf("[ESP32_COMM] GPS positioning stopped\r\n");
    }
    // 烟雾检测命令
    else if (strstr(cmd, CMD_MQ2_START)) {
        mq2_enabled = 1;
        ESP32_COMM_SendStatus("OK");
        printf("[ESP32_COMM] MQ2 smoke detection started\r\n");
    }
    else if (strstr(cmd, CMD_MQ2_STOP)) {
        mq2_enabled = 0;
        ESP32_COMM_SendStatus("OK");
        printf("[ESP32_COMM] MQ2 smoke detection stopped\r\n");
    }
    // 测试命令
    else if (strstr(cmd, CMD_TEST_FALL)) {
        ESP32_COMM_SendFallEvent();
        printf("[ESP32_COMM] 测试：手动触发摔倒事件\r\n");
    }
    else if (strstr(cmd, CMD_TEST_COLLISION)) {
        ESP32_COMM_SendCollisionEvent();
        printf("[ESP32_COMM] 测试：手动触发碰撞事件\r\n");
    }
    // 未知命令
    else {
        ESP32_COMM_SendStatus("ERROR");
        printf("[ESP32_COMM] Unknown command: %s\r\n", cmd);
    }
}

/**
 * @brief  发送心率数据给ESP32
 * @param  hr: 心率值（bpm）
 * @param  spo2: 血氧值（%）
 * @retval None
 */
void ESP32_COMM_SendHeartRate(uint8_t hr, uint8_t spo2)
{
    char buffer[64];
    int len = sprintf(buffer, "{DATA:HR:%d,SPO2:%d}\r\n", hr, spo2);

    // 使用阻塞发送，避免DMA异步问题
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, len, 100);
}

/**
 * @brief  发送温度数据给ESP32
 * @param  temp: 温度值（°C）
 * @retval None
 */
void ESP32_COMM_SendTemperature(float temp)
{
    char buffer[32];
    int len = sprintf(buffer, "{DATA:TEMP:%.1f}\r\n", temp);

    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, len, 100);
}

/**
 * @brief  发送姿态数据给ESP32
 * @param  pitch: 俯仰角（°）
 * @param  roll: 横滚角（°）
 * @param  yaw: 航向角（°）
 * @retval None
 */
void ESP32_COMM_SendMPU6050(float pitch, float roll, float yaw)
{
    char buffer[64];
    int len = sprintf(buffer, "{DATA:MPU:%.1f,%.1f,%.1f}\r\n", pitch, roll, yaw);

    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, len, 100);
}

/**
 * @brief  发送GPS数据给ESP32
 * @param  lat: 纬度
 * @param  lon: 经度
 * @param  satellites: 卫星数量
 * @retval None
 */
void ESP32_COMM_SendGPS(float lat, float lon, uint8_t satellites)
{
    char buffer[64];
    int len = sprintf(buffer, "{DATA:GPS:%.6f,%.6f,%d}\r\n", lat, lon, satellites);

    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, len, 100);
}

/**
 * @brief  发送状态消息给ESP32
 * @param  status: 状态字符串（如 "OK", "ERROR"）
 * @retval None
 */
void ESP32_COMM_SendStatus(const char* status)
{
    char buffer[32];
    int len = sprintf(buffer, "{STATUS:%s}\r\n", status);

    HAL_StatusTypeDef tx_status = HAL_UART_Transmit(&huart3, (uint8_t*)buffer, len, 100);
    if (tx_status == HAL_OK) {
        printf("[ESP32_COMM] 发送状态: %s", buffer);
    } else {
        printf("[ESP32_COMM] 发送失败! Status=%d\r\n", tx_status);
    }
}

/**
 * @brief  发送任意字符串给ESP32
 * @param  str: 字符串内容
 * @retval None
 */
void ESP32_COMM_SendString(const char* str)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 100);
}

/**
 * @brief  发送摔倒事件给ESP32
 * @retval None
 */
void ESP32_COMM_SendFallEvent(void)
{
    const char* event_msg = "{EVENT:FALL}\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t*)event_msg, strlen(event_msg), 100);
    printf("[ESP32_COMM] ⚠️ 发送摔倒事件\r\n");
}

/**
 * @brief  发送碰撞事件给ESP32
 * @retval None
 */
void ESP32_COMM_SendCollisionEvent(void)
{
    const char* event_msg = "{EVENT:COLLISION}\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t*)event_msg, strlen(event_msg), 100);
    printf("[ESP32_COMM] ⚠️ 发送碰撞事件\r\n");
}

/**
 * @brief  发送烟雾事件给ESP32
 * @retval None
 */
void ESP32_COMM_SendSmokeEvent(void)
{
    const char* event_msg = "{EVENT:SMOKE}\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t*)event_msg, strlen(event_msg), 100);
    printf("[ESP32_COMM] ⚠️ 发送烟雾事件\r\n");
}
