#ifndef __OLED_H
#define __OLED_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ========== 硬件接口配置(根据实际硬件修改) ========== */
#define OLED_I2C_PORT           GPIOB                           // I2C引脚GPIO端口
#define OLED_I2C_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()   // GPIO时钟使能
#define OLED_SCL_PIN            GPIO_PIN_4                      // SCL引脚
#define OLED_SDA_PIN            GPIO_PIN_5                      // SDA引脚

/* I2C引脚操作宏(软件I2C) */
#define OLED_SCL_HIGH()     HAL_GPIO_WritePin(OLED_I2C_PORT, OLED_SCL_PIN, GPIO_PIN_SET)
#define OLED_SCL_LOW()      HAL_GPIO_WritePin(OLED_I2C_PORT, OLED_SCL_PIN, GPIO_PIN_RESET)
#define OLED_SDA_HIGH()     HAL_GPIO_WritePin(OLED_I2C_PORT, OLED_SDA_PIN, GPIO_PIN_SET)
#define OLED_SDA_LOW()      HAL_GPIO_WritePin(OLED_I2C_PORT, OLED_SDA_PIN, GPIO_PIN_RESET)
#define OLED_SDA_READ()     HAL_GPIO_ReadPin(OLED_I2C_PORT, OLED_SDA_PIN)  // 读取SDA引脚状态

/* I2C通信参数 */
#define OLED_I2C_ADDR       0x78    // OLED I2C从机地址(7位地址<<1)

/* 显示参数 */
#define OLED_WIDTH          128     // 屏幕宽度
#define OLED_HEIGHT         64      // 屏幕高度
#define OLED_MAX_LINE       4       // 最大行数
#define OLED_MAX_COLUMN     16      // 最大列数

/* ========== 函数声明 ========== */
void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowFloat(uint8_t Line, uint8_t Column, float Number, uint8_t IntLen, uint8_t DecLen);

/* 缓冲区控制函数（新增） */
void OLED_Flush(void);  // 将缓冲区内容刷新到屏幕

#endif