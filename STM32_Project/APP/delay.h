#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f1xx_hal.h"

/* 使用TIM3定时器延时 - 不占用DWT调试资源 */
extern TIM_HandleTypeDef htim3;  // CubeMX生成的TIM3句柄

/**
  * @brief  初始化TIM3延时
  * @note   必须在main函数中MX_TIM3_Init()之后调用
  * @note   CubeMX配置要求：PSC=71, ARR=65535
  *         72MHz/(71+1) = 1MHz，计数器每次+1 = 1μs
  */
static inline void delay_init(void)
{
    HAL_TIM_Base_Start(&htim3);  // 启动定时器3
}

/**
  * @brief  精确微秒延时
  * @param  us 延时微秒数（最大65535μs = 65.5ms）
  * @note   基于硬件定时器，不占用DWT调试单元
  */
static inline void delay_us(uint32_t us)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0);  // 复位计数器
    while(__HAL_TIM_GET_COUNTER(&htim3) < us);  // 等待计数达到
}

/**
  * @brief  毫秒延时
  * @param  ms 延时毫秒数
  */
static inline void delay_ms(uint32_t ms)
{
    while(ms--)
    {
        delay_us(1000);
    }
}

#endif /* __DELAY_H */