#ifndef STM32_CLASS_PROJECT_DHT11_APP_H
#define STM32_CLASS_PROJECT_DHT11_APP_H

#include "bsp_system.h"

/* 硬件配置 ------------------------------------------------------------------*/
#define DHT11_GPIO_PORT         GPIOA               // DHT11引脚GPIO端口
#define DHT11_GPIO_PIN          GPIO_PIN_8         // DHT11引脚
#define DHT11_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE() // GPIO时钟使能

/* 时序参数 (单位:us) -------------------------------------------------------*/
#define DHT11_RESET_TIME        20000   // 复位时拉低时间 (20ms)
#define DHT11_WAIT_TIME         30      // 复位后等待时间 (30us)
#define DHT11_TIMEOUT           100     // 超时时间 (100us)
#define DHT11_BIT_SAMPLE_TIME   40      // 采样延时 (40us)

/* 引脚操作宏 ----------------------------------------------------------------*/
#define DHT11_PIN_HIGH()    HAL_GPIO_WritePin(DHT11_GPIO_PORT, DHT11_GPIO_PIN, GPIO_PIN_SET)
#define DHT11_PIN_LOW()     HAL_GPIO_WritePin(DHT11_GPIO_PORT, DHT11_GPIO_PIN, GPIO_PIN_RESET)
#define DHT11_PIN_READ()    HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN)

/* 数据范围宏 ----------------------------------------------------------------*/
#define DHT11_TEMP_MIN      0       // 温度最小值 (°C)
#define DHT11_TEMP_MAX      50      // 温度最大值 (°C)
#define DHT11_HUMI_MIN      20      // 湿度最小值 (%)
#define DHT11_HUMI_MAX      90      // 湿度最大值 (%)

/* 数据验证宏 ----------------------------------------------------------------*/
#define DHT11_IS_TEMP_VALID(t)  ((t) >= DHT11_TEMP_MIN && (t) <= DHT11_TEMP_MAX)
#define DHT11_IS_HUMI_VALID(h)  ((h) >= DHT11_HUMI_MIN && (h) <= DHT11_HUMI_MAX)

/* 类型定义 ------------------------------------------------------------------*/
typedef struct {
    uint8_t temperature;    // 温度值 (°C)
    uint8_t humidity;       // 湿度值 (%)
    uint8_t is_valid;       // 数据有效标志 (1=有效, 0=无效)
} DHT11_Data_t;

/* 公共变量 ------------------------------------------------------------------*/
extern DHT11_Data_t dht11_data;

/* 函数声明 ------------------------------------------------------------------*/
uint8_t DHT11_Init(void);                           // 初始化DHT11
uint8_t DHT11_Update(void);                         // 更新DHT11数据
void DHT11_PrintInfo(void);                         // 打印DHT11信息
void dht11_task(void);                              // DHT11任务 (兼容scheduler)

/* 内联辅助函数 --------------------------------------------------------------*/
/**
  * @brief  获取当前温度
  * @retval 温度值 (°C)
  */
static inline uint8_t DHT11_GetTemperature(void)
{
    return dht11_data.temperature;
}

/**
  * @brief  获取当前湿度
  * @retval 湿度值 (%)
  */
static inline uint8_t DHT11_GetHumidity(void)
{
    return dht11_data.humidity;
}

/**
  * @brief  检查数据有效性
  * @retval 1=有效, 0=无效
  */
static inline uint8_t DHT11_IsDataValid(void)
{
    return dht11_data.is_valid;
}

#endif /* STM32_CLASS_PROJECT_DHT11_APP_H */