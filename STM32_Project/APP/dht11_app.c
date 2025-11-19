#include "dht11_app.h"
#include "esp32_comm.h"  // 添加ESP32通信头文件
#include <stdio.h>

/* 引入延时函数 */
extern void delay_us(uint32_t us);  // 如果delay.h在其他地方，就include进来

/* 私有函数声明 */
static void DHT11_GPIO_Init(void);
static void DHT11_SetPinMode(uint32_t mode);
static void DHT11_Reset(void);
static uint8_t DHT11_Check(void);
static uint8_t DHT11_ReadBit(void);
static uint8_t DHT11_ReadByte(void);

/* 公共变量 */
DHT11_Data_t dht11_data = {0};

/**
  * @brief  初始化DHT11 GPIO
  */
static void DHT11_GPIO_Init(void)
{
    DHT11_GPIO_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStruct);

    DHT11_PIN_HIGH();
}

/**
  * @brief  设置DHT11引脚模式
  */
static void DHT11_SetPinMode(uint32_t mode)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_GPIO_PIN;
    GPIO_InitStruct.Mode = mode;
    GPIO_InitStruct.Pull = (mode == GPIO_MODE_INPUT) ? GPIO_PULLUP : GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  复位DHT11
  */
static void DHT11_Reset(void)
{
    DHT11_SetPinMode(GPIO_MODE_OUTPUT_PP);
    DHT11_PIN_LOW();
    HAL_Delay(20);  // 改用HAL_Delay，拉低20ms
    DHT11_PIN_HIGH();
    delay_us(30);   // 拉高30us
}

/**
  * @brief  等待DHT11响应
  */
static uint8_t DHT11_Check(void)
{
    uint8_t retry = 0;

    DHT11_SetPinMode(GPIO_MODE_INPUT);

    // 等待DHT11拉低 (40~80us)
    while(DHT11_PIN_READ() == GPIO_PIN_SET && retry < 100)
    {
        retry++;
        delay_us(1);
    }
    if(retry >= 100) {
        printf("DHT11 Check Fail: No LOW\r\n");
        return 1;
    }

    retry = 0;
    // 等待DHT11拉高 (40~80us)
    while(DHT11_PIN_READ() == GPIO_PIN_RESET && retry < 100)
    {
        retry++;
        delay_us(1);
    }
    if(retry >= 100) {
        printf("DHT11 Check Fail: No HIGH\r\n");
        return 1;
    }

    return 0;
}

/**
  * @brief  从DHT11读取一个位
  */
static uint8_t DHT11_ReadBit(void)
{
    uint8_t retry = 0;

    // 等待变为低电平
    while(DHT11_PIN_READ() == GPIO_PIN_SET && retry < 100)
    {
        retry++;
        delay_us(1);
    }

    retry = 0;
    // 等待变为高电平
    while(DHT11_PIN_READ() == GPIO_PIN_RESET && retry < 100)
    {
        retry++;
        delay_us(1);
    }

    delay_us(40);  // 延时40us后采样

    if(DHT11_PIN_READ() == GPIO_PIN_SET) return 1;
    else return 0;
}

/**
  * @brief  从DHT11读取一个字节
  */
static uint8_t DHT11_ReadByte(void)
{
    uint8_t i, dat = 0;

    for(i = 0; i < 8; i++)
    {
        dat <<= 1;
        dat |= DHT11_ReadBit();
    }

    return dat;
}

/**
  * @brief  初始化DHT11
  */
uint8_t DHT11_Init(void)
{
    DHT11_GPIO_Init();

    dht11_data.temperature = 0;
    dht11_data.humidity = 0;
    dht11_data.is_valid = 0;

    HAL_Delay(1000);  // 上电后等待1秒

    DHT11_Reset();
    return DHT11_Check();
}

/**
  * @brief  更新DHT11数据
  */
uint8_t DHT11_Update(void)
{
    uint8_t buf[5];
    uint8_t i;

    DHT11_Reset();

    if(DHT11_Check() == 0)
    {
        // 读取40位数据
        for(i = 0; i < 5; i++)
        {
            buf[i] = DHT11_ReadByte();
        }

        // 校验和验证
        uint8_t checksum = buf[0] + buf[1] + buf[2] + buf[3];
        if(checksum == buf[4])
        {
            dht11_data.humidity = buf[0];
            dht11_data.temperature = buf[2];

            if(DHT11_IS_TEMP_VALID(dht11_data.temperature) &&
               DHT11_IS_HUMI_VALID(dht11_data.humidity))
            {
                dht11_data.is_valid = 1;
                return 0;
            }
        }
        else
        {
            printf("Checksum Error: %d != %d\r\n", checksum, buf[4]);
        }
    }

    dht11_data.is_valid = 0;
    return 1;
}

/**
  * @brief  打印DHT11信息
  */
void DHT11_PrintInfo(void)
{
    if(dht11_data.is_valid)
    {
        printf("Temp: %d°C, Humi: %d%%\r\n",
               dht11_data.temperature, dht11_data.humidity);
    }
    else
    {
        printf("DHT11 Read Error!\r\n");
    }
}

/**
  * @brief  DHT11任务
  */
void dht11_task(void)
{
    // 检查是否启用了温度检测
    extern volatile uint8_t temperature_enabled;
    if (!temperature_enabled) {
        return;  // 未启用，直接返回
    }

    // 更新DHT11数据
    DHT11_Update();

    // 定期发送数据给ESP32（每1秒发送一次）
    static uint32_t last_send_tick = 0;
    if (HAL_GetTick() - last_send_tick >= 1000) {
        if (dht11_data.is_valid) {
            // 发送温度和湿度数据到ESP32
            char data_msg[64];
            int len = sprintf(data_msg, "{DATA:TEMP:%d,%d}\r\n",
                            dht11_data.temperature, dht11_data.humidity);
            extern UART_HandleTypeDef huart3;
            HAL_UART_Transmit(&huart3, (uint8_t*)data_msg, len, 100);

            // 打印调试信息
            printf("[DHT11] 温度: %d°C, 湿度: %d%%\r\n",
                   dht11_data.temperature, dht11_data.humidity);
        }
        last_send_tick = HAL_GetTick();
    }
}