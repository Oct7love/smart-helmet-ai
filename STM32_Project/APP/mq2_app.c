/**
  ******************************************************************************
  * @file           : mq2_app.c
  * @brief          : MQ-2气体传感器驱动实现
  * @author         : 13615
  * @date           : 2025/11/2
  * @version        : v2.0
  ******************************************************************************
  */

/* 头文件包含 ----------------------------------------------------------------*/
#include "mq2_app.h"
#include "adc.h"
#include "esp32_comm.h"  // 添加ESP32通信头文件
#include <stdio.h>

/* 私有变量 ------------------------------------------------------------------*/
static uint32_t adc_buffer[MQ2_SAMPLE_COUNT];  // ADC的DMA缓冲区

/* 公共变量 ------------------------------------------------------------------*/
MQ2_Data_t mq2_data = {0};  // 全局传感器数据结构

/* 私有函数声明 --------------------------------------------------------------*/
static uint32_t MQ2_ReadADC_Average(void);
static void MQ2_Calculate(void);

/**
  * @brief  初始化MQ-2传感器
  * @note   系统初始化时调用一次
  * @retval 无
  */
void MQ2_Init(void)
{
    // 复位数据结构
    mq2_data.adc_raw = 0;
    mq2_data.adc_voltage = 0.0f;
    mq2_data.sensor_voltage = 0.0f;
    mq2_data.rs = 0.0f;
    mq2_data.ratio = 0.0f;
    mq2_data.ppm = 0.0f;
    mq2_data.alarm = MQ2_ALARM_SAFE;
}

/**
  * @brief  更新传感器读数（周期性调用）
  * @note   主循环中调用此函数
  * @retval 无
  */
void MQ2_Update(void)
{
    // 读取ADC并取均值
    mq2_data.adc_raw = MQ2_ReadADC_Average();
    
    // 执行所有计算
    MQ2_Calculate();
}

/**
  * @brief  获取当前PPM值
  * @retval 气体浓度（ppm）
  */
float MQ2_GetPPM(void)
{
    return mq2_data.ppm;
}

/**
  * @brief  获取当前报警等级
  * @retval 报警等级枚举
  */
MQ2_AlarmLevel_t MQ2_GetAlarmLevel(void)
{
    return mq2_data.alarm;
}

/**
  * @brief  打印传感器信息
  * @retval 无
  */
void MQ2_PrintInfo(void)
{
    printf("ppm :%f\r\n", mq2_data.ppm);
}

/**
  * @brief  通过DMA读取ADC值并计算均值
  * @retval ADC均值
  */
static uint32_t MQ2_ReadADC_Average(void)
{
    // 启动DMA传输（只在第一次调用时启动，后续DMA会连续采样）
    static uint8_t dma_started = 0;
    if (!dma_started) {
        HAL_ADC_Start_DMA(&MQ2_ADC_HANDLE, adc_buffer, MQ2_SAMPLE_COUNT);
        dma_started = 1;
    }

    // 直接读取DMA缓冲区（DMA在后台连续采样）
    // 不需要等待，避免阻塞其他任务

    // 计算总和
    uint32_t sum = 0;
    for (uint8_t i = 0; i < MQ2_SAMPLE_COUNT; i++)
    {
        sum += adc_buffer[i];
    }

    // 返回均值
    return (sum / MQ2_SAMPLE_COUNT);
}

/**
  * @brief  计算所有传感器参数
  * @note   使用宏定义计算，提高效率和可读性
  * @retval 无
  */
static void MQ2_Calculate(void)
{
    // 步骤1: ADC值转换为电压 (0-3.3V)
    mq2_data.adc_voltage = MQ2_ADC_TO_VOLTAGE(mq2_data.adc_raw);
    
    // 步骤2: 传感器电压与ADC电压相同（无分压电路）
    // 如果有分压电路，在此处调整:
    // mq2_data.sensor_voltage = mq2_data.adc_voltage * 分压比;
    mq2_data.sensor_voltage = mq2_data.adc_voltage;
    
    // 步骤3: 验证电压有效性
    if (!MQ2_IS_VOLTAGE_VALID(mq2_data.sensor_voltage))
    {
        mq2_data.ppm = 0.0f;
        mq2_data.alarm = MQ2_ALARM_SAFE;
        return;
    }
    
    // 步骤4: 计算传感器电阻Rs
    mq2_data.rs = MQ2_VOLTAGE_TO_RS(mq2_data.sensor_voltage);
    
    // 步骤5: 计算Rs/R0比值
    mq2_data.ratio = MQ2_RS_TO_RATIO(mq2_data.rs);
    
    // 步骤6: 从比值计算PPM
    mq2_data.ppm = MQ2_RATIO_TO_PPM(mq2_data.ratio);
    
    // 步骤7: 验证并限幅PPM
    if (!MQ2_IS_PPM_VALID(mq2_data.ppm))
    {
        mq2_data.ppm = (mq2_data.ppm < 0.0f) ? 0.0f : 9999.0f;
    }
    
    // 步骤8: 确定报警等级
    mq2_data.alarm = MQ2_CheckAlarm(mq2_data.ppm);
}

/* 兼容旧版本的函数 ----------------------------------------------------------*/
/**
  * @brief  旧版任务函数（向后兼容）
  * @note   推荐使用MQ2_Update()代替
  * @retval 无
  */
void mq2_task(void)
{
    // 检查是否启用了烟雾检测
    extern volatile uint8_t mq2_enabled;
    if (!mq2_enabled) {
        return;  // 未启用，直接返回
    }

    // 更新MQ2数据
    MQ2_Update();

    // 定期发送数据给ESP32（每1秒发送一次）
    static uint32_t last_send_tick = 0;
    if (HAL_GetTick() - last_send_tick >= 1000) {
        // 发送PPM数据到ESP32
        char data_msg[64];
        int len = sprintf(data_msg, "{DATA:MQ2:%.1f,%d}\r\n", mq2_data.ppm, mq2_data.alarm);
        HAL_UART_Transmit(&huart3, (uint8_t*)data_msg, len, 100);

        printf("[MQ2] PPM=%.1f, 报警等级=%d\r\n", mq2_data.ppm, mq2_data.alarm);
        last_send_tick = HAL_GetTick();
    }

    // 边沿触发检测烟雾事件（只在刚检测到时发送一次）
    static uint8_t last_alarm_state = MQ2_ALARM_SAFE;

    if (mq2_data.alarm >= MQ2_ALARM_LOW && last_alarm_state == MQ2_ALARM_SAFE) {
        // 从安全状态变为报警状态，发送烟雾事件
        ESP32_COMM_SendSmokeEvent();
        printf("[MQ2] ⚠️ 烟雾报警！PPM=%.1f, 等级=%d\r\n", mq2_data.ppm, mq2_data.alarm);
    }

    last_alarm_state = (mq2_data.alarm >= MQ2_ALARM_LOW) ? mq2_data.alarm : MQ2_ALARM_SAFE;
}
