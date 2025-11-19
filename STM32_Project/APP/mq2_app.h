/**
  ******************************************************************************
  * @file           : mq2_app.h
  * @brief          : MQ-2气体传感器驱动库
  * @author         : 陈旭东
  * @date           : 2025/11/2
  * @version        : v2.0
  ******************************************************************************
  * @attention
  * 
  * 功能特点:
  * - 基于DMA的ADC采样与均值滤波
  * - 宏定义配置参数，方便移植
  * - 支持多种气体类型 (LPG, CO, 烟雾)
  * - 报警等级检测
  * - 代码结构清晰，可读性强
  * 
  ******************************************************************************
  */

#ifndef STM32_CLASS_PROJECT_MQ2_APP_H
#define STM32_CLASS_PROJECT_MQ2_APP_H

/* 头文件包含 ----------------------------------------------------------------*/
#include "bsp_system.h"
#include <math.h>

/* 硬件配置 ------------------------------------------------------------------*/
#define MQ2_ADC_HANDLE          hadc1           // ADC句柄
#define MQ2_SAMPLE_COUNT        30              // 采样次数（用于均值滤波）

/* 电路参数 ------------------------------------------------------------------*/
#define MQ2_VCC                 5.0f            // 传感器供电电压 (V)
#define MQ2_VREF                3.3f            // ADC参考电压 (V)
#define MQ2_ADC_RESOLUTION      4095.0f         // ADC分辨率 (12位)
#define MQ2_RL_VALUE            4700.0f         // 负载电阻RL (欧姆)
#define MQ2_R0_VALUE            35904.0f        // 清洁空气中传感器电阻R0 (欧姆)

/* 气体检测参数 --------------------------------------------------------------*/
// LPG (液化石油气) - 默认
#define MQ2_GAS_A               11.5428f        // 曲线系数A
#define MQ2_GAS_B               -1.5278f        // 曲线系数B

/* 可选气体类型 (取消注释以使用) ---------------------------------------------*/
// CO (一氧化碳)
// #define MQ2_GAS_A            574.25f
// #define MQ2_GAS_B            -2.222f

// 烟雾
// #define MQ2_GAS_A            659.35f
// #define MQ2_GAS_B            -2.168f

/* 报警阈值 ------------------------------------------------------------------*/
#define MQ2_ALARM_LEVEL_SAFE    50.0f           // 安全等级 (ppm)
#define MQ2_ALARM_LEVEL_LOW     200.0f          // 低级报警 (ppm)
#define MQ2_ALARM_LEVEL_MID     500.0f          // 中级报警 (ppm)
#define MQ2_ALARM_LEVEL_HIGH    1000.0f         // 高级报警 (ppm)

/* 计算宏定义 ----------------------------------------------------------------*/
// ADC原始值转换为电压值 (V = ADC * Vref / 分辨率)
#define MQ2_ADC_TO_VOLTAGE(adc)     ((float)(adc) * (MQ2_VREF / MQ2_ADC_RESOLUTION))

// 电压值转换为传感器电阻Rs (Rs = RL * (Vcc - V) / V)
#define MQ2_VOLTAGE_TO_RS(v)        ((MQ2_RL_VALUE * (MQ2_VCC - (v))) / (v))

// 计算Rs/R0比值 (气体浓度特性曲线的横坐标)
#define MQ2_RS_TO_RATIO(rs)         ((float)(rs) / MQ2_R0_VALUE)

// Rs/R0比值转换为PPM浓度 (PPM = (ratio/A)^(1/B), 对数坐标反推公式)
#define MQ2_RATIO_TO_PPM(ratio)     (powf((ratio) / MQ2_GAS_A, 1.0f / MQ2_GAS_B))

/* 数据验证宏 ----------------------------------------------------------------*/
// 电压有效性检查 (必须在0.01V到Vcc-0.01V之间，避免除零和饱和)
#define MQ2_IS_VOLTAGE_VALID(v)     ((v) > 0.01f && (v) < (MQ2_VCC - 0.01f))

// PPM值有效性检查 (范围0-10000ppm)
#define MQ2_IS_PPM_VALID(ppm)       ((ppm) >= 0.0f && (ppm) < 10000.0f)

/* 类型定义 ------------------------------------------------------------------*/
typedef enum {
    MQ2_ALARM_SAFE = 0,     // 安全 - 无报警
    MQ2_ALARM_LOW,          // 低级报警
    MQ2_ALARM_MID,          // 中级报警
    MQ2_ALARM_HIGH,         // 高级报警
    MQ2_ALARM_DANGER        // 危险 - 立即撤离
} MQ2_AlarmLevel_t;

typedef struct {
    uint32_t    adc_raw;        // ADC原始值
    float       adc_voltage;    // ADC电压 (0-3.3V)
    float       sensor_voltage; // 传感器输出电压 (0-5V)
    float       rs;             // 传感器电阻 (欧姆)
    float       ratio;          // Rs/R0比值
    float       ppm;            // 气体浓度 (ppm)
    MQ2_AlarmLevel_t alarm;     // 报警等级
} MQ2_Data_t;

/* 公共变量 ------------------------------------------------------------------*/
extern MQ2_Data_t mq2_data;

/* 函数声明 ------------------------------------------------------------------*/
void MQ2_Init(void);
void MQ2_Update(void);
float MQ2_GetPPM(void);
MQ2_AlarmLevel_t MQ2_GetAlarmLevel(void);
void MQ2_PrintInfo(void);
void mq2_task(void); // 兼容旧版本
/* 内联辅助函数 --------------------------------------------------------------*/
/**
  * @brief  获取当前报警等级
  * @retval 报警等级枚举
  */
static inline MQ2_AlarmLevel_t MQ2_CheckAlarm(float ppm)
{
    if (ppm < MQ2_ALARM_LEVEL_SAFE)      return MQ2_ALARM_SAFE;
    if (ppm < MQ2_ALARM_LEVEL_LOW)       return MQ2_ALARM_LOW;
    if (ppm < MQ2_ALARM_LEVEL_MID)       return MQ2_ALARM_MID;
    if (ppm < MQ2_ALARM_LEVEL_HIGH)      return MQ2_ALARM_HIGH;
    return MQ2_ALARM_DANGER;
}

#endif /* STM32_CLASS_PROJECT_MQ2_APP_H */