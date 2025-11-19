/**
  ******************************************************************************
  * @file           : max30102_app.h
  * @brief          : MAX30102心率血氧传感器驱动头文件
  * @author         : 13615
  * @date           : 2025/11/11
  * @version        : v1.0
  ******************************************************************************
  */

#ifndef __MAX30102_APP_H
#define __MAX30102_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* 头文件包含 ----------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include <stdbool.h>

/* 硬件配置宏 ----------------------------------------------------------------*/
#define MAX30102_I2C_HANDLE        hi2c2              // I2C句柄(使用I2C2)
#define MAX30102_I2C_ADDR          (0xAE)             // 器件地址（写地址）
#define MAX30102_I2C_TIMEOUT       100                // I2C超时时间(ms)

/* 寄存器地址 ----------------------------------------------------------------*/
#define MAX30102_REG_INT_STATUS1   0x00               // 中断状态1
#define MAX30102_REG_INT_STATUS2   0x01               // 中断状态2
#define MAX30102_REG_INT_ENABLE1   0x02               // 中断使能1
#define MAX30102_REG_INT_ENABLE2   0x03               // 中断使能2
#define MAX30102_REG_FIFO_WR_PTR   0x04               // FIFO写指针
#define MAX30102_REG_FIFO_OVF_CNT  0x05               // FIFO溢出计数
#define MAX30102_REG_FIFO_RD_PTR   0x06               // FIFO读指针
#define MAX30102_REG_FIFO_DATA     0x07               // FIFO数据寄存器
#define MAX30102_REG_MODE_CONFIG   0x09               // 模式配置
#define MAX30102_REG_SPO2_CONFIG   0x0A               // SpO2配置
#define MAX30102_REG_LED1_PA       0x0C               // LED1脉冲幅度(红光)
#define MAX30102_REG_LED2_PA       0x0D               // LED2脉冲幅度(红外)
#define MAX30102_REG_TEMP_INT      0x1F               // 温度整数部分
#define MAX30102_REG_TEMP_FRAC     0x20               // 温度小数部分
#define MAX30102_REG_REV_ID        0xFE               // 版本ID
#define MAX30102_REG_PART_ID       0xFF               // 器件ID

/* 配置参数宏 ----------------------------------------------------------------*/
#define MAX30102_SAMPLE_AVG        4                  // 采样平均次数(1/2/4/8/16/32)
#define MAX30102_SAMPLE_RATE       100                // 采样率(50/100/200/400/800/1000/1600/3200 Hz)
#define MAX30102_LED_PW            411                // LED脉冲宽度(69/118/215/411 us)
#define MAX30102_LED_CURRENT       0x1F               // LED电流(0x00-0xFF, 0.2mA/步)
#define MAX30102_ADC_RANGE         4096               // ADC量程(2048/4096/8192/16384)

/* FIFO配置 ------------------------------------------------------------------*/
#define MAX30102_FIFO_DEPTH        32                 // FIFO深度
#define MAX30102_BUFFER_SIZE       100                // 数据缓冲区大小

/* 算法参数宏 ----------------------------------------------------------------*/
#define MAX30102_HR_MIN            40.0f              // 最小心率(bpm)
#define MAX30102_HR_MAX            200.0f             // 最大心率(bpm)
#define MAX30102_SPO2_MIN          70.0f              // 最小血氧(%)
#define MAX30102_SPO2_MAX          100.0f             // 最大血氧(%)
#define MAX30102_DC_FILTER_ALPHA   0.95f              // 直流滤波系数
#define MAX30102_MEAN_FILTER_SIZE  3                  // 均值滤波窗口(进一步减小,加快响应)
#define MAX30102_PEAK_MIN_INTERVAL 200                // 峰值最小间隔(250→200ms,更快响应)
#define MAX30102_PEAK_MAX_INTERVAL 3000               // 峰值最大间隔(新增,允许慢心率)
#define MAX30102_PEAK_THRESHOLD    0.12f              // 峰值检测阈值(0.15→0.12,提高灵敏度)
#define MAX30102_HR_SMOOTH_ALPHA   0.7f               // 心率平滑系数(提高到0.7,增强抗波动)
#define MAX30102_SPO2_SMOOTH_ALPHA 0.90f              // 血氧平滑系数(提高到0.90,更平滑)
#define MAX30102_MIN_AC_AMPLITUDE  40.0f              // 最小AC幅度(50→40,降低门槛)
#define MAX30102_STABLE_THRESHOLD  3                  // 稳定判定次数(恢复到3,更可靠)
#define MAX30102_OUTLIER_THRESHOLD 50.0f              // 异常值判定阈值(bpm)
#define MAX30102_MIN_STABLE_COUNT  5                  // 最小稳定次数(高度稳定后才难丢失)

/* 计算宏定义 ----------------------------------------------------------------*/
#define MAX30102_IS_HR_VALID(hr)       ((hr) >= MAX30102_HR_MIN && (hr) <= MAX30102_HR_MAX)           // 心率有效性检查
#define MAX30102_IS_SPO2_VALID(spo2)   ((spo2) >= MAX30102_SPO2_MIN && (spo2) <= MAX30102_SPO2_MAX)  // 血氧有效性检查
#define MAX30102_CALC_RATIO(red, ir)   ((float)(red) / (float)(ir))                                   // 计算R值(红光/红外比值)
#define MAX30102_RATIO_TO_SPO2(ratio)  (104.0f - 17.0f * (ratio))                                     // R值转血氧(校准后公式)

/* 数据结构定义 --------------------------------------------------------------*/
typedef struct {
    uint32_t red;                     // 红光原始数据
    uint32_t ir;                      // 红外原始数据
    float red_dc;                     // 红光直流分量
    float ir_dc;                      // 红外直流分量
    float red_ac;                     // 红光交流分量
    float ir_ac;                      // 红外交流分量
    float ratio;                      // R值(红/红外比值)
    float heart_rate;                 // 心率(bpm)
    float spo2;                       // 血氧饱和度(%)
    bool finger_detected;             // 手指检测标志
    uint8_t signal_quality;           // 信号质量(0-100)
    uint8_t hr_status;                // 心率检测状态(0=初始化,1=检测中,2=稳定,3=丢失)
} MAX30102_Data_t;

/* 外部变量声明 --------------------------------------------------------------*/
extern MAX30102_Data_t max30102_data;

/* 函数声明 ------------------------------------------------------------------*/
bool MAX30102_Init(void);                             // 初始化传感器
void MAX30102_Reset(void);                            // 软复位
bool MAX30102_ReadFIFO(void);                         // 读取FIFO数据
void MAX30102_Update(void);                           // 更新传感器数据(周期调用)
float MAX30102_GetHeartRate(void);                    // 获取心率
float MAX30102_GetSpO2(void);                         // 获取血氧
bool MAX30102_IsFingerDetected(void);                 // 检测手指是否放置
void MAX30102_PrintInfo(void);                        // 打印传感器信息
void MAX30102_PrintDebug(void);                       // 打印调试信息

/* 底层I2C函数 ---------------------------------------------------------------*/
bool MAX30102_WriteReg(uint8_t reg, uint8_t data);    // 写寄存器
bool MAX30102_ReadReg(uint8_t reg, uint8_t *data);    // 读寄存器

/* 兼容旧版本 ----------------------------------------------------------------*/
void max30102_task(void);                             // 旧版任务函数(向后兼容)

#ifdef __cplusplus
}
#endif

#endif /* __MAX30102_APP_H */

