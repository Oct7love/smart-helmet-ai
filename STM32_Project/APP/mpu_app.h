/**
  ******************************************************************************
  * @file           : mpu_app.h
  * @brief          : MPU6050六轴传感器驱动库（DMP版本）
  * @author         : AI Assistant
  * @date           : 2025/11/11
  * @version        : v2.0 (DMP + 计步器 + 摔倒检测)
  ******************************************************************************
  * @attention
  * 
  * 功能特点:
  * - 基于硬件I2C通信 + DMP固件
  * - 支持四元数姿态解算（Pitch/Roll/Yaw）
  * - 计步器功能（步数统计、距离计算）
  * - 摔倒检测功能（基于角度和加速度）
  * - 碰撞检测功能（基于SVM加速度总值）
  * - 代码结构清晰，可读性强
  *
  * 硬件连接:
  * - SCL: PB6 (I2C1_SCL)
  * - SDA: PB7 (I2C1_SDA)
  * - VCC: 3.3V
  * - GND: GND
  * - AD0: GND (I2C地址=0xD0)
  *
  ******************************************************************************
  */

#ifndef STM32_CLASS_PROJECT_MPU_APP_H
#define STM32_CLASS_PROJECT_MPU_APP_H

/* 头文件包含 ----------------------------------------------------------------*/
#include "bsp_system.h"
#include "i2c.h"
#include <math.h>

/* DMP 库头文件 */
#include "eMPL_MPU/inv_mpu.h"
#include "eMPL_MPU/inv_mpu_dmp_motion_driver.h"

/* MPU6050寄存器地址 ---------------------------------------------------------*/
#define MPU6050_ADDR                0xD0            // MPU6050的I2C地址(AD0=0时)
#define MPU6050_WHO_AM_I            0x75            // 设备ID寄存器
#define MPU6050_ACCEL_XOUT_H        0x3B            // 加速度X轴高字节
#define MPU6050_TEMP_OUT_H          0x41            // 温度高字节
#define MPU6050_GYRO_XOUT_H         0x43            // 陀螺仪X轴高字节

/* 硬件配置 ------------------------------------------------------------------*/
#define MPU6050_I2C_HANDLE          hi2c1           // I2C句柄
#define MPU6050_I2C_TIMEOUT         100             // I2C超时时间(ms)

/* 计步器配置 ----------------------------------------------------------------*/
#define MPU6050_STEP_LENGTH         0.3f            // 步距（米）默认30cm
#define MPU6050_SOFT_STEP_THRESHOLD 0.15f           // 软件计步阈值（g）降低阈值提高灵敏度
#define MPU6050_STEP_MIN_INTERVAL   250             // 最小步伐间隔(ms)防止重复计数

/* 摔倒检测阈值配置 ----------------------------------------------------------*/
#define MPU6050_FALL_ANGLE_THRESHOLD    60.0f       // 倾斜角度阈值（度）
#define MPU6050_COLLISION_SVM_HIGH      30000       // 碰撞检测上限（SVM值）提高阈值
#define MPU6050_COLLISION_SVM_LOW       8000        // 碰撞检测下限（SVM值）降低阈值
#define MPU6050_COLLISION_TIME_WINDOW   10          // 碰撞检测时间窗口（次）

/* 类型定义 ------------------------------------------------------------------*/
typedef enum {
    MPU6050_OK = 0,         // 操作成功
    MPU6050_ERROR,          // 一般错误
    MPU6050_TIMEOUT,        // 通信超时
    MPU6050_NOT_FOUND,      // 设备未找到
    MPU6050_DMP_ERROR       // DMP错误
} MPU6050_Status_t;

typedef struct {
    // DMP 姿态角（欧拉角，单位：度）
    float       pitch;          // 俯仰角 -90° ~ +90°
    float       roll;           // 横滚角 -180° ~ +180°
    float       yaw;            // 航向角 -180° ~ +180°
    
    // 原始传感器数据
    short       accel_x_raw;    // 加速度X轴原始值
    short       accel_y_raw;    // 加速度Y轴原始值
    short       accel_z_raw;    // 加速度Z轴原始值
    
    short       gyro_x_raw;     // 陀螺仪X轴原始值
    short       gyro_y_raw;     // 陀螺仪Y轴原始值
    short       gyro_z_raw;     // 陀螺仪Z轴原始值
    
    short       temp_raw;       // 温度原始值
    
    // 转换后的物理数据
    float       accel_x_g;      // 加速度X轴(g)
    float       accel_y_g;      // 加速度Y轴(g)
    float       accel_z_g;      // 加速度Z轴(g)
    
    float       gyro_x_dps;     // 角速度X轴(°/s)
    float       gyro_y_dps;     // 角速度Y轴(°/s)
    float       gyro_z_dps;     // 角速度Z轴(°/s)
    
    float       temperature;    // 温度(°C)
    
    // 计步器数据
    uint32_t    step_count;     // 步数
    float       distance;       // 行走距离（米）
    float       step_length;    // 步距（米）
    
    // 摔倒检测数据
    uint8_t     fall_flag;      // 摔倒标志（1=检测到摔倒）
    uint8_t     collision_flag; // 碰撞标志（1=检测到碰撞）
    int         svm;            // 加速度矢量和（Signal Vector Magnitude）
    uint8_t     collision_counter; // 碰撞计数器
    
    // 状态标志
    uint8_t     dmp_enabled;    // DMP使能标志
    uint8_t     is_valid;       // 数据有效标志
} MPU6050_Data_t;

/* 公共变量 ------------------------------------------------------------------*/
extern MPU6050_Data_t mpu6050_data;

/* 函数声明 ------------------------------------------------------------------*/
// 基础功能
MPU6050_Status_t MPU6050_Init(void);
MPU6050_Status_t MPU6050_DMP_Init(void);
MPU6050_Status_t MPU6050_Update(void);
void MPU6050_PrintInfo(void);
void mpu6050_task(void); // 任务函数（供调度器调用）

// 数据获取函数
float MPU6050_GetTemperature(void);
float MPU6050_GetPitch(void);
float MPU6050_GetRoll(void);
float MPU6050_GetYaw(void);

// 计步器功能
uint32_t MPU6050_GetStepCount(void);
float MPU6050_GetDistance(void);
void MPU6050_SetStepLength(float length);
void MPU6050_ResetStepCount(void);

// 摔倒检测功能
uint8_t MPU6050_IsFallDetected(void);
uint8_t MPU6050_IsCollisionDetected(void);
void MPU6050_UpdateFallDetection(void);
int MPU6050_GetSVM(void);

// 原始数据读取（兼容旧版本）
MPU6050_Status_t MPU6050_ReadAccel(short *ax, short *ay, short *az);
MPU6050_Status_t MPU6050_ReadGyro(short *gx, short *gy, short *gz);
MPU6050_Status_t MPU6050_ReadTemp(short *temp);

#endif /* STM32_CLASS_PROJECT_MPU_APP_H */
