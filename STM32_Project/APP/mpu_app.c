/**
  ******************************************************************************
  * @file           : mpu_app.c
  * @brief          : MPU6050六轴传感器驱动实现（DMP版本）
  * @author         : AI Assistant
  * @date           : 2025/11/11
  * @version        : v2.0 (DMP + 计步器 + 摔倒检测)
  ******************************************************************************
  */

/* 头文件包含 ----------------------------------------------------------------*/
#include "mpu_app.h"
#include "esp32_comm.h"  // ESP32通信模块
#include "i2c.h"
#include <stdio.h>

/* 公共变量 ------------------------------------------------------------------*/
MPU6050_Data_t mpu6050_data = {0};  // 全局传感器数据结构

/* 私有函数声明 --------------------------------------------------------------*/
static void MPU6050_CalculatePhysicalValues(void);

/**
  * @brief  初始化MPU6050传感器（基础版本，不使用DMP）
  * @note   如需使用DMP，请调用 MPU6050_DMP_Init()
  * @retval MPU6050状态
  */
MPU6050_Status_t MPU6050_Init(void)
{
    HAL_Delay(100);
    
    // 复位数据结构
    memset(&mpu6050_data, 0, sizeof(MPU6050_Data_t));
    mpu6050_data.step_length = MPU6050_STEP_LENGTH;
    mpu6050_data.collision_counter = MPU6050_COLLISION_TIME_WINDOW;
    mpu6050_data.dmp_enabled = 0;
    
    printf("[MPU6050] Basic Init Success!\r\n");
    return MPU6050_OK;
}

/**
  * @brief  初始化MPU6050 DMP功能
  * @note   必须在系统初始化时调用
  * @retval MPU6050状态
  */
MPU6050_Status_t MPU6050_DMP_Init(void)
{
    uint8_t res;

    // 等待MPU6050上电稳定（关键！）
    printf("[MPU6050] 等待上电稳定...\r\n");
    HAL_Delay(200);

    // 调用DMP初始化函数
    res = mpu_dmp_init();

    if (res == 0)
    {
        mpu6050_data.dmp_enabled = 1;
        mpu6050_data.is_valid = 1;
        printf("[MPU6050] DMP Init Success!\r\n");
        return MPU6050_OK;
    }
    else
    {
        mpu6050_data.dmp_enabled = 0;
        mpu6050_data.is_valid = 0;
        printf("[MPU6050] DMP Init Failed! Error Code: %d\r\n", res);

        // 错误代码说明
        switch(res)
        {
            case 1:  printf("  - Error: Set sensors failed\r\n"); break;
            case 2:  printf("  - Error: Configure FIFO failed\r\n"); break;
            case 3:  printf("  - Error: Set sample rate failed\r\n"); break;
            case 4:  printf("  - Error: Load DMP firmware failed\r\n"); break;
            case 5:  printf("  - Error: Set orientation failed\r\n"); break;
            case 6:  printf("  - Error: Enable features failed\r\n"); break;
            case 7:  printf("  - Error: Set FIFO rate failed\r\n"); break;
            case 8:  printf("  - Error: Self test failed\r\n"); break;
            case 10: printf("  - Error: MPU6050 not found\r\n"); break;
            default: printf("  - Error: Unknown error\r\n"); break;
        }

        // 如果是设备未找到，扫描I2C总线
        if (res == 10)
        {
            printf("\r\n[I2C扫描] 正在扫描I2C1总线(PB6/PB7)...\r\n");
            printf("地址范围: 0x00-0xFE (7位地址)\r\n");

            uint8_t found_count = 0;
            for (uint16_t addr = 0x00; addr <= 0xFE; addr += 2)
            {
                HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c1, addr, 1, 10);
                if (result == HAL_OK)
                {
                    printf("  ✓ 找到设备: 0x%02X (7位: 0x%02X)\r\n", addr, addr >> 1);
                    found_count++;
                }
            }

            if (found_count == 0)
            {
                printf("  ✗ 未找到任何I2C设备！\r\n");
                printf("\r\n请检查:\r\n");
                printf("  1. MPU6050电源是否正常(VCC=3.3V, GND连接)\r\n");
                printf("  2. SCL和SDA是否正确连接到PB6和PB7\r\n");
                printf("  3. I2C总线是否有上拉电阻(模块通常自带)\r\n");
                printf("  4. 线路是否松动或接触不良\r\n");
            }
            else
            {
                printf("\r\n提示: MPU6050默认地址应为0xD0 (AD0=GND) 或 0xD2 (AD0=VCC)\r\n");
                printf("     如果找到其他地址，请检查AD0引脚连接\r\n");
            }
        }

        return MPU6050_DMP_ERROR;
    }
}

/**
  * @brief  更新传感器读数（周期性调用）
  * @note   主循环或调度器中调用此函数
  * @retval MPU6050状态
  */
MPU6050_Status_t MPU6050_Update(void)
{
    uint8_t res;
    
    if (!mpu6050_data.dmp_enabled)
    {
        return MPU6050_ERROR;
    }
    
    // 读取DMP姿态数据
    static float last_pitch = 0, last_roll = 0, last_yaw = 0;
    static uint8_t first_read = 1;
    float new_pitch, new_roll, new_yaw;
    
    res = mpu_dmp_get_data(&new_pitch, &new_roll, &new_yaw);
    
    // 数据有效性检查：只过滤明显错误的数据（如全是NaN或超出物理范围）
    if (!first_read)
    {
        // 只过滤超过180度突变（物理上不可能瞬间旋转超过180度）
        if (fabsf(new_pitch - last_pitch) > 180.0f ||
            fabsf(new_roll - last_roll) > 180.0f ||
            fabsf(new_yaw - last_yaw) > 180.0f)
        {
            // 极端异常数据，保持上次值
            mpu6050_data.pitch = last_pitch;
            mpu6050_data.roll = last_roll;
            mpu6050_data.yaw = last_yaw;
            return MPU6050_ERROR;
        }
    }
    
    // 正常更新
    mpu6050_data.pitch = new_pitch;
    mpu6050_data.roll = new_roll;
    mpu6050_data.yaw = new_yaw;
    last_pitch = new_pitch;
    last_roll = new_roll;
    last_yaw = new_yaw;
    first_read = 0;
    
    if (res == 0)
    {
        mpu6050_data.is_valid = 1;
        
        // 读取原始加速度数据（用于摔倒检测）
        MPU6050_ReadAccel(&mpu6050_data.accel_x_raw,
                         &mpu6050_data.accel_y_raw,
                         &mpu6050_data.accel_z_raw);
        
        // 读取温度数据
        int16_t temp_raw;
        MPU6050_ReadTemp(&temp_raw);
        
        // 计算物理值
        MPU6050_CalculatePhysicalValues();
        
        // 更新摔倒检测
        MPU6050_UpdateFallDetection();
        
        // 更新计步器数据（直接读取，避免重复调用）
        unsigned long steps = 0;
        dmp_get_pedometer_step_count(&steps);
        
        // 软件辅助计步：基于加速度变化检测步伐
        static float last_accel_magnitude = 1.0f;
        static uint32_t last_step_time = 0;
        static uint32_t soft_step_count = 0;
        
        float current_magnitude = sqrtf(
            mpu6050_data.accel_x_g * mpu6050_data.accel_x_g +
            mpu6050_data.accel_y_g * mpu6050_data.accel_y_g +
            mpu6050_data.accel_z_g * mpu6050_data.accel_z_g
        );
        
        // 检测加速度波峰（走路时的特征）
        float accel_change = fabsf(current_magnitude - last_accel_magnitude);
        uint32_t current_time = HAL_GetTick();
        
        if (accel_change > MPU6050_SOFT_STEP_THRESHOLD &&  // 超过阈值
            (current_time - last_step_time) > MPU6050_STEP_MIN_INTERVAL)  // 防抖
        {
            soft_step_count++;
            last_step_time = current_time;
        }
        
        last_accel_magnitude = current_magnitude;
        
        // 使用软件计步器（更灵敏）或DMP计步器（取较大值）
        mpu6050_data.step_count = (soft_step_count > steps) ? soft_step_count : steps;
        mpu6050_data.distance = mpu6050_data.step_count * mpu6050_data.step_length;
        
        return MPU6050_OK;
    }
    else
    {
        mpu6050_data.is_valid = 0;
        return MPU6050_ERROR;
    }
}

/**
  * @brief  打印传感器信息
  * @retval 无
  */
void MPU6050_PrintInfo(void)
{
    if (!mpu6050_data.is_valid)
    {
        printf("[MPU6050] Data Invalid!\r\n");
        return;
    }
    
    printf("=== MPU6050 DMP Data ===\r\n");
    printf("Pitch: %.1f°, Roll: %.1f°, Yaw: %.1f°\r\n", 
           mpu6050_data.pitch, 
           mpu6050_data.roll, 
           mpu6050_data.yaw);
    
    printf("Accel: X=%d, Y=%d, Z=%d (raw)\r\n",
           mpu6050_data.accel_x_raw,
           mpu6050_data.accel_y_raw,
           mpu6050_data.accel_z_raw);
    
    printf("SVM: %d\r\n", mpu6050_data.svm);
    
    printf("Steps: %lu, Distance: %.2fm\r\n",
           mpu6050_data.step_count,
           mpu6050_data.distance);
    
    if (mpu6050_data.fall_flag)
        printf("** FALL DETECTED **\r\n");
    
    if (mpu6050_data.collision_flag)
        printf("** COLLISION DETECTED **\r\n");
    
    printf("========================\r\n");
}

/**
  * @brief  任务函数（供调度器调用）
  * @retval 无
  */
void mpu6050_task(void)
{
    static uint8_t is_initialized = 0;
    static uint32_t last_print_tick = 0;
    static uint32_t last_steps = 0;
    static uint32_t heartbeat_tick = 0;

    // 调试开关：设置为0可禁用所有printf，提高性能
    #define MPU6050_DEBUG_PRINT 0

    // 首次运行时初始化计步器参数（仅当DMP已启用时）
    if (!is_initialized && mpu6050_data.dmp_enabled)
    {
        MPU6050_SetStepLength(0.35f);  // 设置步距35cm（根据身高调整）
        MPU6050_ResetStepCount();      // 重置计步器
        #if MPU6050_DEBUG_PRINT
        printf("计步器初始化完成: 步距=0.35m\r\n");
        #endif
        is_initialized = 1;
    }
    
    // 如果DMP未启用，硬件未连接
    if (!mpu6050_data.dmp_enabled)
    {
        extern volatile uint8_t mpu6050_enabled;

        // 每5秒打印一次状态（仅在调试模式）
        #if MPU6050_DEBUG_PRINT
        static uint32_t last_warn_tick = 0;
        if ((HAL_GetTick() - last_warn_tick) >= 5000)
        {
            printf("[MPU6050_TASK] ⚠️ DMP初始化失败，请检查硬件连接！\r\n");
            printf("  - 检查SCL(PB6)和SDA(PB7)连接\r\n");
            printf("  - 检查VCC(3.3V)和GND连接\r\n");
            printf("  - 检查AD0引脚是否接GND (I2C地址应为0xD0)\r\n");
            last_warn_tick = HAL_GetTick();
        }
        #endif

        return;
    }
    
    // 更新 MPU6050 数据（每100ms调用一次）
    MPU6050_Status_t update_status = MPU6050_Update();

    if (update_status == MPU6050_OK)
    {
        // 检查ESP32是否启用姿态检测
        extern volatile uint8_t mpu6050_enabled;

        // 每1秒发送数据给ESP32和打印数据（只在ESP32启用时）
        if (mpu6050_enabled && (HAL_GetTick() - last_print_tick) >= 1000)
        {
            // 发送数据到ESP32
            ESP32_COMM_SendMPU6050(mpu6050_data.pitch,
                                   mpu6050_data.roll,
                                   mpu6050_data.yaw);

            // 打印调试信息
            printf("\r\n--- MPU6050 数据 ---\r\n");
            printf("温度: %.1f°C\r\n", mpu6050_data.temperature);
            printf("姿态: Pitch=%.1f° Roll=%.1f° Yaw=%.1f°\r\n",
                   mpu6050_data.pitch, mpu6050_data.roll, mpu6050_data.yaw);
            printf("加速度: X=%.3fg Y=%.3fg Z=%.3fg (SVM=%d)\r\n",
                   mpu6050_data.accel_x_g,
                   mpu6050_data.accel_y_g,
                   mpu6050_data.accel_z_g,
                   mpu6050_data.svm);
            printf("计步器: 步数=%lu  距离=%.2fm (%.3fkm)\r\n",
                   mpu6050_data.step_count,
                   mpu6050_data.distance,
                   mpu6050_data.distance / 1000.0f);

            // 检测步数变化
            if (mpu6050_data.step_count != last_steps)
            {
                printf("✓ 新增 %lu 步！\r\n",
                       mpu6050_data.step_count - last_steps);
                last_steps = mpu6050_data.step_count;
            }

            printf("-------------------\r\n");
            last_print_tick = HAL_GetTick();
        }

        // 摔倒/碰撞检测（始终检测，但只在启用时发送和打印）
        static uint8_t last_fall_flag = 0;
        static uint8_t last_collision_flag = 0;

        // 摔倒事件检测（边沿触发，只在第一次检测到时发送）
        if (mpu6050_data.fall_flag && !last_fall_flag)
        {
            // 只有ESP32启用时才发送和打印
            if (mpu6050_enabled) {
                printf("⚠️ 摔倒检测！\r\n");
                ESP32_COMM_SendFallEvent();
            }
        }
        last_fall_flag = mpu6050_data.fall_flag;

        // 碰撞事件检测（边沿触发，只在第一次检测到时发送）
        if (mpu6050_data.collision_flag && !last_collision_flag)
        {
            // 只有ESP32启用时才发送和打印
            if (mpu6050_enabled) {
                printf("⚠️ 碰撞检测！\r\n");
                ESP32_COMM_SendCollisionEvent();
            }
        }
        last_collision_flag = mpu6050_data.collision_flag;
    }
}

/**
  * @brief  获取温度
  * @retval 温度值（°C）
  */
float MPU6050_GetTemperature(void)
{
    return mpu6050_data.temperature;
}

/**
  * @brief  获取俯仰角
  * @retval 俯仰角（°）
  */
float MPU6050_GetPitch(void)
{
    return mpu6050_data.pitch;
}

/**
  * @brief  获取横滚角
  * @retval 横滚角（°）
  */
float MPU6050_GetRoll(void)
{
    return mpu6050_data.roll;
}

/**
  * @brief  获取航向角
  * @retval 航向角（°）
  */
float MPU6050_GetYaw(void)
{
    return mpu6050_data.yaw;
}

/**
  * @brief  获取步数
  * @retval 步数
  */
uint32_t MPU6050_GetStepCount(void)
{
    unsigned long steps = 0;
    dmp_get_pedometer_step_count(&steps);
    mpu6050_data.step_count = steps;
    mpu6050_data.distance = mpu6050_data.step_count * mpu6050_data.step_length;
    return mpu6050_data.step_count;
}

/**
  * @brief  获取行走距离
  * @retval 距离（米）
  */
float MPU6050_GetDistance(void)
{
    MPU6050_GetStepCount(); // 更新步数和距离
    return mpu6050_data.distance;
}

/**
  * @brief  设置步距
  * @param  length: 步距（米）
  * @retval 无
  */
void MPU6050_SetStepLength(float length)
{
    mpu6050_data.step_length = length;
}

/**
  * @brief  重置步数计数器
  * @retval 无
  */
void MPU6050_ResetStepCount(void)
{
    dmp_set_pedometer_step_count(0);
    mpu6050_data.step_count = 0;
    mpu6050_data.distance = 0.0f;
}

/**
  * @brief  检测是否摔倒
  * @retval 1=检测到摔倒, 0=正常
  */
uint8_t MPU6050_IsFallDetected(void)
{
    return mpu6050_data.fall_flag;
}

/**
  * @brief  检测是否碰撞
  * @retval 1=检测到碰撞, 0=正常
  */
uint8_t MPU6050_IsCollisionDetected(void)
{
    return mpu6050_data.collision_flag;
}

/**
  * @brief  获取加速度矢量和（SVM）
  * @retval SVM值
  */
int MPU6050_GetSVM(void)
{
    return mpu6050_data.svm;
}

/**
  * @brief  更新摔倒检测逻辑
  * @note   内部函数，由MPU6050_Update调用
  * @retval 无
  */
void MPU6050_UpdateFallDetection(void)
{
    // 计算SVM（加速度矢量和）
    mpu6050_data.svm = sqrt(pow(mpu6050_data.accel_x_raw, 2) + 
                            pow(mpu6050_data.accel_y_raw, 2) + 
                            pow(mpu6050_data.accel_z_raw, 2));
    
    // 1. 角度检测：检测倾斜角度是否超过阈值
    if (fabs(mpu6050_data.pitch) > MPU6050_FALL_ANGLE_THRESHOLD || 
        fabs(mpu6050_data.roll) > MPU6050_FALL_ANGLE_THRESHOLD)
    {
        mpu6050_data.fall_flag = 1;
    }
    else
    {
        mpu6050_data.fall_flag = 0;
    }
    
    // 2. 碰撞检测：检测加速度突变
    if (mpu6050_data.svm > MPU6050_COLLISION_SVM_HIGH || 
        mpu6050_data.svm < MPU6050_COLLISION_SVM_LOW)
    {
        mpu6050_data.collision_counter = 0; // 重置计数器
    }
    
    mpu6050_data.collision_counter++;
    
    // 如果在时间窗口内，标记为碰撞
    if (mpu6050_data.collision_counter <= MPU6050_COLLISION_TIME_WINDOW)
    {
        mpu6050_data.collision_flag = 1;
    }
    else
    {
        // 限制计数器上限
        if (mpu6050_data.collision_counter > MPU6050_COLLISION_TIME_WINDOW)
            mpu6050_data.collision_counter = MPU6050_COLLISION_TIME_WINDOW;
        
        mpu6050_data.collision_flag = 0;
    }
}

/**
  * @brief  读取加速度原始数据
  * @param  ax, ay, az: 加速度xyz轴数据指针
  * @retval MPU6050状态
  */
MPU6050_Status_t MPU6050_ReadAccel(short *ax, short *ay, short *az)
{
    uint8_t buf[6];
    HAL_StatusTypeDef hal_status;
    
    hal_status = HAL_I2C_Mem_Read(&MPU6050_I2C_HANDLE,
                                  MPU6050_ADDR,
                                  MPU6050_ACCEL_XOUT_H,
                                  I2C_MEMADD_SIZE_8BIT,
                                  buf,
                                  6,
                                  MPU6050_I2C_TIMEOUT);
    
    if (hal_status == HAL_OK)
    {
        *ax = (int16_t)((buf[0] << 8) | buf[1]);
        *ay = (int16_t)((buf[2] << 8) | buf[3]);
        *az = (int16_t)((buf[4] << 8) | buf[5]);
        return MPU6050_OK;
    }
    
    return MPU6050_ERROR;
}

/**
  * @brief  读取陀螺仪原始数据
  * @param  gx, gy, gz: 陀螺仪xyz轴数据指针
  * @retval MPU6050状态
  */
MPU6050_Status_t MPU6050_ReadGyro(short *gx, short *gy, short *gz)
{
    uint8_t buf[6];
    HAL_StatusTypeDef hal_status;
    
    hal_status = HAL_I2C_Mem_Read(&MPU6050_I2C_HANDLE,
                                  MPU6050_ADDR,
                                  MPU6050_GYRO_XOUT_H,
                                  I2C_MEMADD_SIZE_8BIT,
                                  buf,
                                  6,
                                  MPU6050_I2C_TIMEOUT);
    
    if (hal_status == HAL_OK)
    {
        *gx = (int16_t)((buf[0] << 8) | buf[1]);
        *gy = (int16_t)((buf[2] << 8) | buf[3]);
        *gz = (int16_t)((buf[4] << 8) | buf[5]);
        return MPU6050_OK;
    }
    
    return MPU6050_ERROR;
}

/**
  * @brief  读取温度原始数据
  * @param  temp: 温度数据指针
  * @retval MPU6050状态
  */
MPU6050_Status_t MPU6050_ReadTemp(short *temp)
{
    uint8_t buf[2];
    HAL_StatusTypeDef hal_status;
    
    hal_status = HAL_I2C_Mem_Read(&MPU6050_I2C_HANDLE,
                                  MPU6050_ADDR,
                                  MPU6050_TEMP_OUT_H,
                                  I2C_MEMADD_SIZE_8BIT,
                                  buf,
                                  2,
                                  MPU6050_I2C_TIMEOUT);
    
    if (hal_status == HAL_OK)
    {
        *temp = (int16_t)((buf[0] << 8) | buf[1]);
        mpu6050_data.temperature = 36.53f + ((float)*temp) / 340.0f;
        return MPU6050_OK;
    }
    
    return MPU6050_ERROR;
}

/**
  * @brief  计算物理值（内部函数）
  * @retval 无
  */
static void MPU6050_CalculatePhysicalValues(void)
{
    // 加速度转换（假设量程±2g，灵敏度16384 LSB/g）
    mpu6050_data.accel_x_g = (float)mpu6050_data.accel_x_raw / 16384.0f;
    mpu6050_data.accel_y_g = (float)mpu6050_data.accel_y_raw / 16384.0f;
    mpu6050_data.accel_z_g = (float)mpu6050_data.accel_z_raw / 16384.0f;
    
    // 陀螺仪转换（假设量程±250°/s，灵敏度131 LSB/(°/s)）
    mpu6050_data.gyro_x_dps = (float)mpu6050_data.gyro_x_raw / 131.0f;
    mpu6050_data.gyro_y_dps = (float)mpu6050_data.gyro_y_raw / 131.0f;
    mpu6050_data.gyro_z_dps = (float)mpu6050_data.gyro_z_raw / 131.0f;
}
