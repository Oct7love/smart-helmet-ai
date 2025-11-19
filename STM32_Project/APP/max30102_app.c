/**
  ******************************************************************************
  * @file           : max30102_app.c
  * @brief          : MAX30102心率血氧传感器驱动实现
  * @author         : 13615
  * @date           : 2025/11/11
  * @version        : v1.0
  ******************************************************************************
  */

/* 头文件包含 ----------------------------------------------------------------*/
#include "max30102_app.h"
#include "esp32_comm.h"  // ESP32通信模块
#include <stdio.h>
#include <math.h>
#include <string.h>

/* 私有变量 ------------------------------------------------------------------*/
static uint32_t red_buffer[MAX30102_BUFFER_SIZE];    // 红光数据缓冲区
static uint32_t ir_buffer[MAX30102_BUFFER_SIZE];     // 红外数据缓冲区
static uint16_t buffer_index = 0;                     // 缓冲区索引
static float red_mean_filter[MAX30102_MEAN_FILTER_SIZE];  // 红光均值滤波缓冲
static float ir_mean_filter[MAX30102_MEAN_FILTER_SIZE];   // 红外均值滤波缓冲
static uint8_t filter_index = 0;                      // 滤波器索引
static float last_peak_value = 0.0f;                 // 上次峰值
static float peak_threshold = 0.0f;                  // 动态峰值阈值
static uint32_t last_peak_time = 0;                  // 上次峰值时间
static float spo2_filtered = 0.0f;                   // 血氧滤波值

/* 公共变量 ------------------------------------------------------------------*/
MAX30102_Data_t max30102_data = {0};                  // 全局传感器数据结构

/* 私有函数声明 --------------------------------------------------------------*/
static void MAX30102_ProcessData(void);               // 处理数据
static void MAX30102_CalculateHR(void);               // 计算心率
static void MAX30102_CalculateSpO2(void);             // 计算血氧
static float MAX30102_MeanFilter(float *buffer, float new_val, uint8_t size);  // 均值滤波

/**
  * @brief  初始化MAX30102传感器
  * @retval true:成功 false:失败
  */
bool MAX30102_Init(void)
{
    uint8_t part_id = 0;
    
    // 复位数据结构
    memset(&max30102_data, 0, sizeof(MAX30102_Data_t));
    buffer_index = 0;
    filter_index = 0;
    
    // 延时等待传感器上电稳定
    HAL_Delay(100);
    
    // 读取器件ID验证
    if (!MAX30102_ReadReg(MAX30102_REG_PART_ID, &part_id)) {
        printf("MAX30102: I2C communication failed!\r\n");
        return false;
    }
    
    if (part_id != 0x15) {  // MAX30102的器件ID应为0x15
        printf("MAX30102: Wrong part ID: 0x%02X\r\n", part_id);
        return false;
    }
    
    // 软复位
    MAX30102_Reset();
    HAL_Delay(50);
    
    // 配置中断(A_FULL使能)
    MAX30102_WriteReg(MAX30102_REG_INT_ENABLE1, 0x80);
    MAX30102_WriteReg(MAX30102_REG_INT_ENABLE2, 0x00);
    
    // FIFO配置(样本平均=4, FIFO回滚使能)
    MAX30102_WriteReg(MAX30102_REG_FIFO_WR_PTR, 0x00);
    MAX30102_WriteReg(MAX30102_REG_FIFO_OVF_CNT, 0x00);
    MAX30102_WriteReg(MAX30102_REG_FIFO_RD_PTR, 0x00);
    
    // 模式配置(SpO2模式: 红光+红外)
    MAX30102_WriteReg(MAX30102_REG_MODE_CONFIG, 0x03);
    
    // SpO2配置(ADC=4096, 采样率=100Hz, LED脉宽=411us)
    MAX30102_WriteReg(MAX30102_REG_SPO2_CONFIG, 0x27);
    
    // LED电流配置(红光和红外都设为0x1F ≈ 6.4mA)
    MAX30102_WriteReg(MAX30102_REG_LED1_PA, MAX30102_LED_CURRENT);
    MAX30102_WriteReg(MAX30102_REG_LED2_PA, MAX30102_LED_CURRENT);
    
    printf("MAX30102: Init success!\r\n");
    return true;
}

/**
  * @brief  软复位传感器
  * @retval 无
  */
void MAX30102_Reset(void)
{
    MAX30102_WriteReg(MAX30102_REG_MODE_CONFIG, 0x40);
}

/**
  * @brief  读取FIFO数据
  * @retval true:成功 false:失败
  */
bool MAX30102_ReadFIFO(void)
{
    uint8_t temp[6];
    
    // 读取6字节数据(红光3字节 + 红外3字节)
    if (HAL_I2C_Mem_Read(&MAX30102_I2C_HANDLE, MAX30102_I2C_ADDR, 
                         MAX30102_REG_FIFO_DATA, I2C_MEMADD_SIZE_8BIT, 
                         temp, 6, MAX30102_I2C_TIMEOUT) != HAL_OK) {
        return false;
    }
    
    // 组合为32位数据(18位有效位)
    max30102_data.red = ((uint32_t)temp[0] << 16) | ((uint32_t)temp[1] << 8) | temp[2];
    max30102_data.ir = ((uint32_t)temp[3] << 16) | ((uint32_t)temp[4] << 8) | temp[5];
    
    // 保留18位有效数据
    max30102_data.red &= 0x3FFFF;
    max30102_data.ir &= 0x3FFFF;
    
    // 存入缓冲区
    red_buffer[buffer_index] = max30102_data.red;
    ir_buffer[buffer_index] = max30102_data.ir;
    buffer_index = (buffer_index + 1) % MAX30102_BUFFER_SIZE;
    
    return true;
}

/**
  * @brief  更新传感器数据(周期性调用)
  * @retval 无
  */
void MAX30102_Update(void)
{
    // 读取FIFO数据
    if (!MAX30102_ReadFIFO()) {
        return;
    }
    
    // 手指检测(红外信号强度检查)
    max30102_data.finger_detected = (max30102_data.ir > 50000);
    
    if (max30102_data.finger_detected) {
        // 处理数据并计算
        MAX30102_ProcessData();
        MAX30102_CalculateHR();
        MAX30102_CalculateSpO2();
    } else {
        // 无手指时复位数据
        max30102_data.heart_rate = 0.0f;
        max30102_data.spo2 = 0.0f;
        max30102_data.signal_quality = 0;
    }
}

/**
  * @brief  数据处理(滤波和提取AC/DC分量)
  * @retval 无
  */
static void MAX30102_ProcessData(void)
{
    // 直流滤波(低通滤波器)
    max30102_data.red_dc = MAX30102_DC_FILTER_ALPHA * max30102_data.red_dc + 
                           (1.0f - MAX30102_DC_FILTER_ALPHA) * max30102_data.red;
    max30102_data.ir_dc = MAX30102_DC_FILTER_ALPHA * max30102_data.ir_dc + 
                          (1.0f - MAX30102_DC_FILTER_ALPHA) * max30102_data.ir;
    
    // 交流分量提取
    max30102_data.red_ac = max30102_data.red - max30102_data.red_dc;
    max30102_data.ir_ac = max30102_data.ir - max30102_data.ir_dc;
    
    // 均值滤波平滑AC信号
    max30102_data.red_ac = MAX30102_MeanFilter(red_mean_filter, max30102_data.red_ac, MAX30102_MEAN_FILTER_SIZE);
    max30102_data.ir_ac = MAX30102_MeanFilter(ir_mean_filter, max30102_data.ir_ac, MAX30102_MEAN_FILTER_SIZE);
}

/**
  * @brief  计算心率(改进的自适应峰值检测算法-增强版)
  * @retval 无
  */
static void MAX30102_CalculateHR(void)
{
    static float beat_avg = 0.0f;
    static float last_ir_ac = 0.0f;
    static float max_peak = -10000.0f;
    static float min_valley = 10000.0f;
    static bool rising = false;
    static uint8_t stable_count = 0;
    static uint8_t init_count = 0;
    uint32_t current_time = HAL_GetTick();

    // 🔥 调试：每2秒打印一次信号值
    static uint32_t last_debug = 0;
    if ((current_time - last_debug) >= 2000) {
        printf("[HR_DEBUG] IR_AC=%.1f, Amplitude=%.1f, Threshold=%.1f, Init=%d, Status=%d\r\n",
               max30102_data.ir_ac,
               max_peak - min_valley,
               min_valley + (max_peak - min_valley) * MAX30102_PEAK_THRESHOLD,
               init_count,
               max30102_data.hr_status);
        last_debug = current_time;
    }

    // 初始化阶段:建立峰谷基线(从20减到10次,加快初始化)
    if (init_count < 10) {
        if (max30102_data.ir_ac > max_peak) max_peak = max30102_data.ir_ac;
        if (max30102_data.ir_ac < min_valley) min_valley = max30102_data.ir_ac;
        init_count++;
        last_ir_ac = max30102_data.ir_ac;
        max30102_data.hr_status = 0;  // 初始化中
        return;
    }
    
    // 更新峰谷值(自适应阈值)
    if (max30102_data.ir_ac > max_peak) {
        max_peak = max30102_data.ir_ac;
    } else {
        max_peak *= 0.98f;  // 加快衰减,更快适应
    }
    
    if (max30102_data.ir_ac < min_valley) {
        min_valley = max30102_data.ir_ac;
    } else {
        min_valley *= 0.98f;
    }
    
    // 计算幅度和动态阈值
    float amplitude = max_peak - min_valley;
    float dynamic_threshold = min_valley + amplitude * MAX30102_PEAK_THRESHOLD;
    
    // 检查AC幅度是否足够(至少要有一定波动)
    // 如果已经检测到心率,降低要求避免丢失
    float min_amplitude = (beat_avg > 0.0f) ? MAX30102_MIN_AC_AMPLITUDE * 0.7f : MAX30102_MIN_AC_AMPLITUDE;
    if (amplitude < min_amplitude) {
        // 🔥 调试：信号太弱
        static uint32_t weak_signal_warn = 0;
        if ((current_time - weak_signal_warn) >= 3000) {
            printf("[HR_DEBUG] ⚠️ 信号太弱! amplitude=%.1f < min=%.1f\r\n", amplitude, min_amplitude);
            weak_signal_warn = current_time;
        }
        last_ir_ac = max30102_data.ir_ac;
        return;  // 信号太弱,跳过检测
    }
    
    // 🔥 调试：状态机追踪
    static uint32_t state_debug_time = 0;
    if ((current_time - state_debug_time) >= 3000) {
        printf("[HR_DEBUG] State: last_ir_ac=%.1f, current=%.1f, rising=%d, threshold=%.1f\r\n",
               last_ir_ac, max30102_data.ir_ac, rising, dynamic_threshold);
        state_debug_time = current_time;
    }

    // 状态机检测峰值
    if (max30102_data.ir_ac > last_ir_ac) {
        // 上升沿
        if (max30102_data.ir_ac > dynamic_threshold) {
            rising = true;
            printf("[HR_DEBUG] ⬆️ 上升沿，rising=true\r\n");
        }
    } else if (max30102_data.ir_ac < last_ir_ac) {
        // 下降沿
        if (rising) {
            // 检测到峰值
            uint32_t time_since_last = current_time - last_peak_time;

            // 🔥 调试：检测到峰值
            printf("[HR_DEBUG] ✓ 峰值! time_since_last=%lums\r\n", time_since_last);

            // 第一次检测到峰值：只初始化时间戳，不计算心率
            if (last_peak_time == 0) {
                last_peak_time = current_time;
                printf("[HR_DEBUG] 第一次峰值，初始化时间戳\r\n");
            }
            // 后续峰值：检查时间间隔并计算心率
            else {
                // 检查时间间隔是否合理（0.2-3.0秒之间）
                if (time_since_last > MAX30102_PEAK_MIN_INTERVAL && time_since_last < MAX30102_PEAK_MAX_INTERVAL) {
                    float instant_hr = 60000.0f / time_since_last;

                    // 🔥 调试：计算出的瞬时心率
                    printf("[HR_DEBUG] instant_hr=%.1f bpm\r\n", instant_hr);

                    // 验证心率合理性
                    if (MAX30102_IS_HR_VALID(instant_hr)) {
                        // 第一次检测快速响应
                        if (beat_avg == 0.0f) {
                            beat_avg = instant_hr;
                            max30102_data.heart_rate = beat_avg;
                            stable_count = 1;
                            max30102_data.hr_status = 1;  // 检测中
                        } else if (fabsf(instant_hr - beat_avg) < 45.0f) {
                            // 正常波动范围,平滑更新
                            beat_avg = MAX30102_HR_SMOOTH_ALPHA * beat_avg + 
                                       (1.0f - MAX30102_HR_SMOOTH_ALPHA) * instant_hr;
                            max30102_data.heart_rate = beat_avg;
                            stable_count++;
                            if (stable_count >= MAX30102_STABLE_THRESHOLD) {
                                max30102_data.hr_status = 2;  // 已稳定
                            } else {
                                max30102_data.hr_status = 1;  // 检测中
                            }
                        } else if (fabsf(instant_hr - beat_avg) < MAX30102_OUTLIER_THRESHOLD) {
                            // 中等波动(45-50 bpm),已稳定后才接受
                            if (stable_count >= MAX30102_STABLE_THRESHOLD) {
                                // 更保守的更新(降低权重)
                                beat_avg = 0.85f * beat_avg + 0.15f * instant_hr;
                                max30102_data.heart_rate = beat_avg;
                                max30102_data.hr_status = 2;  // 保持稳定
                            }
                            // 未稳定时拒绝接受(可能是干扰)
                        } else {
                            // 大幅波动(>50 bpm)
                            if (stable_count >= MAX30102_MIN_STABLE_COUNT) {
                                // 高度稳定后:极保守的更新(极低权重)
                                beat_avg = 0.90f * beat_avg + 0.10f * instant_hr;
                                max30102_data.heart_rate = beat_avg;
                                max30102_data.hr_status = 2;
                            } else if (stable_count <= 2) {
                                // 初期不稳定:可能初始值错误,允许大幅修正
                                beat_avg = 0.5f * beat_avg + 0.5f * instant_hr;
                                max30102_data.heart_rate = beat_avg;
                                stable_count = 1;  // 重置计数,重新稳定
                                max30102_data.hr_status = 1;
                            }
                            // 中间状态(3-4次)拒绝,避免随意跳变
                        }
                    }
                } else {
                    // 🔥 调试：时间间隔不合理
                    printf("[HR_DEBUG] ⚠️ 时间间隔不合理! interval=%lums (应在%d-%dms之间)\r\n",
                           time_since_last, MAX30102_PEAK_MIN_INTERVAL, MAX30102_PEAK_MAX_INTERVAL);
                }

                // 无论时间间隔是否合理，都更新时间戳
                last_peak_time = current_time;
            }
            rising = false;
        }
    }
    
    last_ir_ac = max30102_data.ir_ac;
    
    // 超时处理:根据稳定程度动态调整容忍度
    if (last_peak_time > 0) {
        uint32_t time_no_beat = current_time - last_peak_time;
        
        // 根据稳定次数决定超时阈值
        uint32_t timeout_threshold;
        if (stable_count >= MAX30102_MIN_STABLE_COUNT) {
            // 高度稳定:3秒才标记丢失
            timeout_threshold = 3000;
        } else if (stable_count >= MAX30102_STABLE_THRESHOLD) {
            // 已稳定:2.5秒标记丢失
            timeout_threshold = 2500;
        } else {
            // 未稳定:2秒标记丢失
            timeout_threshold = 2000;
        }
        
        if (time_no_beat > timeout_threshold) {
            max30102_data.hr_status = 3;  // 丢失状态
        }
        
        // 如果持续丢失5秒,强制重置(可能初始值错误)
        if (time_no_beat > 5000) {
            beat_avg = 0.0f;          // 清空旧值
            stable_count = 0;         // 重置稳定计数
            max30102_data.hr_status = 0;  // 回到初始化
        }
        
        // 完全超时清零
        if (time_no_beat > 12000) {
            max30102_data.heart_rate = 0.0f;
            last_peak_time = 0;
            init_count = 0;
            max_peak = -10000.0f;
            min_valley = 10000.0f;
        }
    }
}

/**
  * @brief  计算血氧饱和度(改进算法,减少波动)
  * @retval 无
  */
static void MAX30102_CalculateSpO2(void)
{
    // 计算R值(红光AC/DC 除以 红外AC/DC)
    if (max30102_data.ir_dc > 100.0f && max30102_data.red_dc > 100.0f) {
        float red_ratio = fabsf(max30102_data.red_ac) / max30102_data.red_dc;
        float ir_ratio = fabsf(max30102_data.ir_ac) / max30102_data.ir_dc;
        
        // 限制比值范围,防止异常值
        if (red_ratio > 0.001f && red_ratio < 1.0f && 
            ir_ratio > 0.001f && ir_ratio < 1.0f) {
            
            max30102_data.ratio = red_ratio / ir_ratio;
            
            // 限制R值范围(合理范围0.4-2.0)
            if (max30102_data.ratio < 0.4f) max30102_data.ratio = 0.4f;
            if (max30102_data.ratio > 2.0f) max30102_data.ratio = 2.0f;
            
            // 使用校准后的经验公式计算SpO2
            float spo2_raw = MAX30102_RATIO_TO_SPO2(max30102_data.ratio);
            
            // 初始化滤波值
            if (spo2_filtered == 0.0f) {
                spo2_filtered = spo2_raw;
            }
            
            // 指数平滑滤波,减少波动
            spo2_filtered = MAX30102_SPO2_SMOOTH_ALPHA * spo2_filtered + 
                           (1.0f - MAX30102_SPO2_SMOOTH_ALPHA) * spo2_raw;
            
            max30102_data.spo2 = spo2_filtered;
            
            // 限幅到合理范围
            if (max30102_data.spo2 < MAX30102_SPO2_MIN) {
                max30102_data.spo2 = MAX30102_SPO2_MIN;
            }
            if (max30102_data.spo2 > MAX30102_SPO2_MAX) {
                max30102_data.spo2 = MAX30102_SPO2_MAX;
            }
        }
    }
    
    // 改进的信号质量评估
    float ac_amplitude = fabsf(max30102_data.ir_ac);
    if (max30102_data.ir > 150000 && ac_amplitude > 500.0f) {
        max30102_data.signal_quality = 95;
    } else if (max30102_data.ir > 100000 && ac_amplitude > 300.0f) {
        max30102_data.signal_quality = 85;
    } else if (max30102_data.ir > 50000 && ac_amplitude > 100.0f) {
        max30102_data.signal_quality = 70;
    } else if (max30102_data.ir > 30000) {
        max30102_data.signal_quality = 50;
    } else {
        max30102_data.signal_quality = 20;
    }
}

/**
  * @brief  均值滤波器
  * @param  buffer: 滤波缓冲区
  * @param  new_val: 新数据
  * @param  size: 缓冲区大小
  * @retval 滤波后的值
  */
static float MAX30102_MeanFilter(float *buffer, float new_val, uint8_t size)
{
    static uint8_t index = 0;
    float sum = 0.0f;
    
    buffer[index] = new_val;
    index = (index + 1) % size;
    
    for (uint8_t i = 0; i < size; i++) {
        sum += buffer[i];
    }
    
    return sum / size;
}

/**
  * @brief  获取心率
  * @retval 心率(bpm)
  */
float MAX30102_GetHeartRate(void)
{
    return max30102_data.heart_rate;
}

/**
  * @brief  获取血氧饱和度
  * @retval 血氧(%)
  */
float MAX30102_GetSpO2(void)
{
    return max30102_data.spo2;
}

/**
  * @brief  检测手指是否放置
  * @retval true:有手指 false:无手指
  */
bool MAX30102_IsFingerDetected(void)
{
    return max30102_data.finger_detected;
}

/**
  * @brief  打印传感器信息
  * @retval 无
  */
void MAX30102_PrintInfo(void)
{
    if (max30102_data.finger_detected) {
        const char *status_str[] = {"初始化", "检测中", "稳定", "丢失"};
        printf("HR(心率): %.1f bpm [%s] | SpO2(血氧): %.1f%% | Signal(信号质量): %d%%\r\n", 
               max30102_data.heart_rate,
               status_str[max30102_data.hr_status],
               max30102_data.spo2,
               max30102_data.signal_quality);
    } else {
        printf("No finger detected!\r\n");
    }
}

/**
  * @brief  打印调试信息(帮助诊断心率检测问题)
  * @retval 无
  */
void MAX30102_PrintDebug(void)
{
    printf("[DEBUG] IR:%lu Red:%lu | IR_AC:%.1f Red_AC:%.1f | IR_DC:%.1f | Finger:%d\r\n",
           max30102_data.ir,
           max30102_data.red,
           max30102_data.ir_ac,
           max30102_data.red_ac,
           max30102_data.ir_dc,
           max30102_data.finger_detected);
}

/**
  * @brief  写寄存器
  * @param  reg: 寄存器地址
  * @param  data: 写入的数据
  * @retval true:成功 false:失败
  */
bool MAX30102_WriteReg(uint8_t reg, uint8_t data)
{
    return (HAL_I2C_Mem_Write(&MAX30102_I2C_HANDLE, MAX30102_I2C_ADDR, 
                              reg, I2C_MEMADD_SIZE_8BIT, 
                              &data, 1, MAX30102_I2C_TIMEOUT) == HAL_OK);
}

/**
  * @brief  读寄存器
  * @param  reg: 寄存器地址
  * @param  data: 读取的数据指针
  * @retval true:成功 false:失败
  */
bool MAX30102_ReadReg(uint8_t reg, uint8_t *data)
{
    return (HAL_I2C_Mem_Read(&MAX30102_I2C_HANDLE, MAX30102_I2C_ADDR, 
                             reg, I2C_MEMADD_SIZE_8BIT, 
                             data, 1, MAX30102_I2C_TIMEOUT) == HAL_OK);
}

/* 兼容旧版本的函数 ----------------------------------------------------------*/
/**
  * @brief  旧版任务函数(向后兼容)
  * @note   推荐使用MAX30102_Update()代替
  * @retval 无
  */
void max30102_task(void)
{
    // 检查ESP32是否启用心率检测
    extern volatile uint8_t heart_rate_enabled;

    // 🔥 调试：每5秒打印一次任务状态
    static uint32_t last_debug_time = 0;
    if ((HAL_GetTick() - last_debug_time) >= 5000) {
        printf("[MAX30102_TASK] 运行中, heart_rate_enabled=%d (地址: %p)\r\n",
               heart_rate_enabled, (void*)&heart_rate_enabled);
        last_debug_time = HAL_GetTick();
    }

    if (!heart_rate_enabled) {
        // 🔥 调试：未启用时也打印
        static uint32_t last_disabled_msg = 0;
        if ((HAL_GetTick() - last_disabled_msg) >= 10000) {
            printf("[MAX30102_TASK] ✗ 未启用，跳过检测 (heart_rate_enabled=%d)\r\n", heart_rate_enabled);
            last_disabled_msg = HAL_GetTick();
        }
        return;  // 未启用则不执行
    }

    // 🔥 调试：任务已启用
    static uint8_t first_run = 1;
    if (first_run) {
        printf("[MAX30102_TASK] ✓ 任务已启用，开始检测心率! (heart_rate_enabled=%d)\r\n", heart_rate_enabled);
        first_run = 0;
    }

    // 更新传感器数据
    MAX30102_Update();

    // 获取心率和血氧数据
    uint8_t hr = (uint8_t)MAX30102_GetHeartRate();
    uint8_t spo2 = (uint8_t)MAX30102_GetSpO2();

    // 发送数据给ESP32（每秒发送一次）
    static uint32_t last_send_time = 0;
    if ((HAL_GetTick() - last_send_time) >= 1000) {
        // 🔥 调试：打印心率数据
        printf("[MAX30102_TASK] HR=%d, SpO2=%d, Finger=%d\r\n",
               hr, spo2, max30102_data.finger_detected);

        if (hr > 0 && hr < 200) {  // 有效心率范围
            ESP32_COMM_SendHeartRate(hr, spo2);
            printf("[MAX30102_TASK] ✓ 发送数据到ESP32\r\n");
        } else {
            printf("[MAX30102_TASK] ✗ 心率无效，未发送\r\n");
        }
        last_send_time = HAL_GetTick();
    }
}

