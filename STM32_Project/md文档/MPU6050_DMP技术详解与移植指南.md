# MPU6050 DMP 技术详解与移植指南

## 📚 目录
1. [DMP 算法原理详解](#1-dmp-算法原理详解)
2. [移植过程详解](#2-移植过程详解)
3. [应用层实现详解](#3-应用层实现详解)

---

## 1. DMP 算法原理详解

### 1.1 什么是 DMP？

**DMP (Digital Motion Processor)** = **数字运动处理器**

这是 MPU6050 内置的一个硬件协处理器，专门用于处理运动传感器数据。

#### 为什么需要 DMP？

**传统方式的问题：**
```
原始数据 → MCU读取 → 复杂计算（姿态融合）→ 得到姿态角
   ↓           ↓              ↓                    ↓
 陀螺仪      I2C通信      占用CPU资源          延迟大、精度低
加速度计                 需要浮点运算
```

**使用 DMP 的优势：**
```
原始数据 → DMP处理 → 四元数/姿态角 → MCU直接读取
   ↓         ↓            ↓              ↓
 内部      硬件加速    高精度结果      节省CPU
 传感器    独立运算    实时性好        功耗低
```

### 1.2 DMP 核心算法

#### 1.2.1 姿态解算算法

DMP 使用 **四元数（Quaternion）** 进行姿态融合：

**数学原理：**
```
四元数 Q = [q0, q1, q2, q3]
其中：
- q0: 实部（scalar）
- q1, q2, q3: 虚部（vector）
```

**优势：**
- ✅ 避免万向节锁（Gimbal Lock）
- ✅ 计算效率高
- ✅ 无奇异点
- ✅ 平滑连续

**四元数到欧拉角转换：**

```c
// DMP 输出四元数 → 转换为欧拉角
q0 = quat[0] / q30;  // q30 = 1073741824.0f (2^30)
q1 = quat[1] / q30;
q2 = quat[2] / q30;
q3 = quat[3] / q30;

// 转换公式（来自 mpu_dmp_get_data）
pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;  // 俯仰角
roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, 
              -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;  // 横滚角
yaw   = atan2(2 * (q1 * q2 + q0 * q3),
              q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3;  // 航向角
```

**为什么乘以 57.3？**
```
57.3 ≈ 180/π
弧度转角度：角度 = 弧度 × (180/π)
```

#### 1.2.2 传感器融合算法

DMP 内部使用 **互补滤波 + 卡尔曼滤波** 融合陀螺仪和加速度计数据：

**融合原理：**

1. **陀螺仪积分**（短期准确，长期漂移）
   ```
   角度(t) = 角度(t-1) + 角速度 × Δt
   问题：累积误差 → 长期漂移
   ```

2. **加速度计计算**（长期稳定，短期噪声大）
   ```
   Roll = atan2(ay, az)
   Pitch = atan2(-ax, sqrt(ay² + az²))
   问题：受震动影响 → 短期抖动
   ```

3. **DMP 融合策略**
   ```
   最终角度 = α × 陀螺仪积分 + (1-α) × 加速度计
   其中 α 动态调整：
   - 静止时：更信任加速度计（α小）
   - 运动时：更信任陀螺仪（α大）
   ```

**DMP 固件内部流程：**
```
[陀螺仪] ──┐
           ├──> [预处理] ──> [互补滤波] ──> [卡尔曼滤波] ──> [四元数]
[加速度] ──┘      ↓              ↓              ↓
                去噪声        姿态融合      最优估计
                校准零偏      动态权重      误差修正
```

#### 1.2.3 计步器算法

DMP 内置的计步算法基于 **加速度模式识别**：

**原理：**

1. **检测步态特征**
   ```
   走路时的加速度波形：
   
   加速度
     ^
     |    /\      /\      /\
     |   /  \    /  \    /  \
     |  /    \  /    \  /    \
     | /      \/      \/      \
     +-------------------------> 时间
         步1     步2     步3
   
   特征：
   - 周期性波动
   - 峰值识别
   - 频率范围：0.5-3Hz
   ```

2. **步态检测条件**
   ```c
   // DMP 内部伪代码
   if (加速度变化幅度 > 阈值 && 
       频率在合理范围 && 
       模式匹配步态特征) {
       步数++;
   }
   ```

3. **防误判机制**
   - 时间窗口：连续两步间隔 0.3-2 秒
   - 幅度阈值：排除小幅度抖动
   - 模式匹配：排除非步态运动

**调用方式：**
```c
// 启用计步功能
dmp_enable_feature(DMP_FEATURE_PEDOMETER);

// 读取步数
unsigned long steps;
dmp_get_pedometer_step_count(&steps);
```

---

## 2. 移植过程详解

### 2.1 移植架构

**分层设计：**
```
┌─────────────────────────────────────┐
│      应用层 (mpu_app.c/h)           │  ← 我们编写的高层API
│  - MPU6050_DMP_Init()               │
│  - MPU6050_Update()                 │
│  - 摔倒检测、计步器等               │
├─────────────────────────────────────┤
│    DMP辅助层 (inv_mpu.c末尾)       │  ← 正点原子的辅助函数
│  - mpu_dmp_init()                   │
│  - mpu_dmp_get_data()               │
│  - run_self_test()                  │
├─────────────────────────────────────┤
│    eMPL库 (InvenSense官方)         │  ← DMP固件和驱动
│  - inv_mpu.c/h                      │
│  - inv_mpu_dmp_motion_driver.c/h   │
│  - dmpKey.h, dmpmap.h (固件)       │
├─────────────────────────────────────┤
│    HAL抽象层 (我们修改的部分)      │  ← 平台适配
│  - i2c_write/read → HAL_I2C_xxx    │
│  - delay_ms → HAL_Delay             │
│  - get_ms → HAL_GetTick             │
└─────────────────────────────────────┘
```

### 2.2 核心修改详解

#### 修改 1: inv_mpu.h - 简化中断结构

**为什么修改？**
```c
// 原始代码（针对 MSP430/UC3L0 平台）
struct int_param_s {
#if defined EMPL_TARGET_MSP430
    void (*cb)(void);
    unsigned short pin;
    unsigned char lp_exit;
    unsigned char active_low;
#elif defined EMPL_TARGET_UC3L0
    unsigned long pin;
    void (*cb)(volatile void*);
    void *arg;
#endif
};
```

**问题：** STM32 不属于上述任何平台，编译会报错。

**解决方案：**
```c
// 简化为最小结构（STM32不使用外部中断）
struct int_param_s {
    void *arg;  // 占位符，实际未使用
};
```

**原理：** DMP 使用轮询模式读取 FIFO，不需要中断回调。

---

#### 修改 2: inv_mpu.c - 平台适配

**核心修改：替换平台相关代码**

**原始代码：**
```c
#if defined MOTION_DRIVER_TARGET_MSP430
    #include "msp430_i2c.h"
    #define i2c_write   msp430_i2c_write
    #define i2c_read    msp430_i2c_read
    // ...
#elif defined EMPL_TARGET_UC3L0
    #include "twi.h"
    #define i2c_write(a,b,c,d) twi_write(a,b,d,c)
    // ...
#else
    #error Gyro driver is missing implementations
#endif
```

**我们的修改：**
```c
/* STM32 HAL Library Platform Adaptation */
#include "i2c.h"
#include "main.h"

// I2C 操作适配
#define i2c_write(dev_addr, reg_addr, data_size, p_data) \
    HAL_I2C_Mem_Write(&hi2c1, dev_addr, reg_addr, \
                      I2C_MEMADD_SIZE_8BIT, p_data, data_size, 0xFFFFFF)

#define i2c_read(dev_addr, reg_addr, data_size, p_data) \
    HAL_I2C_Mem_Read(&hi2c1, dev_addr, reg_addr, \
                     I2C_MEMADD_SIZE_8BIT, p_data, data_size, 0xFFFFFF)

// 时间函数适配
#define delay_ms HAL_Delay
#define get_ms(p) do{ *p = HAL_GetTick(); } while(0)

// 日志函数（关闭）
#define log_i(...) do {} while (0)
#define log_e(...) do {} while (0)

// 数学函数
#define fabs fabsf
#define min(a,b) ((a<b)?a:b)
```

**为什么这样做？**

1. **I2C 通信适配**
   ```c
   // eMPL 调用
   i2c_write(0xD0, 0x6B, 1, &data);
   
   // 实际执行
   HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x6B, 
                     I2C_MEMADD_SIZE_8BIT, &data, 1, 0xFFFFFF);
   ```

2. **时间函数适配**
   ```c
   // eMPL 调用
   delay_ms(100);        // 延时100ms
   get_ms(&timestamp);   // 获取时间戳
   
   // 实际执行
   HAL_Delay(100);
   timestamp = HAL_GetTick();
   ```

**修改 I2C 地址：**
```c
// 原始代码
const struct hw_s hw = {
    .addr = 0x68,  // 7位地址
    // ...
};

// 修改后
const struct hw_s hw = {
    .addr = 0xD0,  // 8位地址（含读写位）
    // ...
};
```

**原因：** HAL 库使用 8 位地址格式（`地址 << 1`）。

**删除中断注册：**
```c
// 删除这两行（STM32 不使用外部中断）
if (int_param)
    reg_int_cb(int_param);
```

---

#### 修改 3: inv_mpu_dmp_motion_driver.c - 时间函数

**修改内容：**
```c
/* STM32 HAL Library Platform Adaptation for DMP */
#include "main.h"
#define delay_ms HAL_Delay
#define get_ms(p) do{ *p = HAL_GetTick(); } while(0)
#define log_i(...) do {} while (0)
#define log_e(...) do {} while (0)
```

**修改 __no_operation() 为 __NOP()：**
```c
// 原始代码（IAR编译器）
__no_operation();

// 修改后（ARM GCC）
__NOP();
```

**原因：** 不同编译器的内置函数名称不同。

---

#### 修改 4: 添加辅助函数（inv_mpu.c 末尾）

参考正点原子代码，添加三个关键函数：

**1. mpu_dmp_init() - 一键初始化**
```c
uint8_t mpu_dmp_init(void)
{
    uint8_t res = 0;
    struct int_param_s int_param;
    
    // 步骤1: 初始化 MPU6050
    if (mpu_init(&int_param) == 0) {
        
        // 步骤2: 设置传感器（陀螺仪+加速度计）
        res = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
        if (res) return 1;
        
        // 步骤3: 配置 FIFO
        res = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
        if (res) return 2;
        
        // 步骤4: 设置采样率（100Hz）
        res = mpu_set_sample_rate(DEFAULT_MPU_HZ);
        if (res) return 3;
        
        // 步骤5: 加载 DMP 固件（关键！）
        res = dmp_load_motion_driver_firmware();
        if (res) return 4;
        
        // 步骤6: 设置陀螺仪方向
        res = dmp_set_orientation(
            inv_orientation_matrix_to_scalar(gyro_orientation));
        if (res) return 5;
        
        // 步骤7: 使能 DMP 功能
        res = dmp_enable_feature(
            DMP_FEATURE_6X_LP_QUAT |          // 6轴四元数
            DMP_FEATURE_TAP |                  // 敲击检测
            DMP_FEATURE_ANDROID_ORIENT |       // 屏幕旋转
            DMP_FEATURE_SEND_RAW_ACCEL |       // 原始加速度
            DMP_FEATURE_SEND_CAL_GYRO |        // 校准陀螺仪
            DMP_FEATURE_GYRO_CAL);             // 陀螺仪校准
        if (res) return 6;
        
        // 步骤8: 设置 FIFO 速率
        res = dmp_set_fifo_rate(DEFAULT_MPU_HZ);
        if (res) return 7;
        
        // 步骤9: 自检（可选，已注释）
        // res = run_self_test();
        // if (res) return 8;
        
        // 步骤10: 使能 DMP
        res = mpu_set_dmp_state(1);
        if (res) return 9;
        
    } else {
        return 10;  // MPU6050 未找到
    }
    
    return 0;  // 成功
}
```

**为什么需要这个函数？**
- 封装复杂的初始化流程
- 提供清晰的错误代码
- 简化应用层调用

**2. mpu_dmp_get_data() - 读取姿态角**
```c
uint8_t mpu_dmp_get_data(float *pitch, float *roll, float *yaw)
{
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    unsigned long sensor_timestamp;
    short gyro[3], accel[3], sensors;
    unsigned char more;
    long quat[4];
    
    // 从 FIFO 读取数据
    if (dmp_read_fifo(gyro, accel, quat, 
                      &sensor_timestamp, &sensors, &more))
        return 1;
    
    // 检查是否有四元数数据
    if (sensors & INV_WXYZ_QUAT) {
        // 转换 q30 格式为浮点数
        q0 = quat[0] / q30;
        q1 = quat[1] / q30;
        q2 = quat[2] / q30;
        q3 = quat[3] / q30;
        
        // 四元数 → 欧拉角
        *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;
        *roll = atan2(2 * q2 * q3 + 2 * q0 * q1, 
                     -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;
        *yaw = atan2(2 * (q1 * q2 + q0 * q3),
                    q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3;
    } else {
        return 2;
    }
    
    return 0;
}
```

**关键点：**
- `q30` 格式：DMP 输出的四元数是 30 位定点数
- `q30 = 1073741824.0f = 2^30`
- 需要除以 q30 转换为浮点数

**3. run_self_test() - 自检与校准**
```c
uint8_t run_self_test(void)
{
    int result;
    long gyro[3], accel[3];
    
    // 运行自检
    result = mpu_run_self_test(gyro, accel);
    
    if (result == 0x3) {  // 陀螺仪和加速度计都通过
        // 获取灵敏度
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        mpu_get_accel_sens(&accel_sens);
        
        // 应用偏差校准到 DMP
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
        
        return 0;
    }
    
    return 1;  // 自检失败
}
```

**为什么我们注释掉了？**
- 自检对环境要求很高（需要完全静止）
- 很多项目直接跳过自检
- 不影响 DMP 正常工作

---

### 2.3 为什么要这样移植？

**设计原则：**

1. **最小侵入原则**
   - 只修改平台相关代码
   - 保留 eMPL 库核心逻辑
   - 便于后续升级

2. **分层封装原则**
   ```
   应用层 → 不关心底层细节
   辅助层 → 简化调用流程
   驱动层 → 提供标准接口
   HAL层 → 平台适配
   ```

3. **可移植性原则**
   - 通过宏定义适配不同平台
   - 更换平台只需修改 HAL 层
   - 应用层代码不变

---

## 3. 应用层实现详解

### 3.1 整体架构（mpu_app.c）

```c
┌─────────────────────────────────────┐
│  MPU6050_DMP_Init()                 │  初始化入口
├─────────────────────────────────────┤
│  MPU6050_Update()                   │  数据更新（周期调用）
│    ├─ mpu_dmp_get_data()            │    读取姿态角
│    ├─ MPU6050_ReadAccel()           │    读取加速度
│    ├─ MPU6050_CalculatePhysicalValues() │ 转换物理值
│    └─ MPU6050_UpdateFallDetection() │    摔倒检测
├─────────────────────────────────────┤
│  计步器功能                          │
│    ├─ MPU6050_GetStepCount()        │    获取步数
│    ├─ MPU6050_GetDistance()         │    计算距离
│    ├─ MPU6050_SetStepLength()       │    设置步距
│    └─ MPU6050_ResetStepCount()      │    重置步数
├─────────────────────────────────────┤
│  摔倒检测功能                        │
│    ├─ MPU6050_UpdateFallDetection() │    更新检测逻辑
│    ├─ MPU6050_IsFallDetected()      │    查询摔倒状态
│    └─ MPU6050_IsCollisionDetected() │    查询碰撞状态
└─────────────────────────────────────┘
```

### 3.2 DMP 姿态读取实现

**流程图：**
```
[调度器 100ms] 
    ↓
MPU6050_Update()
    ↓
mpu_dmp_get_data(&pitch, &roll, &yaw)
    ↓
dmp_read_fifo() ← 从硬件 FIFO 读取
    ↓
[四元数 q30 格式]
    ↓
转换为浮点数
    ↓
四元数 → 欧拉角转换
    ↓
[Pitch, Roll, Yaw]
    ↓
存入 mpu6050_data 全局变量
```

**代码实现：**
```c
MPU6050_Status_t MPU6050_Update(void)
{
    uint8_t res;
    
    // 1. 检查 DMP 是否已使能
    if (!mpu6050_data.dmp_enabled) {
        return MPU6050_ERROR;
    }
    
    // 2. 读取 DMP 姿态数据
    res = mpu_dmp_get_data(&mpu6050_data.pitch, 
                           &mpu6050_data.roll, 
                           &mpu6050_data.yaw);
    
    if (res == 0) {
        mpu6050_data.is_valid = 1;
        
        // 3. 读取原始加速度（用于摔倒检测）
        MPU6050_ReadAccel(&mpu6050_data.accel_x_raw,
                         &mpu6050_data.accel_y_raw,
                         &mpu6050_data.accel_z_raw);
        
        // 4. 计算物理值
        MPU6050_CalculatePhysicalValues();
        
        // 5. 更新摔倒检测
        MPU6050_UpdateFallDetection();
        
        return MPU6050_OK;
    } else {
        mpu6050_data.is_valid = 0;
        return MPU6050_ERROR;
    }
}
```

**为什么要分开读取姿态和加速度？**
- DMP 输出的是融合后的姿态角（精度高）
- 原始加速度用于摔倒检测（需要瞬时值）
- 两者用途不同，数据来源不同

---

### 3.3 计步器功能实现

**DMP 计步器原理：**

DMP 内部有专门的计步算法，我们只需调用 API：

```c
uint32_t MPU6050_GetStepCount(void)
{
    unsigned long steps = 0;
    
    // 从 DMP 读取步数
    dmp_get_pedometer_step_count(&steps);
    
    // 更新全局数据
    mpu6050_data.step_count = steps;
    
    // 计算行走距离
    mpu6050_data.distance = mpu6050_data.step_count * 
                            mpu6050_data.step_length;
    
    return mpu6050_data.step_count;
}
```

**距离计算：**
```c
距离（米）= 步数 × 步距（米）

例如：
步数 = 1000
步距 = 0.3m (30cm)
距离 = 1000 × 0.3 = 300m
```

**步距设置：**
```c
void MPU6050_SetStepLength(float length)
{
    mpu6050_data.step_length = length;
}

// 使用示例
MPU6050_SetStepLength(0.35f);  // 设置为35cm
```

**步距参考值：**
- 儿童：0.25-0.3m
- 成人女性：0.3-0.35m
- 成人男性：0.35-0.4m
- 快步走：0.4-0.5m

**重置步数：**
```c
void MPU6050_ResetStepCount(void)
{
    dmp_set_pedometer_step_count(0);  // 重置 DMP 内部计数器
    mpu6050_data.step_count = 0;
    mpu6050_data.distance = 0.0f;
}
```

---

### 3.4 摔倒检测实现

**检测原理：双重判断**

#### 方法1: 角度检测

```c
// 倾斜角度超过阈值 → 判定为摔倒
if (fabs(mpu6050_data.pitch) > MPU6050_FALL_ANGLE_THRESHOLD || 
    fabs(mpu6050_data.roll) > MPU6050_FALL_ANGLE_THRESHOLD)
{
    mpu6050_data.fall_flag = 1;  // 摔倒标志
}
```

**原理图：**
```
正常站立：              摔倒：
    ↑ Z                    
    |                   ──┐ Z
    |                     |
    |______ Y            ↓|_____ Y
    
Pitch ≈ 0°            Pitch > 60°
Roll ≈ 0°             Roll > 60°
```

**为什么选 60°？**
- 正常站立：Pitch/Roll < 10°
- 弯腰/下蹲：10° - 45°
- 摔倒/躺下：> 60°
- 60° 是经验阈值，可根据实际调整

#### 方法2: 碰撞检测（SVM 算法）

**SVM = Signal Vector Magnitude（信号矢量幅值）**

```c
// 计算加速度矢量和
mpu6050_data.svm = sqrt(pow(mpu6050_data.accel_x_raw, 2) + 
                        pow(mpu6050_data.accel_y_raw, 2) + 
                        pow(mpu6050_data.accel_z_raw, 2));
```

**物理意义：**
```
SVM = |a| = √(ax² + ay² + az²)

静止状态：SVM ≈ 16384 (1g)
自由落体：SVM ≈ 0
剧烈碰撞：SVM > 23000 (>1.5g)
```

**碰撞检测逻辑：**

```c
// 检测加速度突变
if (mpu6050_data.svm > MPU6050_COLLISION_SVM_HIGH ||  // >23000
    mpu6050_data.svm < MPU6050_COLLISION_SVM_LOW)     // <12000
{
    mpu6050_data.collision_counter = 0;  // 重置计数器
}

mpu6050_data.collision_counter++;

// 时间窗口内标记为碰撞
if (mpu6050_data.collision_counter <= MPU6050_COLLISION_TIME_WINDOW)  // ≤10
{
    mpu6050_data.collision_flag = 1;
}
else
{
    mpu6050_data.collision_flag = 0;
}
```

**时间窗口原理：**
```
碰撞发生：
t=0    t=1    t=2    ...  t=10   t=11
 ↓      ↓      ↓            ↓      ↓
[碰撞] → 标记为碰撞 → ... → 标记 → 清除标记

碰撞后 1 秒内（10次×100ms）保持碰撞标志
```

**为什么需要时间窗口？**
- 碰撞是瞬时事件
- 防止标志闪烁
- 给应用层足够的响应时间

---

### 3.5 完整检测流程

```c
void MPU6050_UpdateFallDetection(void)
{
    // ============ 步骤1: 计算 SVM ============
    mpu6050_data.svm = sqrt(
        pow(mpu6050_data.accel_x_raw, 2) + 
        pow(mpu6050_data.accel_y_raw, 2) + 
        pow(mpu6050_data.accel_z_raw, 2)
    );
    
    // ============ 步骤2: 角度检测 ============
    if (fabs(mpu6050_data.pitch) > 60.0f || 
        fabs(mpu6050_data.roll) > 60.0f)
    {
        mpu6050_data.fall_flag = 1;  // 摔倒
    }
    else
    {
        mpu6050_data.fall_flag = 0;  // 正常
    }
    
    // ============ 步骤3: 碰撞检测 ============
    // 检测加速度突变
    if (mpu6050_data.svm > 23000 || mpu6050_data.svm < 12000)
    {
        mpu6050_data.collision_counter = 0;
    }
    
    mpu6050_data.collision_counter++;
    
    // 时间窗口判断
    if (mpu6050_data.collision_counter <= 10)
    {
        mpu6050_data.collision_flag = 1;  // 碰撞
    }
    else
    {
        if (mpu6050_data.collision_counter > 10)
            mpu6050_data.collision_counter = 10;  // 限幅
        
        mpu6050_data.collision_flag = 0;  // 正常
    }
}
```

---

### 3.6 实际应用场景

#### 场景1: 智能安全帽

```c
void smart_helmet_monitor(void)
{
    if (MPU6050_Update() == MPU6050_OK)
    {
        // 摔倒检测
        if (MPU6050_IsFallDetected())
        {
            // 触发报警
            Buzzer_ON();
            Send_SMS_Alert("工人摔倒！");
            Send_GPS_Location();
            Record_Accident_Time();
        }
        
        // 碰撞检测
        if (MPU6050_IsCollisionDetected())
        {
            int svm = MPU6050_GetSVM();
            printf("碰撞强度: %d\r\n", svm);
            
            if (svm > 30000)  // 剧烈碰撞
            {
                Emergency_Stop();
                Call_Ambulance();
            }
        }
        
        // 姿态异常
        float pitch = MPU6050_GetPitch();
        if (fabs(pitch) > 45.0f)
        {
            printf("警告：头部倾斜过大！\r\n");
        }
    }
}
```

#### 场景2: 老人看护

```c
void elderly_care_system(void)
{
    static uint32_t fall_start_time = 0;
    
    if (MPU6050_IsFallDetected())
    {
        if (fall_start_time == 0)
        {
            fall_start_time = HAL_GetTick();
        }
        
        // 摔倒超过5秒 → 严重情况
        if (HAL_GetTick() - fall_start_time > 5000)
        {
            Send_Alert_To_Family();
            Call_Emergency_Service();
        }
    }
    else
    {
        fall_start_time = 0;  // 恢复正常
    }
    
    // 活动监测
    uint32_t steps = MPU6050_GetStepCount();
    if (steps < 100)  // 一天步数少于100
    {
        printf("提醒：今日活动量不足\r\n");
    }
}
```

#### 场景3: 运动追踪

```c
void fitness_tracker(void)
{
    static uint32_t last_steps = 0;
    static float total_calories = 0.0f;
    
    // 步数统计
    uint32_t steps = MPU6050_GetStepCount();
    float distance = MPU6050_GetDistance();
    
    if (steps != last_steps)
    {
        // 卡路里计算（简化公式）
        float calories = (steps - last_steps) * 0.04f;  // 每步0.04卡
        total_calories += calories;
        
        printf("步数: %lu\r\n", steps);
        printf("距离: %.2f km\r\n", distance / 1000.0f);
        printf("热量: %.1f kcal\r\n", total_calories);
        
        last_steps = steps;
    }
    
    // 运动强度分析
    int svm = MPU6050_GetSVM();
    if (svm > 20000)
        printf("运动强度: 高\r\n");
    else if (svm > 17000)
        printf("运动强度: 中\r\n");
    else
        printf("运动强度: 低\r\n");
}
```

---

## 4. 总结

### 4.1 DMP 算法优势

| 特性 | 软件算法 | DMP 算法 |
|------|----------|----------|
| 计算负担 | 占用 MCU 30-50% | < 5% |
| 精度 | ±2-5° | ±0.5° |
| 延迟 | 10-50ms | < 5ms |
| 功耗 | 高 | 低（硬件加速）|
| 抗干扰 | 一般 | 强（固件优化）|

### 4.2 移植要点总结

1. **平台适配**：修改 I2C、时间函数
2. **地址转换**：0x68 → 0xD0
3. **删除中断**：STM32 使用轮询模式
4. **添加辅助函数**：简化初始化和数据读取
5. **跳过自检**：提高成功率

### 4.3 应用层设计要点

1. **分层架构**：清晰的接口层次
2. **错误处理**：详细的错误代码
3. **功能封装**：计步器、摔倒检测独立模块
4. **灵活配置**：可调整阈值参数
5. **实时性**：100ms 更新周期

---

## 📚 参考资料

1. [MPU6050 数据手册](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)
2. [DMP 寄存器手册](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
3. 正点原子 MPU6050 教程
4. InvenSense eMPL 库文档

---

**文档版本：** v1.0  
**创建日期：** 2025/11/11  
**作者：** AI Assistant  

**本文档详细解释了 DMP 算法原理、移植过程和应用实现，希望对您的项目有帮助！** 🚀

