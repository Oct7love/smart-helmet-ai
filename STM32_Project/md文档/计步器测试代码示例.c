/**
 * @file    计步器测试代码示例.c
 * @brief   MPU6050 计步器功能测试代码
 * @author  AI Assistant
 * @date    2025-11-11
 */

/* 在 main.c 中使用这些代码片段 */

//=============================================================================
// 测试1: 基础计步测试
//=============================================================================
void Test_BasicPedometer(void)
{
    printf("\r\n=== 基础计步测试 ===\r\n");
    printf("请准备行走20步...\r\n");
    HAL_Delay(2000);
    
    // 重置计步器
    MPU6050_ResetStepCount();
    MPU6050_SetStepLength(0.35f);  // 根据自己身高调整
    
    printf("开始计步！\r\n");
    uint32_t start_tick = HAL_GetTick();
    uint32_t last_steps = 0;
    
    // 监测30秒
    while ((HAL_GetTick() - start_tick) < 30000)
    {
        // 更新数据（这是关键！）
        if (MPU6050_Update() == MPU6050_OK)
        {
            uint32_t steps = mpu6050_data.step_count;
            float distance = mpu6050_data.distance;
            
            // 步数变化时打印
            if (steps != last_steps)
            {
                printf("步数: %lu, 距离: %.2fm\r\n", steps, distance);
                last_steps = steps;
            }
        }
        
        HAL_Delay(100);  // 100ms 更新一次
    }
    
    printf("测试结束！最终步数: %lu\r\n", mpu6050_data.step_count);
}

//=============================================================================
// 测试2: 实时监控（显示详细信息）
//=============================================================================
void Test_RealtimeMonitor(void)
{
    static uint32_t last_print_tick = 0;
    static uint32_t last_steps = 0;
    
    // 每秒打印一次
    if ((HAL_GetTick() - last_print_tick) >= 1000)
    {
        if (MPU6050_Update() == MPU6050_OK)
        {
            uint32_t steps = mpu6050_data.step_count;
            float distance = mpu6050_data.distance;
            int svm = mpu6050_data.svm;
            
            printf("步数:%lu  距离:%.2fm  SVM:%d", steps, distance, svm);
            
            // 标记步数变化
            if (steps != last_steps)
            {
                printf(" ✓ +%lu步", steps - last_steps);
                last_steps = steps;
            }
            
            printf("\r\n");
        }
        
        last_print_tick = HAL_GetTick();
    }
}

//=============================================================================
// 测试3: 步频分析
//=============================================================================
void Test_StepFrequencyAnalysis(void)
{
    static uint32_t last_steps = 0;
    static uint32_t last_step_time = 0;
    static uint32_t step_intervals[10] = {0};  // 记录最近10个步间隔
    static uint32_t interval_index = 0;
    
    if (MPU6050_Update() == MPU6050_OK)
    {
        uint32_t steps = mpu6050_data.step_count;
        
        // 检测到新步数
        if (steps > last_steps)
        {
            uint32_t now = HAL_GetTick();
            
            if (last_step_time != 0)
            {
                // 计算步间隔
                uint32_t interval = now - last_step_time;
                step_intervals[interval_index] = interval;
                interval_index = (interval_index + 1) % 10;
                
                // 计算平均步频
                uint32_t sum = 0;
                for (int i = 0; i < 10; i++)
                {
                    sum += step_intervals[i];
                }
                uint32_t avg_interval = sum / 10;
                float step_freq = 60000.0f / avg_interval;  // 步/分钟
                
                printf("第 %lu 步  间隔:%lums  频率:%.1f步/分\r\n",
                       steps, interval, step_freq);
                
                // 分析步态
                if (step_freq < 60)
                    printf("  步态: 慢走\r\n");
                else if (step_freq < 120)
                    printf("  步态: 正常\r\n");
                else if (step_freq < 140)
                    printf("  步态: 快走\r\n");
                else
                    printf("  步态: 慢跑\r\n");
            }
            
            last_step_time = now;
            last_steps = steps;
        }
    }
}

//=============================================================================
// 测试4: 校准测试
//=============================================================================
void Test_PedometerCalibration(void)
{
    printf("\r\n=== 计步器校准测试 ===\r\n");
    printf("请准确行走20步并数清楚！\r\n");
    printf("3秒后开始...\r\n");
    HAL_Delay(3000);
    
    // 重置
    MPU6050_ResetStepCount();
    
    printf("开始行走！\r\n");
    uint32_t start_tick = HAL_GetTick();
    uint32_t last_detected_steps = 0;
    
    // 监测60秒（足够走20步）
    while ((HAL_GetTick() - start_tick) < 60000)
    {
        if (MPU6050_Update() == MPU6050_OK)
        {
            uint32_t steps = mpu6050_data.step_count;
            
            // 步数变化时显示
            if (steps != last_detected_steps)
            {
                printf("检测到: %lu 步\r\n", steps);
                last_detected_steps = steps;
            }
            
            // 达到20步后自动结束
            if (steps >= 20)
            {
                HAL_Delay(2000);  // 等待2秒确认
                break;
            }
        }
        
        HAL_Delay(100);
    }
    
    uint32_t final_steps = mpu6050_data.step_count;
    float accuracy = (final_steps / 20.0f) * 100.0f;
    
    printf("\r\n=== 校准结果 ===\r\n");
    printf("实际步数: 20\r\n");
    printf("检测步数: %lu\r\n", final_steps);
    printf("准确率: %.1f%%\r\n", accuracy);
    
    if (accuracy >= 95.0f)
        printf("✅ 精度优秀！\r\n");
    else if (accuracy >= 85.0f)
        printf("⚠️ 精度良好\r\n");
    else if (accuracy >= 75.0f)
        printf("⚠️ 精度一般，建议调整安装位置\r\n");
    else
        printf("❌ 精度偏低，请检查安装方式\r\n");
}

//=============================================================================
// 测试5: 加速度与步数关联测试
//=============================================================================
void Test_AccelStepCorrelation(void)
{
    static uint32_t last_steps = 0;
    static int last_svm = 16384;  // 1g = 16384
    
    if (MPU6050_Update() == MPU6050_OK)
    {
        uint32_t steps = mpu6050_data.step_count;
        int svm = mpu6050_data.svm;
        int svm_diff = abs(svm - last_svm);
        
        // 显示加速度变化
        if (svm_diff > 1000)
        {
            printf("加速度变化: SVM=%d (Δ=%d)", svm, svm_diff);
            
            // 标记是否计步
            if (steps != last_steps)
            {
                printf(" ✓ 计步");
                last_steps = steps;
            }
            else
            {
                printf(" ✗ 未计步");
            }
            
            printf("\r\n");
        }
        
        last_svm = svm;
    }
}

//=============================================================================
// 测试6: 综合显示（放在主循环）
//=============================================================================
void Test_ComprehensiveDisplay(void)
{
    static uint32_t last_print_tick = 0;
    
    // 每500ms 更新一次
    if ((HAL_GetTick() - last_print_tick) >= 500)
    {
        if (MPU6050_Update() == MPU6050_OK)
        {
            printf("\r\n=== MPU6050 实时数据 ===\r\n");
            printf("姿态: Pitch=%.1f° Roll=%.1f° Yaw=%.1f°\r\n",
                   mpu6050_data.pitch, mpu6050_data.roll, mpu6050_data.yaw);
            
            printf("加速度: X=%d Y=%d Z=%d (SVM=%d)\r\n",
                   mpu6050_data.accel_x_raw, 
                   mpu6050_data.accel_y_raw,
                   mpu6050_data.accel_z_raw,
                   mpu6050_data.svm);
            
            printf("计步: 步数=%lu  距离=%.2fm (%.3fkm)\r\n",
                   mpu6050_data.step_count,
                   mpu6050_data.distance,
                   mpu6050_data.distance / 1000.0f);
            
            if (mpu6050_data.fall_flag)
                printf("⚠️ 摔倒检测！\r\n");
            
            if (mpu6050_data.collision_flag)
                printf("⚠️ 碰撞检测！\r\n");
            
            printf("=======================\r\n");
        }
        
        last_print_tick = HAL_GetTick();
    }
}

//=============================================================================
// 主函数使用示例
//=============================================================================
void Example_MainLoop(void)
{
    // 初始化
    if (MPU6050_DMP_Init() != MPU6050_OK)
    {
        printf("MPU6050 初始化失败！\r\n");
        while(1);
    }
    
    // 设置步距
    MPU6050_SetStepLength(0.35f);
    
    printf("MPU6050 初始化成功！\r\n");
    printf("开始计步测试...\r\n");
    
    // 可选：运行校准测试
    // Test_PedometerCalibration();
    
    // 主循环
    while(1)
    {
        // 选择一个测试函数
        Test_ComprehensiveDisplay();  // 综合显示
        // Test_RealtimeMonitor();       // 实时监控
        // Test_StepFrequencyAnalysis(); // 步频分析
        // Test_AccelStepCorrelation();  // 加速度关联
        
        HAL_Delay(10);
    }
}

//=============================================================================
// 快速调试代码（最简单版本）
//=============================================================================
void QuickTest_Pedometer(void)
{
    static uint32_t last_steps = 0;
    
    // 100ms 更新一次
    if (MPU6050_Update() == MPU6050_OK)
    {
        uint32_t steps = mpu6050_data.step_count;
        
        // 步数变化时打印
        if (steps != last_steps)
        {
            printf("步数: %lu\r\n", steps);
            last_steps = steps;
        }
    }
}

/*
=============================================================================
使用说明：

1. 在 main.c 的 main() 函数中初始化：
   
   MPU6050_DMP_Init();
   MPU6050_SetStepLength(0.35f);

2. 在 main() 的 while(1) 循环中调用：
   
   // 最简单的方式
   QuickTest_Pedometer();
   HAL_Delay(100);
   
   // 或者使用详细测试
   Test_ComprehensiveDisplay();
   HAL_Delay(10);

3. 测试流程：
   
   a) 编译下载
   b) 打开串口（115200）
   c) 将设备固定在腰部
   d) 正常行走20-30步
   e) 观察串口输出

4. 预期结果：
   
   步数应该接近实际步数（误差 ±10%）
   距离 = 步数 × 步距

=============================================================================
*/

