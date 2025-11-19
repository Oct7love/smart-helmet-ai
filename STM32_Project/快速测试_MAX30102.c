/**
  ******************************************************************************
  * @brief  MAX30102快速测试代码（复制到main.c）
  ******************************************************************************
  */

/* 1. 在 USER CODE BEGIN Includes 添加 */
#include "max30102_app.h"

/* 2. 在main函数中初始化部分添加（MX_I2C2_Init()之后） */
void USER_CODE_SETUP(void)
{
    printf("\r\n======== MAX30102测试 ========\r\n");
    
    if (MAX30102_Init()) {
        printf("✓ MAX30102初始化成功!\r\n");
        printf("请将手指放在传感器上...\r\n\r\n");
    } else {
        printf("✗ MAX30102初始化失败!\r\n");
        printf("请检查:\r\n");
        printf("1. I2C2配置(PB10/PB11)\r\n");
        printf("2. 硬件连接\r\n");
        printf("3. 3.3V供电\r\n");
    }
}

/* 3. 在main函数的while(1)循环中添加 */
void USER_CODE_LOOP(void)
{
    MAX30102_Update();      // 更新数据
    MAX30102_PrintInfo();   // 打印信息
    HAL_Delay(50);          // 50ms周期
}

/* ========== 完整示例 ========== */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();
    MX_I2C1_Init();  // MPU6050
    MX_I2C2_Init();  // MAX30102 ← 确保调用
    
    /* USER CODE BEGIN 2 */
    USER_CODE_SETUP();  // 初始化MAX30102
    /* USER CODE END 2 */
    
    while (1)
    {
        /* USER CODE BEGIN 3 */
        USER_CODE_LOOP();  // 测试MAX30102
        /* USER CODE END 3 */
    }
}

