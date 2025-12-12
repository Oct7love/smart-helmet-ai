/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_system.h"
#include "max30102_app.h"
#include "oled.h"
#include "gps_app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // 初始化延时函数
  ringbuffer_init(&usart_rb);  // ✅ 加这一行
  delay_init();

  OLED_Init();
  // printf("OLED Init OK\r\n");

  // 初始化MAX30102并检查结果
  printf("[MAIN] 正在初始化MAX30102...\r\n");
  if (MAX30102_Init()) {
      printf("[MAIN] ✓ MAX30102初始化成功\r\n");
  } else {
      printf("[MAIN] ✗ MAX30102初始化失败！\r\n");
  }
  MQ2_Init();
  MPU6050_Init();
  // MPU6050_DMP_Init();
  // /*max30102测试*/
  // printf("\r\n=== MAX30102测试 ===\r\n");
  // if (MAX30102_Init()) {
  //   printf("✓ 初始化成功!\r\n");
  //   printf("请将手指放在传感器上...\r\n\r\n");
  // } else {
  //   printf("✗ 初始化失败!\r\n");
  // }


  // 等待MPU6050上电稳定
  HAL_Delay(100);

  // 初始化MPU6050 DMP（必须在main中初始化，不能在task中）
  printf("Initializing MPU6050 DMP...\r\n");
  MPU6050_Status_t mpu_status = MPU6050_DMP_Init();
  if (mpu_status == MPU6050_OK)
  {
      printf("MPU6050 DMP Init Success!\r\n");
  }
  else
  {
      printf("MPU6050 DMP Init Failed! System will continue without MPU6050.\r\n");
  }
  printf("System Init Complete\r\n");

  // 初始化GPS定位模块
  // printf("GPS Init OK\r\n");

  // 初始化ESP32通信模块
  ESP32_COMM_Init();
  printf("ESP32 Communication Init OK\r\n");

  scheduler_init();
  // printf("Scheduler Init OK\r\n");
  // printf("System Ready!\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static uint32_t debug_timer = 0;

  while (1)
  {
    // 运行任务调度器
    scheduler_run();

    // ==================== OLED 多页面显示 ====================
    OLED_Clear();  // 清空缓冲区

    // 根据当前选中的传感器显示不同内容
    switch (current_selected_sensor) {
      case SENSOR_HR:
        // 心率血氧页面
        OLED_ShowString(1, 1, "[HR]");
        OLED_ShowString(1, 6, heart_rate_enabled ? "ON " : "OFF");

        if (heart_rate_enabled) {
          OLED_ShowString(2, 1, "HR:");
          OLED_ShowNum(2, 5, (uint32_t)max30102_data.heart_rate, 3);
          OLED_ShowString(2, 9, "bpm");

          OLED_ShowString(3, 1, "SpO2:");
          OLED_ShowNum(3, 7, (uint32_t)max30102_data.spo2, 3);
          OLED_ShowChar(3, 10, '%');
        } else {
          OLED_ShowString(3, 1, "Press KEY1");
          OLED_ShowString(4, 1, "to START");
        }
        break;

      case SENSOR_TEMP:
        // 温湿度页面
        OLED_ShowString(1, 1, "[TEMP]");
        OLED_ShowString(1, 7, temperature_enabled ? "ON " : "OFF");

        if (temperature_enabled) {
          OLED_ShowString(2, 1, "Temp:");
          OLED_ShowNum(2, 7, dht11_data.temperature, 2);
          OLED_ShowString(2, 9, "C");

          OLED_ShowString(3, 1, "Humi:");
          OLED_ShowNum(3, 7, dht11_data.humidity, 2);
          OLED_ShowString(3, 9, "%");
        } else {
          OLED_ShowString(3, 1, "Press KEY1");
          OLED_ShowString(4, 1, "to START");
        }
        break;

      case SENSOR_MPU:
        // 姿态检测页面
        OLED_ShowString(1, 1, "[MPU]");
        OLED_ShowString(1, 6, mpu6050_enabled ? "ON " : "OFF");

        if (mpu6050_enabled) {
          OLED_ShowString(2, 1, "Pitch:");
          OLED_ShowFloat(2, 7, mpu6050_data.pitch, 3, 1);

          OLED_ShowString(3, 1, "Roll:");
          OLED_ShowFloat(3, 7, mpu6050_data.roll, 3, 1);

          OLED_ShowString(4, 1, "Steps:");
          OLED_ShowNum(4, 8, mpu6050_data.step_count, 4);
        } else {
          OLED_ShowString(3, 1, "Press KEY1");
          OLED_ShowString(4, 1, "to START");
        }
        break;

      case SENSOR_MQ2:
        // 烟雾检测页面
        OLED_ShowString(1, 1, "[MQ2]");
        OLED_ShowString(1, 6, mq2_enabled ? "ON " : "OFF");

        if (mq2_enabled) {
          OLED_ShowString(2, 1, "PPM:");
          OLED_ShowFloat(2, 6, mq2_data.ppm, 4, 1);

          OLED_ShowString(3, 1, "Level:");
          OLED_ShowNum(3, 8, mq2_data.alarm, 1);

          // 显示报警状态文字
          const char* level_text[] = {"Safe", "Low", "Mid", "High"};
          if (mq2_data.alarm < 4) {
            OLED_ShowString(4, 1, (char*)level_text[mq2_data.alarm]);
          }
        } else {
          OLED_ShowString(3, 1, "Press KEY1");
          OLED_ShowString(4, 1, "to START");
        }
        break;

      case SENSOR_GPS:
        // GPS定位页面
        OLED_ShowString(1, 1, "[GPS]");
        OLED_ShowString(1, 6, gps_enabled ? "ON " : "OFF");

        if (gps_enabled) {
          OLED_ShowString(2, 1, "SAT:");
          OLED_ShowNum(2, 5, gps_data.num_satellites, 2);

          OLED_ShowString(3, 1, "Lat:");
          OLED_ShowFloat(3, 5, gps_data.latitude, 2, 4);

          OLED_ShowString(4, 1, "Lon:");
          OLED_ShowFloat(4, 5, gps_data.longitude, 3, 4);
        } else {
          OLED_ShowString(3, 1, "Press KEY1");
          OLED_ShowString(4, 1, "to START");
        }
        break;

      default:
        OLED_ShowString(2, 1, "Unknown");
        break;
    }

    // 刷新OLED显示
    OLED_Flush();

    // 主循环延时
    HAL_Delay(100);  // 每100ms刷新一次OLED

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
