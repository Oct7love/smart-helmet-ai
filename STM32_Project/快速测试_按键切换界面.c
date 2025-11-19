/**
  ******************************************************************************
  * @file    快速测试_按键切换界面.c
  * @brief   直接复制到main.c中测试按键切换界面功能
  * @date    2025-11-11
  ******************************************************************************
  * 使用方法:
  * 1. 复制下面的代码到 main.c 对应位置
  * 2. 连接PC8和PC9按键(一端接GPIO,另一端接GND)
  * 3. 编译烧录
  * 4. 按下PC9(下一页)或PC8(上一页)测试
  ******************************************************************************
  */

/* ========== 步骤1: 在main.c头部添加头文件 ========== */

/* USER CODE BEGIN Includes */
#include "bsp_system.h"
#include "ui_manager.h"  // 添加UI管理系统头文件
/* USER CODE END Includes */


/* ========== 步骤2: 在main()函数中初始化UI系统 ========== */

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  
  /* Configure the system clock */
  SystemClock_Config();
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  
  /* USER CODE BEGIN 2 */
  
  // 初始化延时函数
  delay_init();
  
  // 初始化OLED
  OLED_Init();
  printf("OLED Init OK\r\n");
  
  // 初始化传感器(根据实际需求选择)
  // MPU6050_DMP_Init();   // MPU6050初始化
  // DHT11_Init();         // DHT11初始化(如果有)
  // MAX30102_Init();      // MAX30102初始化(如果有)
  // MQ2_Init();           // MQ2初始化(如果有)
  
  // 初始化调度器(如果使用)
  scheduler_init();
  printf("Scheduler Init OK\r\n");
  
  // 【重点】初始化UI管理系统(含PC8/PC9按键GPIO配置)
  UI_Init();
  printf("UI Manager Init OK\r\n");
  
  printf("System Ready! Press PC8(上一页) or PC9(下一页)\r\n");
  
  /* USER CODE END 2 */
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 【重点】调用UI任务(按键扫描+界面刷新)
    UI_Task();
    
    // 运行其他任务调度器(如果有)
    scheduler_run();
    
    // 小延迟(推荐50ms)
    HAL_Delay(50);
    
    /* USER CODE END WHILE */
  }
  /* USER CODE END 3 */
}


/* ========== 可选: 在scheduler.c中集成(更优雅的方式) ========== */

// 如果您使用调度器,可以在 scheduler.c 中添加UI任务:

/*
// 在scheduler.c顶部添加
#include "ui_manager.h"

static uint32_t last_time_ui = 0;  // UI任务时间戳

// 在scheduler_init()中添加
void scheduler_init(void)
{
    printf("Scheduler Init...\r\n");
    
    UI_Init();  // 初始化UI管理系统
    
    // ... 其他任务初始化 ...
    
    printf("Scheduler Init Complete\r\n");
}

// 在scheduler_run()中添加
void scheduler_run(void)
{
    uint32_t current_time = HAL_GetTick();
    
    // UI任务(50ms周期)
    if (current_time - last_time_ui >= 50) {
        UI_Task();
        last_time_ui = current_time;
    }
    
    // ... 其他任务 ...
}
*/


/* ========== 预期效果 ========== */

/*
串口输出:
---------
OLED Init OK
Scheduler Init OK
UI Manager Init OK (PC8=上一页, PC9=下一页)
System Ready! Press PC8(上一页) or PC9(下一页)

Key PC9: 下一页
下一页 -> 界面1

Key PC9: 下一页
下一页 -> 界面2

Key PC8: 上一页
上一页 -> 界面1


OLED显示:
---------
界面0: [MPU6050] 1/6 + 三轴加速度
界面1: [DHT11] 2/6 + 温湿度
界面2: [MAX30102] 3/6 + 心率血氧数据
界面3: [PPG] 4/6 + 实时波形
界面4: [MQ2] 5/6 + 烟雾传感器
界面5: [SYS] 6/6 + 系统信息

按键操作:
---------
PC9: 下一页(1→2→3→4→5→6→1循环)
PC8: 上一页(6→5→4→3→2→1→6循环)
*/


/* ========== 故障排查 ========== */

// 问题1: 按键无响应
// 解决: 
// - 检查PC8/PC9接线(一端接GPIO,另一端接GND)
// - 用万用表测量按下时PC8/PC9引脚电压(应为0V)
// - 确认UI_Init()已调用

// 问题2: 界面不刷新
// 解决:
// - 确认UI_Task()在主循环中周期调用
// - 检查OLED_Init()是否成功
// - 查看串口是否有"切换到界面X"的输出

// 问题3: 编译错误
// 解决:
// - 确认CMakeLists.txt中已添加ui_manager.c
// - 包含所有依赖的头文件(ui_manager.h, bsp_system.h等)
// - 重新运行cmake配置: cd build && cmake .. -G Ninja

// 问题4: 某个界面显示为0
// 解决:
// - 检查对应传感器是否已初始化
// - 如MPU6050未初始化,界面0会显示0.00
// - 传感器初始化应在UI_Init()之前完成


/* ========== 高级功能示例 ========== */

// 示例1: 手动跳转到指定界面
void JumpToPage(uint8_t page)
{
    if (page < 6) {
        UI_SetPage((UI_Page_t)page);
        printf("跳转到界面%d\r\n", page);
    }
}

// 示例2: 根据串口命令切换界面
// 在UART接收中断中添加:
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    extern uint8_t rx_buffer[1];
    
    switch (rx_buffer[0]) {
        case '0': UI_SetPage(UI_PAGE_MPU6050); break;
        case '1': UI_SetPage(UI_PAGE_DHT11); break;
        case '2': UI_SetPage(UI_PAGE_MAX30102_DATA); break;
        case '3': UI_SetPage(UI_PAGE_MAX30102_PPG); break;
        case '4': UI_SetPage(UI_PAGE_MQ2); break;
        case '5': UI_SetPage(UI_PAGE_SYSTEM_INFO); break;
        case 'n': UI_NextPage(); break;  // 下一页
        case 'p': UI_PrevPage(); break;  // 上一页
    }
    
    HAL_UART_Receive_IT(huart, rx_buffer, 1);
}
*/

// 示例3: 定时自动轮播
void AutoRotateDemo(void)
{
    static uint32_t last_rotate = 0;
    
    // 每3秒自动切换到下一页
    if (HAL_GetTick() - last_rotate > 3000) {
        UI_NextPage();
        last_rotate = HAL_GetTick();
    }
}


/* ========== 测试清单 ========== */

/*
[ ] 1. 按下PC9,界面从0切换到1
[ ] 2. 继续按PC9,依次切换到2,3,4,5,6,再回到0(循环)
[ ] 3. 按下PC8,界面从0切换到6(反向)
[ ] 4. 继续按PC8,依次切换到5,4,3,2,1,再回到0(反向循环)
[ ] 5. 快速连按PC9,每次都正确切换,无误触发
[ ] 6. 界面0显示MPU6050加速度(如果已初始化)
[ ] 7. 界面2显示MAX30102心率血氧(如果已初始化)
[ ] 8. 界面3显示PPG波形动态滚动(如果已初始化)
[ ] 9. 串口输出"Key PC9: 下一页"等调试信息
[ ] 10. 无界面闪烁或卡顿现象
*/


/* ========== 完成! ========== */

// ✅ 复制以上代码到main.c对应位置
// ✅ 连接PC8和PC9按键到GND
// ✅ 编译烧录测试
// ✅ 查看按键切换界面_使用说明.md获取更多帮助

// 祝您测试顺利! 🎉

