//
// Created by 13615 on 2025/11/1.
//
#include "scheduler.h"
#include "key_app.h"  // 按键处理模块

uint8_t task_num; // 全局变量存储任务数量

typedef struct {
    void (*task_func)(void); // 任务函数指针
    uint32_t rate_ms; // 任务执行周期(毫秒)
    uint32_t last_run; // 上次执行时间戳
} task_t;

void led_proc() {

}
void printf_proc() {
    printf("hello STM32\r\n");
}
// 任务调度器数组，存储所有需要调度的任务及其上次执行时间等信息
static task_t scheduler_task[] =
{
    // {printf_proc,100,0},
    {key_task,20,0},      // 按键扫描任务,20ms周期
    {mq2_task,500,0},
    {dht11_task,1000,0},  // DHT11任务,1000ms周期
    {mpu6050_task,100,0}, // MPU6050任务,100ms周期
    {max30102_task,100,0}, // MAX30102任务,100ms周期
    {gps_task,500,0}, // GPS任务,500ms周期
};

void scheduler_init(void) // 初始化调度器，计算任务数量
{
    task_num = sizeof(scheduler_task) / sizeof(task_t); // 计算任务数量
}

void scheduler_run(void) // 执行调度器，遍历所有任务并根据上次执行时间判断是否执行任务函数
{
    for (uint8_t i = 0; i < task_num; i++) // 遍历所有任务
    {
        uint32_t now_time = HAL_GetTick(); // 获取当前系统时间(毫秒)

        // 检查距离上次执行是否达到任务执行周期
        if (now_time >= scheduler_task[i].rate_ms + scheduler_task[i].last_run)
        {
            scheduler_task[i].last_run = now_time; // 更新最近执行时间为当前时间
            scheduler_task[i].task_func(); // 执行任务函数
        }
    }
}
