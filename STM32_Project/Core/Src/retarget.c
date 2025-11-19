/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    retarget.c
  * @brief   printf重定向到UART (GCC版本)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "usart.h"
#include <stdio.h>
#include <errno.h>
#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO

/* GCC需要实现_write函数来重定向printf */
int _write(int file, char *ptr, int len)
{
    if(file == STDOUT_FILENO || file == STDERR_FILENO) {
        // 使用100ms超时，避免永久阻塞
        if (HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 100) == HAL_OK) {
            return len;
        } else {
            return 0;  // 超时返回0，不阻塞程序
        }
    }
    errno = EIO;
    return -1;
}

/* 可选:实现_read函数来支持scanf */
int _read(int file, char *ptr, int len)
{
    if(file == STDIN_FILENO) {
        HAL_UART_Receive(&huart1, (uint8_t*)ptr, 1, HAL_MAX_DELAY);
        return 1;
    }
    errno = EIO;
    return -1;
}
