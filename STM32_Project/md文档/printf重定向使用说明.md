# STM32 Printf 重定向使用说明

## ✅ 已完成配置

### 1. 重定向实现文件
- **文件位置**: `Core/Src/retarget.c`
- **功能**: 将 `printf` 输出重定向到 USART1

### 2. 使用方法

#### 在代码中直接使用 printf:
```c
#include <stdio.h>

printf("Hello STM32!\r\n");
printf("Number: %d\r\n", 123);
printf("Float: %.2f\r\n", 3.14);
```

#### 在 main.c 中已添加测试代码:
```c
printf("STM32 Printf Redirection Test\r\n");
printf("System Clock: %lu Hz\r\n", HAL_RCC_GetHCLKFreq());
```

### 3. 硬件连接
- **UART**: USART1
- **波特率**: 115200
- **TX引脚**: PA9
- **RX引脚**: PA10

### 4. 串口调试工具设置
使用任意串口调试助手(如PuTTY、SecureCRT、串口助手等):
- 波特率: **115200**
- 数据位: **8**
- 停止位: **1**
- 校验位: **无**
- 流控: **无**

### 5. 编译结果
✅ 编译成功
- RAM 使用: 2096 B / 48 KB (4.26%)
- FLASH 使用: 12984 B / 256 KB (4.95%)

---

## 🎯 方法2: 使用微库(可选,节省空间)

如果需要减小代码体积,可以启用微库(MicroLIB):

### 在 CMakeLists.txt 中添加编译选项:
```cmake
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    --specs=nano.specs  # 使用nano库,减小代码体积
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    --specs=nano.specs
    -u _printf_float  # 如果需要支持浮点数printf
)
```

---

## 🔧 常见问题

### Q1: printf 没有输出?
- 检查 UART 是否初始化: `MX_USART1_UART_Init();`
- 检查串口工具波特率是否为 115200
- 检查硬件连接是否正确

### Q2: 输出乱码?
- 检查波特率设置是否一致
- 检查系统时钟配置是否正确

### Q3: 想使用其他 UART?
修改 `retarget.c` 中的 `&huart1` 为其他 UART 句柄,如 `&huart2`

---

## 📝 注意事项

1. **换行符**: 使用 `\r\n` 而不是 `\n`,确保在串口工具中正确换行
2. **阻塞模式**: 当前使用阻塞发送,大量数据可能影响实时性
3. **中断模式**: 如需高性能,可改用 `HAL_UART_Transmit_IT()` 或 DMA
4. **浮点数**: 默认支持浮点数输出,如需节省空间可禁用

---

## 🚀 高级用法

### 使用 DMA 提升性能(推荐):
```c
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&ch, 1);
    return ch;
}
```

### 添加缓冲区:
```c
#define PRINTF_BUF_SIZE 256
static uint8_t printf_buf[PRINTF_BUF_SIZE];
static uint16_t printf_idx = 0;

void printf_flush(void)
{
    if(printf_idx > 0) {
        HAL_UART_Transmit(&huart1, printf_buf, printf_idx, 0xFFFF);
        printf_idx = 0;
    }
}
```
