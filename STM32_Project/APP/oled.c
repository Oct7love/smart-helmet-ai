#include "OLED.h"
#include "oledfont.h"
#include <string.h>  // 用于 memset

/* ========== 显示缓冲区 ========== */
static uint8_t OLED_Buffer[128 * 8] = {0};  // 1024字节的显示缓冲区

/* ========== 私有函数声明 ========== */
static void OLED_I2C_Init(void);
static void OLED_I2C_Start(void);
static void OLED_I2C_Stop(void);
static void OLED_I2C_SendByte(uint8_t Byte);
static void OLED_WriteCommand(uint8_t Command);
static void OLED_WriteData(uint8_t Data);
static void OLED_SetCursor(uint8_t Y, uint8_t X);
static uint32_t OLED_Pow(uint32_t X, uint32_t Y);

/* ========== I2C延时函数(调整延时可改变I2C速度) ========== */
static void OLED_I2C_Delay(void)
{
    // 软件延时,使用volatile防止编译器优化
    volatile uint16_t i;
    for(i = 0; i < 5; i++);  // 恢复到原始的稳定值
}

/* ========== I2C引脚初始化 ========== */
static void OLED_I2C_Init(void)
{
    OLED_I2C_CLK_ENABLE(); // 使能GPIO时钟

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = OLED_SCL_PIN | OLED_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // 推挽输出(改回原来的配置)
    GPIO_InitStruct.Pull = GPIO_NOPULL;          // 推挽不需要上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(OLED_I2C_PORT, &GPIO_InitStruct);

    OLED_SCL_HIGH(); // 初始状态为高
    OLED_SDA_HIGH();
}

/* ========== I2C起始信号 ========== */
static void OLED_I2C_Start(void)
{
    OLED_SDA_HIGH();
    OLED_SCL_HIGH();
    OLED_I2C_Delay();
    OLED_SDA_LOW();  // SCL高电平时,SDA下降沿 = START
    OLED_I2C_Delay();
    OLED_SCL_LOW();
    OLED_I2C_Delay();
}

/* ========== I2C停止信号 ========== */
static void OLED_I2C_Stop(void)
{
    OLED_SDA_LOW();
    OLED_SCL_LOW();
    OLED_I2C_Delay();
    OLED_SCL_HIGH();
    OLED_I2C_Delay();
    OLED_SDA_HIGH(); // SCL高电平时,SDA上升沿 = STOP
    OLED_I2C_Delay();
}

/* ========== I2C发送字节 ========== */
static void OLED_I2C_SendByte(uint8_t Byte)
{
    for(uint8_t i = 0; i < 8; i++)
    {
        if(Byte & (0x80 >> i)) // 从高位开始发送
        {
            OLED_SDA_HIGH();
        }
        else
        {
            OLED_SDA_LOW();
        }
        OLED_I2C_Delay();
        OLED_SCL_HIGH(); // SCL上升沿,从机读取数据
        OLED_I2C_Delay();
        OLED_SCL_LOW();  // SCL下降沿,准备下一位
        OLED_I2C_Delay();
    }

    // 第9个时钟(ACK位,不处理应答)
    OLED_SDA_HIGH(); // 释放SDA
    OLED_I2C_Delay();
    OLED_SCL_HIGH();
    OLED_I2C_Delay();
    OLED_SCL_LOW();
    OLED_I2C_Delay();
}

/* ========== OLED写命令 ========== */
static void OLED_WriteCommand(uint8_t Command)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(OLED_I2C_ADDR); // 从机地址
    OLED_I2C_SendByte(0x00);          // 控制字节:写命令
    OLED_I2C_SendByte(Command);       // 命令数据
    OLED_I2C_Stop();
}

/* ========== OLED写数据 - 直接写到屏幕 ========== */
static void OLED_WriteData(uint8_t Data)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(OLED_I2C_ADDR); // 从机地址
    OLED_I2C_SendByte(0x40);          // 控制字节:写数据
    OLED_I2C_SendByte(Data);          // 显示数据
    OLED_I2C_Stop();
}

/* ========== OLED连续写多个数据 - 优化版本 ========== */
static void OLED_WriteDataBurst(uint8_t *Data, uint16_t Length)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(OLED_I2C_ADDR); // 从机地址
    OLED_I2C_SendByte(0x40);          // 控制字节:写数据

    // 一个I2C事务中连续发送所有数据
    for(uint16_t i = 0; i < Length; i++)
    {
        OLED_I2C_SendByte(Data[i]);
    }

    OLED_I2C_Stop();
}

/* ========== 设置光标位置 ========== */
static void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteCommand(0xB0 | Y);                    // 设置页地址(Y坐标)
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));    // 设置列地址高4位
    OLED_WriteCommand(0x00 | (X & 0x0F));           // 设置列地址低4位
}

/* ========== 清屏 - 现在改为清空缓冲区 ========== */
void OLED_Clear(void)
{
    memset(OLED_Buffer, 0, 128 * 8);
}

/* ========== 显示单个字符(8x16字体) - 改为写缓冲区 ========== */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    if(Char < ' ' || Char > '~') return; // 超出ASCII可见字符范围

    uint8_t Y = (Line - 1) * 2;      // 转换为页地址
    uint8_t X = (Column - 1) * 8;    // 转换为列地址

    // 写上半部分到缓冲区
    if(Y < 8 && X < 128)
    {
        for(uint8_t i = 0; i < 8; i++)
        {
            if(X + i < 128)
                OLED_Buffer[Y * 128 + X + i] = OLED_F8x16[Char - ' '][i];
        }
    }

    // 写下半部分到缓冲区
    if(Y + 1 < 8 && X < 128)
    {
        for(uint8_t i = 0; i < 8; i++)
        {
            if(X + i < 128)
                OLED_Buffer[(Y + 1) * 128 + X + i] = OLED_F8x16[Char - ' '][i + 8];
        }
    }
}

/* ========== 显示字符串 - 改为写缓冲区 ========== */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    for(uint8_t i = 0; String[i] != '\0'; i++)
    {
        OLED_ShowChar(Line, Column + i, String[i]);
    }
}

/* ========== 次方函数 ========== */
static uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while(Y--)
    {
        Result *= X;
    }
    return Result;
}

/* ========== 显示无符号数字 - 改为写缓冲区 ========== */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    for(uint8_t i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/* ========== 显示有符号数字 - 改为写缓冲区 ========== */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint32_t Number1;
    if(Number >= 0)
    {
        OLED_ShowChar(Line, Column, '+');
        Number1 = Number;
    }
    else
    {
        OLED_ShowChar(Line, Column, '-');
        Number1 = -Number;
    }
    for(uint8_t i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/* ========== 显示浮点数 - 改为写缓冲区 ========== */
void OLED_ShowFloat(uint8_t Line, uint8_t Column, float Number, uint8_t IntLen, uint8_t DecLen)
{
    uint32_t Number1;

    // 处理负数
    if(Number < 0)
    {
        OLED_ShowChar(Line, Column, '-');
        Number = -Number;
        Column++;
    }

    // 整数部分
    Number1 = (uint32_t)Number;
    OLED_ShowNum(Line, Column, Number1, IntLen);

    // 小数点
    OLED_ShowChar(Line, Column + IntLen, '.');

    // 小数部分
    Number1 = (uint32_t)((Number - (uint32_t)Number) * OLED_Pow(10, DecLen));
    OLED_ShowNum(Line, Column + IntLen + 1, Number1, DecLen);
}

/* ========== 显示十六进制数字 - 改为写缓冲区 ========== */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t SingleNumber;
    for(uint8_t i = 0; i < Length; i++)
    {
        SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
        if(SingleNumber < 10)
        {
            OLED_ShowChar(Line, Column + i, SingleNumber + '0');
        }
        else
        {
            OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
        }
    }
}

/* ========== 显示二进制数字 - 改为写缓冲区 ========== */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    for(uint8_t i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
    }
}

/* ========== OLED初始化 ========== */
void OLED_Init(void)
{
    // printf("[OLED]Starting OLED initialization...\r\n");

    HAL_Delay(100); // 上电延时
    // printf("[OLED]Power delay done\r\n");

    OLED_I2C_Init(); // 初始化I2C引脚
    // printf("[OLED]I2C pins initialized\r\n");

    // OLED初始化序列
    // printf("[OLED]Sending init commands...\r\n");

    OLED_WriteCommand(0xAE); // 关闭显示
    // printf("[OLED]0xAE sent (display off)\r\n");

    OLED_WriteCommand(0xD5); // 设置显示时钟分频比/振荡器频率
    OLED_WriteCommand(0x80);
    // printf("[OLED]0xD5/0x80 sent (clock)\r\n");

    OLED_WriteCommand(0xA8); // 设置多路复用率
    OLED_WriteCommand(0x3F);
    // printf("[OLED]0xA8/0x3F sent (mux ratio)\r\n");

    OLED_WriteCommand(0xD3); // 设置显示偏移
    OLED_WriteCommand(0x00);
    // printf("[OLED]0xD3/0x00 sent (display offset)\r\n");

    OLED_WriteCommand(0x40); // 设置显示开始行
    // printf("[OLED]0x40 sent (start line)\r\n");

    OLED_WriteCommand(0xA1); // 设置左右方向
    // printf("[OLED]0xA1 sent (segment remap)\r\n");

    OLED_WriteCommand(0xC8); // 设置上下方向
    // printf("[OLED]0xC8 sent (COM direction)\r\n");

    OLED_WriteCommand(0xDA); // 设置COM引脚硬件配置
    OLED_WriteCommand(0x12);
    // printf("[OLED]0xDA/0x12 sent (COM pins)\r\n");

    OLED_WriteCommand(0x81); // 设置对比度
    OLED_WriteCommand(0xCF);
    // printf("[OLED]0x81/0xCF sent (contrast)\r\n");

    OLED_WriteCommand(0xD9); // 设置预充电周期
    OLED_WriteCommand(0xF1);
    // printf("[OLED]0xD9/0xF1 sent (precharge)\r\n");

    OLED_WriteCommand(0xDB); // 设置VCOMH取消选择级别
    OLED_WriteCommand(0x30);
    // printf("[OLED]0xDB/0x30 sent (VCOMH)\r\n");

    OLED_WriteCommand(0xA4); // 全局显示开启
    // printf("[OLED]0xA4 sent (display all on)\r\n");

    OLED_WriteCommand(0xA6); // 设置正常/反色显示
    // printf("[OLED]0xA6 sent (normal display)\r\n");

    OLED_WriteCommand(0x8D); // 设置充电泵
    OLED_WriteCommand(0x14); // 0x14=开启,0x10=关闭
    // printf("[OLED]0x8D/0x14 sent (charge pump on)\r\n");

    OLED_WriteCommand(0xAF); // 开启显示
    // printf("[OLED]0xAF sent (display on)\r\n");

    // 清屏
    OLED_Clear();
    // printf("[OLED]Initialization complete!\r\n");
}

/* ========== 将缓冲区内容一次性刷新到屏幕 ========== */
void OLED_Flush(void)
{
    // 恢复到原始的逐字节发送方式（稳定版本）
    // 虽然速度慢，但这是经过验证能正常工作的版本
    for(uint8_t j = 0; j < 8; j++)
    {
        OLED_SetCursor(j, 0);
        for(uint8_t i = 0; i < 128; i++)
        {
            OLED_WriteData(OLED_Buffer[j * 128 + i]);
        }
    }
}
