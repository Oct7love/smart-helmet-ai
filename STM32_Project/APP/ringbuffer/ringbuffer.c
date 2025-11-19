#include "ringbuffer.h"

// 初始化环形缓冲区
void ringbuffer_init(ringbuffer_t *rb)
{
    // 设置读指针和写指针初始值为0
    rb->r = 0;
    rb->w = 0;
    // 清空缓冲区内存
    memset(rb->buffer, 0, sizeof(uint8_t) * RINGBUFFER_SIZE);
    // 初始化项目数量为0
    rb->itemCount = 0;
}

// 检查环形缓冲区是否已满
uint8_t ringbuffer_is_full(ringbuffer_t *rb)
{
    // 如果项目数量等于缓冲区大小，返回1表示已满，否则返回0表示未满
    return (rb->itemCount == RINGBUFFER_SIZE);
}

// 检查环形缓冲区是否为空
uint8_t ringbuffer_is_empty(ringbuffer_t *rb)
{
    // 如果项目数量为0，返回1表示空，否则返回0表示非空
    return (rb->itemCount == 0);
}

// 向环形缓冲区写入数据
int8_t ringbuffer_write(ringbuffer_t *rb, uint8_t *data, uint32_t num)
{
    // 写入长度为 0，直接返回
    if (num == 0)
        return 0;

    // 若当前已满，直接返回错误
    if (rb->itemCount >= RINGBUFFER_SIZE)
        return -1;

    // 根据剩余空间限制本次实际写入长度，避免越界
    uint32_t free = RINGBUFFER_SIZE - rb->itemCount;
    if (num > free)
        num = free;

    // 循环写入数据到缓冲区
    while (num--)
    {
        rb->buffer[rb->w] = *data++;              // 写入数据并移动写指针
        rb->w = (rb->w + 1) % RINGBUFFER_SIZE;    // 写指针循环移动
        rb->itemCount++;                          // 增加项目数量
    }

    return 0;  // 写入成功返回 0
}

// 从环形缓冲区读取数据
int8_t ringbuffer_read(ringbuffer_t *rb, uint8_t *data, uint32_t num)
{
    // 读取长度为 0，直接返回
    if (num == 0)
        return 0;

    // 如果缓冲区为空，返回 -1
    if (ringbuffer_is_empty(rb))
        return -1;

    // 不能读取超过当前已有的数据量
    if (num > rb->itemCount)
        num = rb->itemCount;

    // 从缓冲区循环读取数据
    while (num--)
    {
        *data++ = rb->buffer[rb->r];              // 读取数据并移动读指针
        rb->r = (rb->r + 1) % RINGBUFFER_SIZE;    // 读指针循环移动
        rb->itemCount--;                          // 减少项目数量
    }
    return 0;  // 读取成功返回 0
}