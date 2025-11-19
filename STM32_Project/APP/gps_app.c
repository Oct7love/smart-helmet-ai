#include "gps_app.h"
#include "esp32_comm.h"  // 添加ESP32通信头文件

// 全局GPS数据
GPS_Data_t gps_data = {0};

#define BUUFER_SIZE 1000

// 定义环形缓冲区和接收缓冲区
ringbuffer_t usart_rb;
uint8_t usart_read_buffer[BUUFER_SIZE];

// ===== GPS的UART回调已移至 usart.c 的统一回调中处理 =====

void gps_process(unsigned char *uartReadBuff)
{
    // 查找字符串 "$GNGGA" 和 "\r\n$GNGLL"，分别表示开始和结束的标志
    char *start = strstr((const char *)uartReadBuff, "$GNGGA");
    char *end = strstr((const char *)uartReadBuff, "\r\n$GNGLL");

    // 如果没有找到这两个标志，说明没有找到有效的 GPS 数据
    if (start == NULL || end == NULL)
    {
        // 数据未准备好，静默返回
        return;
    }
    else
    {
        // 成功找到开始和结束标志，提取数据
        // 创建一个足够存储 $GNGGA 数据的缓冲区
        char gngga[100];

        // 从 uartReadBuff 中提取从 $GNGGA 到 $GNGLL 之间的数据
        strncpy(gngga, start, end - start);
        gngga[end - start] = '\0'; // 确保字符串以 NULL 结尾

        // 定义分隔符和一个数组来存储解析出的字段
        char *token;
        token = strtok(gngga, ","); // 使用逗号分隔每个字段
        char *nmea_fields[15];      // 最多支持 15 个字段
        int i = 0;

        // 逐个字段提取并存入 nmea_fields 数组中
        while (token != NULL)
        {
            nmea_fields[i] = token;
            token = strtok(NULL, ","); // 获取下一个字段
            i++;
            if (i >= 15)
                break; // 限制字段数为 15
        }

        // 如果提取到的字段大于 6，说明数据有效
        if (i > 6)
        {
            // 提取时间 (nmea_fields[1])
            if (strlen(nmea_fields[1]) > 0)
            {
                strncpy(gps_data.time, nmea_fields[1], 10);
                gps_data.time[10] = '\0';
            }

            // 解析纬度
            int lat_deg = (int)(atof(nmea_fields[2]) / 100); // 取出度数
            double lat_min = atof(nmea_fields[2]) - (lat_deg * 100); // 取出分数

            // 计算纬度
            float latitude_decimal = lat_deg + (lat_min / 60);
            gps_data.lat_dir = nmea_fields[3][0]; // N 或 S
            if (nmea_fields[3][0] == 'S') // 如果是南纬，取负
                latitude_decimal = -latitude_decimal;
            gps_data.latitude = latitude_decimal;

            // 解析经度
            int lon_deg = (int)(atof(nmea_fields[4]) / 100); // 取出度数
            float lon_min = atof(nmea_fields[4]) - (lon_deg * 100); // 取出分数

            // 计算经度
            float longitude_decimal = lon_deg + (lon_min / 60);
            gps_data.lon_dir = nmea_fields[5][0]; // E 或 W
            if (nmea_fields[5][0] == 'W') // 如果是西经，取负
                longitude_decimal = -longitude_decimal;
            gps_data.longitude = longitude_decimal;

            // 提取定位质量 (nmea_fields[6]) - 0=无效, 1=GPS单点定位
            gps_data.gps_quality = atoi(nmea_fields[6]);

            // 提取卫星数目 (nmea_fields[7])
            gps_data.num_satellites = atoi(nmea_fields[7]);

            // 提取水平稀释度 (nmea_fields[8])
            if (strlen(nmea_fields[8]) > 0)
                gps_data.hdop = atof(nmea_fields[8]);

            // 提取海拔 (nmea_fields[9])
            if (strlen(nmea_fields[9]) > 0)
                gps_data.altitude = atof(nmea_fields[9]);
        }
    }
}


void gps_task(void)
{
    // 检查是否启用了GPS导航
    extern volatile uint8_t gps_enabled;
    if (!gps_enabled) {
        return;  // 未启用，直接返回
    }

    // 如果环形缓冲区为空，直接返回
    if (ringbuffer_is_empty(&usart_rb))
        return;

    // 本次最多读取 BUUFER_SIZE 字节，避免 usart_read_buffer 溢出
    uint32_t count = usart_rb.itemCount;
    if (count > BUUFER_SIZE)
    {
        count = BUUFER_SIZE;
    }

    // 从环形缓冲区读取数据到读取缓冲区
    ringbuffer_read(&usart_rb, usart_read_buffer, count);

    // 调用 GPS 解析函数，处理接收到的数据
    gps_process((unsigned char *)usart_read_buffer);

    // 清空读取缓冲区
    memset(usart_read_buffer, 0, sizeof(usart_read_buffer));

    // 定期打印GPS状态（每1秒打印一次，方便调试）
    static uint32_t last_send_tick = 0;
    if (HAL_GetTick() - last_send_tick >= 1000) {
        // 打印GPS原始状态
        printf("[GPS] ========== GPS状态 ==========\r\n");
        printf("[GPS] 定位质量: %d (0=无效, 1=GPS单点定位)\r\n", gps_data.gps_quality);
        printf("[GPS] 卫星数量: %d\r\n", gps_data.num_satellites);

        if (gps_data.gps_quality > 0) {
            // 定位成功，发送数据到ESP32
            ESP32_COMM_SendGPS(gps_data.latitude, gps_data.longitude, gps_data.num_satellites);

            printf("[GPS] 纬度: %.6f%c\r\n", gps_data.latitude, gps_data.lat_dir);
            printf("[GPS] 经度: %.6f%c\r\n", gps_data.longitude, gps_data.lon_dir);
            printf("[GPS] 海拔: %.1fm\r\n", gps_data.altitude);
            printf("[GPS] 时间: %s (UTC)\r\n", gps_data.time);
            printf("[GPS] ✓ 定位成功！\r\n");
        } else {
            printf("[GPS] ⏳ 正在搜索卫星信号...\r\n");
        }
        printf("[GPS] ===============================\r\n");
        last_send_tick = HAL_GetTick();
    }
}