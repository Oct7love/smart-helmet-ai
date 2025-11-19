#ifndef GPS_APP_H
#define GPS_APP_H

#include "bsp_system.h"

// GPS数据结构体
typedef struct {
    char time[11];           // UTC时间 HH:MM:SS.SS
    float latitude;          // 纬度（十进制度数）
    float longitude;         // 经度（十进制度数）
    char lat_dir;            // 纬度方向 N/S
    char lon_dir;            // 经度方向 E/W
    uint8_t gps_quality;     // 定位质量 (0=无效, 1=GPS)
    uint8_t num_satellites;  // 卫星数目
    float hdop;              // 水平稀释度
    float altitude;          // 海拔高度
} GPS_Data_t;

// 全局GPS数据
extern GPS_Data_t gps_data;

void gps_task();
void gps_process(unsigned char *uartReadBuff);
#endif
