#pragma once

#include <stdint.h>

//最大gps个数
#define MAX_GPS_COUNT 3

// 使用utc时间设置本地rtc时钟(同一次启动周期只执行一次)
bool gps_rtcUpdated();
bool gps_setRtcOnce(double lat, double lon, uint16_t utc_year, uint8_t utc_month, uint8_t utc_day,
    uint8_t utc_hour, uint8_t utc_minute, uint8_t utc_second);

// utc时间打包成64bit数据
uint64_t gps_packUtc(uint16_t utc_year, uint8_t utc_month, uint8_t utc_day,
    uint8_t utc_hour, uint8_t utc_minute, double utc_second);

//gps设置
struct GpsConfig
{
    // 搜星GNSS设置
    uint32_t GNSS_Mode[2];

    // 延时时间
    float delay[2];

    // 双天线参数
    float daoVecX[2];
    float daoVecY[2];
    float daoVecZ[2];
};