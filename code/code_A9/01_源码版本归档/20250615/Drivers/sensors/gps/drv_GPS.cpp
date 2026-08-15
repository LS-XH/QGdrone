#include "Basic.hpp"
#include "gpsConfig.hpp"
#include "SensorsBackend.hpp"
#include "Parameters.hpp"
#include "drv_GPS.hpp"

#include "drv_GPS1_Ublox.hpp"
#include "drv_GPS1_UM982.hpp"
#include "drv_GPS1_UM482.hpp"

static bool rtc_updated = false;
bool gps_rtcUpdated()
{
    return rtc_updated;
}
bool gps_setRtcOnce(double lat, double lon, uint16_t utc_year, uint8_t utc_month, uint8_t utc_day,
    uint8_t utc_hour, uint8_t utc_minute, uint8_t utc_second)
{
    if (!rtc_updated)
    {
        rtc_updated = true;
			
        int8_t TimeZone = GetTimeZone(lat, lon);
				RTC_TimeStruct rtc;
        UTC2LocalTime(&rtc, utc_year, utc_month, utc_day, utc_hour, utc_minute, utc_second, TimeZone, 0);
        Set_RTC_Time(&rtc);	
        return true;
    }
    return false;
}

uint64_t gps_packUtc(uint16_t utc_year, uint8_t utc_month, uint8_t utc_day,
    uint8_t utc_hour, uint8_t utc_minute, double utc_second)
{
    return ((uint64_t)(utc_year - 1990) << 56) | ((uint64_t)utc_month << 52) | ((uint64_t)utc_day << 46) |
           ((uint64_t)utc_hour << 40) | ((uint64_t)utc_minute << 32) | ((uint64_t)utc_second << 24) |
           ((uint64_t)(utc_second * 1e6) % (uint64_t)1e6);
}

void init_drv_GPS()
{
    // 注册GPS参数
    GpsConfig initial_cfg;
    // 搜星GNSS设置
    initial_cfg.GNSS_Mode[0] = 0;
    // 延时时间
    initial_cfg.delay[0] = 0.1;
    // 双天线参数
    initial_cfg.daoVecX[0] = 0;
    initial_cfg.daoVecY[0] = 0;
    initial_cfg.daoVecZ[0] = 0;

    MAV_PARAM_TYPE param_types[] = {
        // 搜星GNSS设置
        MAV_PARAM_TYPE_UINT32,
        // 延时时间
        MAV_PARAM_TYPE_REAL32,
        // 双天线参数
        MAV_PARAM_TYPE_REAL32,
        MAV_PARAM_TYPE_REAL32,
        MAV_PARAM_TYPE_REAL32,
    };

    SName param_names1[] = {
        // 搜星GNSS设置
        "GPS1_GNSS",
        // 延时时间
        "GPS1_delay",
        // 双天线参数
        "GPS1_daoVecX",
        "GPS1_daoVecY",
        "GPS1_daoVecZ",
    };
    ParamGroupRegister("GPS1Cfg", 2, sizeof(initial_cfg) / 8, param_types, param_names1, (uint64_t *)&initial_cfg);

    SName param_names2[] = {
        // 搜星GNSS设置
        "GPS2_GNSS",
        // 延时时间
        "GPS2_delay",
        // 双天线参数
        "GPS2_daoVecX",
        "GPS2_daoVecY",
        "GPS2_daoVecZ",
    };
    ParamGroupRegister("GPS2Cfg", 2, sizeof(initial_cfg) / 8, param_types, param_names2, (uint64_t *)&initial_cfg);

    SName param_names3[] = {
        // 搜星GNSS设置
        "GPS3_GNSS",
        // 延时时间
        "GPS3_delay",
        // 双天线参数
        "GPS3_daoVecX",
        "GPS3_daoVecY",
        "GPS3_daoVecZ",
    };
    ParamGroupRegister("GPS3Cfg", 2, sizeof(initial_cfg) / 8, param_types, param_names3, (uint64_t *)&initial_cfg);

    // 初始化外设驱动
    init_drv_GPS1_Ublox();
    init_drv_GPS1_UM982();
		init_drv_GPS1_UM482();
}