#pragma once

#include "basic.hpp"
#include "vector3.hpp"
#include <stdint.h>

// 传感器类型
enum IMU_DRIVER_TYPE
{
    IMU_DRIVER_TYPE_NULL = 0,

    IMU_DRIVER_TYPE_GYRO,
    IMU_DRIVER_TYPE_ACCEL,
    IMU_DRIVER_TYPE_MAG,
	
    IMU_DRIVER_TYPE_GYRO_ACCEL,
    IMU_DRIVER_TYPE_GYRO_ACCEL_MAG,
	
		IMU_DRIVER_TYPE_GYROACCEL,
    IMU_DRIVER_TYPE_GYROACCEL_MAG,

    IMU_DRIVER_TYPE_ALT,
};

enum IMU_DATA_STATUS
{
    IMU_DATA_STATUS_HEALTHY = 0,
	
    IMU_DATA_STATUS_ERROR = 0,
};

class IMUDriverBase
{
  public:
    virtual ~IMUDriverBase() = default;

    /*初始化驱动
      freq: 传感器运行频率 按DRIVER_TYPE顺序返回运行频率
      返回值:
        IMU_DRIVER_TYPE_NULL: 传感器初始化失败
    */
    virtual IMU_DRIVER_TYPE init() = 0;

    // 获取传感器类型
    virtual IMU_DRIVER_TYPE get_driverType() const = 0;

    // 获取传感器频率
    virtual void get_freq(uint16_t freq[]) const = 0;

    // 获取传感器名称
    virtual SName get_name() const = 0;

    // 获取参数1 imu-陀螺仪灵敏度 alt-延时s
    virtual double get_param1() const = 0;

    // 获取参数2 imu-加速度灵敏度 alt-短期信任度cm
    virtual double get_param2() const = 0;

    // 获取参数3 imu-罗盘灵敏度 alt-长期信任度cm
    virtual double get_param3() const = 0;

    /*获取数据
        sId: 传感器数据通道 按DRIVER_TYPE顺序
        rx_data: 获取的传感器数据
            imu: 三轴原始数据
            alt: y-气压mmPa z-高度mm
        temperature: 温度 <-300为不可用
        status: 数据状态
      返回值:
        true-成功 false-失败
    */
    virtual bool sample(uint8_t sId, 
			vector3<int32_t> *rx_data1, vector3<int32_t> *rx_data2, vector3<int32_t> *rx_data3, 
			double *temperature1, 
			IMU_DATA_STATUS *status1, IMU_DATA_STATUS *status2, IMU_DATA_STATUS *status3) = 0;
};