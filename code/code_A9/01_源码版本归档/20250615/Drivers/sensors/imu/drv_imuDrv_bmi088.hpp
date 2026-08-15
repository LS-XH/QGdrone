#pragma once

#include "IMUDriverBase.hpp"

class imuDrv_bmi088 : public IMUDriverBase
{
  public:
    imuDrv_bmi088();

    IMU_DRIVER_TYPE init();
    bool sample(uint8_t sId,
        vector3<int32_t> *rx_data1, vector3<int32_t> *rx_data2, vector3<int32_t> *rx_data3,
        double *temperature,
        IMU_DATA_STATUS *status1, IMU_DATA_STATUS *status2, IMU_DATA_STATUS *status3);

    IMU_DRIVER_TYPE get_driverType() const
    {
        return driver_type;
    }

    void get_freq(uint16_t freq[]) const
    {
        freq[0] = freqGyro;
        freq[1] = freqAccel;
    }

    SName get_name() const
    {
        return name;
    }

    double get_param1() const
    {
        return gyro_sensitivity;
    }
    double get_param2() const
    {
        return accel_sensitivity;
    }
    double get_param3() const
    {
        return 0;
    }

  private:
    IMU_DRIVER_TYPE driver_type;
    uint16_t freqGyro, freqAccel;
    SName name;

    double gyro_sensitivity;
    double accel_sensitivity;
};