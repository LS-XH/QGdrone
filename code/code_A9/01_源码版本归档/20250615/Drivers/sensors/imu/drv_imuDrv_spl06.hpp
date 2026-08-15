#pragma once

#include "IMUDriverBase.hpp"

class imuDrv_spl06 : public IMUDriverBase
{
  public:
    imuDrv_spl06();

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
        freq[0] = this->freq;
    }

    SName get_name() const
    {
        return name;
    }

    double get_param1() const
    {
        return delay;
    }
    double get_param2() const
    {
        return trust;
    }
    double get_param3() const
    {
        return lt_trust;
    }

  private:
    IMU_DRIVER_TYPE driver_type;
    uint16_t freq;
    double delay, trust, lt_trust;
    SName name;

    struct COEFFICIENTS
    {
        int16_t c0;
        int16_t c1;
        int32_t c00;
        int32_t c10;
        int16_t c01;
        int16_t c11;
        int16_t c20;
        int16_t c21;
        int16_t c30;
        double KP;
        double KT;
    };
    COEFFICIENTS coefficients;

    void spl06_pressure_rateset(uint8_t u8SmplRate, uint8_t u8OverSmpl);
    void spl06_temperature_rateset(uint8_t u8SmplRate, uint8_t u8OverSmpl);
};