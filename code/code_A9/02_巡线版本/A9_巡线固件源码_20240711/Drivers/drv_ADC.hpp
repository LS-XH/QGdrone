#pragma once

/*
  获取电池电流大小,单位：A
*/
float adcGet_MainBaterry1_Current();

/*
  获取电池电压,单位；V
*/
float adcGet_MainBaterry1_Voltage();

/*
  获取VDDA基准电压,单位；V
*/
float adcGet_VDDA_Voltage();

/*
  获取温度，单位：℃
*/
float adcGet_CPUTemperature();

void init_drv_ADC(void);