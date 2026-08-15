#pragma once
#include <stdint.h>
#include "TimeBase.hpp"

struct canBatteryStatus
{
	//厂商编号
	int16_t manufacturerID;
	//电池型号编码
	int16_t batteryTypeID;
	//电池总电压,单位（mv） 
	uint16_t batteryMainVoltage;
	//充放电电流,单位（10mA）注：正数充电，负数放电
	int16_t current;
	//电池温度,单位（1℃）  
	int16_t batteryTemp;
	//电量百分比,单位（%） 
	uint16_t batteryCapacityPercent;
	//循环计数
	uint16_t useCircleCount;
	//健康状况,单位（%）  
	int16_t health;	
	//各节电池电压,单位（mv）
	uint16_t batteryCellVoltage[14];					
	//电池设计容量,单位（mAh）
	uint16_t batteryCapacityExpected;		
	//电池剩余容量,单位（mAh）
	uint16_t batteryCapacityRemain;
	//电池错误信息,每位表示一种错误类型的状态
	uint32_t errorFlag;
	// 电池序列号
	uint8_t BatteryID[16];	
	
	// 更新时间
	TIME upDateTime;
  // 单节数量
	uint8_t cellNum;
	
}__PACKED;


// 获取电池状态
bool getTATTUBatteryStatus(canBatteryStatus& battery);
// 驱动初始化
void init_drv_CAN_BMS_TATTU();