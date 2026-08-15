#pragma once
#include "AuxFuncs.hpp"

struct GpsDAOConfig
{
	/*搜星GNSS设置
		0:不变
		63:GPS+SBAS+Galileo+BeiDou+IMES+QZSS
		119:GPS+SBAS+Galileo+IMES+QZSS+GLONASS
	*/
	uint8_t GNSS_Mode[8];
	
	//延时时间
	float delay[2];
	
	//双天线向量（前左上坐标系）
	float DRTK_VecX[2];
	float DRTK_VecY[2];
	float DRTK_VecZ[2];
};

void init_drv_RTK_DAO_Base();