#include "drv_YT_wasintekG1.hpp"
#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"
#include "AuxFuncs.hpp"

struct DriverInfo
{
	uint32_t param;
	Port port;
};

static inline void calcChecksum( uint8_t ptr[] )
{
	uint8_t len = ptr[1] - 1;
	uint8_t t = 0;
	uint8_t crc = 0x00;
	while(len--) 
	{
		crc ^= ptr[t++];
		for (uint8_t i=8; i>0; --i) 
		{
			if (crc & 0x80)
				crc = (crc << 1) ^ 0xD5; 
			else
				crc = (crc << 1); 
		}
	}
	len = ptr[1];
	ptr[len-1] = crc;
}

static void YT_wasintekG1_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	#define freq 20
	
	TIME pTIME, yTIME, zTIME, fTIME, sTIME, RtTIME;
	float last_fChann, last_sChann, last_RtChann;
	bool shooting_video = false;	uint8_t taking_photo = 255;	uint16_t photo_counter = 0;
	bool zooming = false;
	
	//初始化变量
	YTCtrl ty_ctrl;
	if( get_DgYTFocusCtrl(&ty_ctrl) )
		last_fChann = ty_ctrl.value;
	if( get_DgYTShootCtrl(&ty_ctrl) )
		last_sChann = ty_ctrl.value;
	if( get_DgYTReturnCtrl(&ty_ctrl) )
		last_RtChann = ty_ctrl.value;
	
	while(1)
	{
		os_delay(1.0/freq);
		
		bool RateCtrl = false;
		float yawRate = 0;	float pitRate = 0;
		if( get_DgYTPitCtrl(&ty_ctrl) )
		{	//俯仰控制
			if( ty_ctrl.update_TIME != pTIME )
			{
				pTIME = ty_ctrl.update_TIME;
				
				if( ty_ctrl.mode == 0 )
				{	//角度控制

				}
				else if( ty_ctrl.mode == 1 )
				{	//角速度控制
					RateCtrl = true;
					pitRate = -ty_ctrl.value;
				}
			}
		}
		if( get_DgYTYawCtrl(&ty_ctrl) )
		{	//偏航控制
			if( ty_ctrl.update_TIME != yTIME )
			{
				yTIME = ty_ctrl.update_TIME;
				
				if( ty_ctrl.mode == 0 )
				{	//角度控制

				}
				else if( ty_ctrl.mode == 1 )
				{	//角速度控制
					RateCtrl = true;
					yawRate = -ty_ctrl.value;
				}
			}
		}
		if( get_DgYTShootCtrl(&ty_ctrl) )
		{	//拍照录像控制
			if( ty_ctrl.update_TIME != sTIME )
			{
				sTIME = ty_ctrl.update_TIME;
				
				if( ty_ctrl.mode == 0 )
				{	//按钮上拍照 按钮下录像
					if( ty_ctrl.value > 25 )
					{
						if( taking_photo >= freq )
						{	//拍照
							uint8_t YT_cmd[] = { 0xaa, 0x05, 0x01, 0x04, 0x0a };
							calcChecksum(YT_cmd);
							driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
							
							taking_photo = 0;
						}
						else
							++taking_photo;
					}
					else if( ty_ctrl.value < -25 )
					{	//开始录像
						uint8_t YT_cmd[] = { 0xaa, 0x05, 0x01, 0x05, 0x0a };
						calcChecksum(YT_cmd);
						driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
						
						shooting_video = true;
						taking_photo = 255;
					}
					else
					{	//停止录像
						uint8_t YT_cmd[] = { 0xaa, 0x05, 0x01, 0x01, 0x0a };
						calcChecksum(YT_cmd);
						driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
							
						shooting_video = false;
						taking_photo = 255;
						photo_counter = 0;
					}
				}
				else if( ty_ctrl.mode == 1 )
				{	//通道改变拍照
					if( fabsf( ty_ctrl.value - last_sChann ) > 10 )
					{
						uint8_t YT_cmd[] = { 0xaa, 0x05, 0x01, 0x04, 0x0a };
						calcChecksum(YT_cmd);
						driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
					}
					last_sChann = ty_ctrl.value;
				}
			}
		}
		if( get_DgYTReturnCtrl(&ty_ctrl) )
		{	//归位控制
			if( ty_ctrl.update_TIME != RtTIME )
			{
				RtTIME = ty_ctrl.update_TIME;
				
				if( ty_ctrl.mode == 0 )
				{	//按钮按下归位
					if( ty_ctrl.value > 25 )
					{
						uint8_t YT_cmd[] = { 0xaa, 0x05, 0x05, 0x02, 0xa7 };
						calcChecksum(YT_cmd);
						driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
					}
				}
				else if( ty_ctrl.mode == 1 )
				{	//通道改变归位
					if( fabsf( ty_ctrl.value - last_RtChann ) > 10 )
					{
						uint8_t YT_cmd[] = { 0xaa, 0x05, 0x05, 0x02, 0xa7 };
						calcChecksum(YT_cmd);
						driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
					}
					last_RtChann = ty_ctrl.value;
				}
			}
		}
		
		if( RateCtrl )
		{	//控制角速度
			float yaw_rate=0, pit_rate=0;
			yaw_rate = remove_deadband( yawRate, 5.0f );
			pit_rate = remove_deadband( pitRate, 5.0f );
			uint8_t YT_cmd[] = { 0xaa, 0x09, 0x05, 0x06, 0x00, 0x00, 0x00, 0x11, 0x8d };
			*(int16_t*)&YT_cmd[6] = __REV16((int16_t)(pit_rate*1.0f));
			*(int16_t*)&YT_cmd[4] = __REV16((int16_t)(yaw_rate*1.0f));
			calcChecksum(YT_cmd);
			driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
		}
	}
}

static bool YT_wasintekG1_DriverInit( Port port, uint32_t param )
{
	//波特率19200
	port.SetBaudRate( 115200, 2, 2 );

	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	xTaskCreate( YT_wasintekG1_Server, "YT_wasintekG1", 812, (void*)driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_YT_wasintekG1()
{
	PortFunc_Register( 105, YT_wasintekG1_DriverInit );
}