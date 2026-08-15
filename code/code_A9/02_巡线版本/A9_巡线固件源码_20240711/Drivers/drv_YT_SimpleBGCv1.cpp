#include "drv_YT_SimpleBGCv1.hpp"
#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"
#include "AuxFuncs.hpp"
#include "SBGC.h"
#include "Commulink.hpp"

struct DriverInfo
{
	uint32_t param;
	Port port;
};

static inline uint16_t CalcCheckSum( uint8_t* buf )
{
	buf[3] = buf[1] + buf[2];
	uint8_t size = buf[2];
	uint8_t sum = 0;
	for( uint8_t i=4; i<4+size; ++i )
		sum += buf[i];
	buf[4+size] = sum;
	return 5+size;
}

static void YT_SimpleBGCv1_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	#define freq 50
	double h = 1.0 / freq;
	
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
	
	SBGC_Parser sbgc;
	sbgc.init(&driver_info.port);
	SBGC_cmd_control_t c = { 0, 0, 0, 0, 0, 0, 0 };
	while(1)
	{
		os_delay(h);
	
//		c.mode = SBGC_CONTROL_MODE_ANGLE;
//		c.anglePITCH = SBGC_DEGREE_TO_ANGLE(60);
//		SBGC_cmd_control_send(c, sbgc);
//		os_delay(5);
//		c.anglePITCH = SBGC_DEGREE_TO_ANGLE(-60);
//		SBGC_cmd_control_send(c, sbgc);
//		os_delay(5);
		
		if( get_DgYTPitCtrl(&ty_ctrl) )
		{	//俯仰控制
			if( ty_ctrl.update_TIME != pTIME )
			{
				pTIME = ty_ctrl.update_TIME;
				
				if( ty_ctrl.mode == 0 )
				{	//角度控制
					c.mode = SBGC_CONTROL_MODE_ANGLE;
					c.anglePITCH = SBGC_DEGREE_TO_ANGLE(ty_ctrl.value);
					SBGC_cmd_control_send(c, sbgc);
				}
				else if( ty_ctrl.mode == 1 )
				{	//角速度控制
					c.mode = SBGC_CONTROL_MODE_ANGLE;
					c.anglePITCH += SBGC_DEGREE_TO_ANGLE(ty_ctrl.value*h);
					SBGC_cmd_control_send(c, sbgc);
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
					c.mode = SBGC_CONTROL_MODE_ANGLE;
					c.angleYAW = SBGC_DEGREE_TO_ANGLE(ty_ctrl.value);
					SBGC_cmd_control_send(c, sbgc);
				}
				else if( ty_ctrl.mode == 1 )
				{	//角速度控制
					c.mode = SBGC_CONTROL_MODE_ANGLE;
					c.angleYAW += SBGC_DEGREE_TO_ANGLE(ty_ctrl.value*h);
					SBGC_cmd_control_send(c, sbgc);
				}
			}
		}
//		if( get_DgYTZoomCtrl(&ty_ctrl) )
//		{	//倍数控制
//			if( ty_ctrl.update_TIME != zTIME )
//			{
//				zTIME = ty_ctrl.update_TIME;
//				
//				if( ty_ctrl.mode == 0 )
//				{	//倍数控制
//					
//				}
//				else if( ty_ctrl.mode == 1 )
//				{	//放大缩小控制
//					uint8_t zoom = 0;
//					if( ty_ctrl.value > 10 )
//						zoom = 0x6;
//					else if( ty_ctrl.value < -10 )
//						zoom = 0x5;
//					
//					if( zoom )
//					{
//						YT_cmd[2] = zoom;
//						driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
//						zooming = true;
//					}
//					else if( zooming )
//					{
//						YT_cmd[2] = 0x7;
//						driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
//						zooming = false;
//					}
//				}
//			}
//		}
//		if( get_DgYTFocusCtrl(&ty_ctrl) )
//		{	//聚焦控制
//			if( ty_ctrl.update_TIME != fTIME )
//			{
//				fTIME = ty_ctrl.update_TIME;
//				
//				if( ty_ctrl.mode == 0 )
//				{	//按钮按下聚焦
//					if( ty_ctrl.value > 25 )
//					{
//						
//					}
//				}
//				else if( ty_ctrl.mode == 1 )
//				{	//通道改变聚焦
//					if( fabsf( ty_ctrl.value - last_fChann ) > 10 )
//					{
//						
//					}
//					last_fChann = ty_ctrl.value;
//				}
//			}
//		}
//		if( get_DgYTShootCtrl(&ty_ctrl) )
//		{	//拍照录像控制
//			if( ty_ctrl.update_TIME != sTIME )
//			{
//				sTIME = ty_ctrl.update_TIME;
//				
//				if( ty_ctrl.mode == 0 )
//				{	//按钮上拍照 按钮下录像
//					if( ty_ctrl.value > 25 )
//					{
//						if( taking_photo >= freq )
//						{
//							YT_cmd[2] = 0x0E;
//							driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
//							taking_photo = 0;
//						}
//						else
//							++taking_photo;
//					}
//					else if( ty_ctrl.value < -25 )
//					{
//						YT_cmd[2] = 0x0C;
//						driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
//						shooting_video = true;
//						taking_photo = 255;
//					}
//					else
//					{
//						if( shooting_video )
//						{
//							YT_cmd[2] = 0x0D;
//							driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
//						}
//						shooting_video = false;
//						taking_photo = 255;
//						photo_counter = 0;
//					}
//				}
//				else if( ty_ctrl.mode == 1 )
//				{	//通道改变拍照
//					if( fabsf( ty_ctrl.value - last_sChann ) > 10 )
//					{
//						YT_cmd[2] = 0x0E;
//						driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
//					}
//					last_sChann = ty_ctrl.value;
//				}
//			}
//		}
		if( get_DgYTReturnCtrl(&ty_ctrl) )
		{	//归位控制
			if( ty_ctrl.update_TIME != RtTIME )
			{
				RtTIME = ty_ctrl.update_TIME;
				
				if( ty_ctrl.mode == 0 )
				{	//按钮按下归位
					if( ty_ctrl.value > 25 )
					{
						c.mode = SBGC_CONTROL_MODE_ANGLE;
						c.anglePITCH = 0;
						c.angleROLL = 0;
						c.angleYAW = 0;
						SBGC_cmd_control_send(c, sbgc);
					}
				}
				else if( ty_ctrl.mode == 1 )
				{	//通道改变归位
					if( fabsf( ty_ctrl.value - last_RtChann ) > 10 )
					{
						c.mode = SBGC_CONTROL_MODE_ANGLE;
						c.anglePITCH = 0;
						c.angleROLL = 0;
						c.angleYAW = 0;
						SBGC_cmd_control_send(c, sbgc);
					}
					last_RtChann = ty_ctrl.value;
				}
			}
		}
	}
}

static bool YT_SimpleBGCv1_DriverInit( Port port, uint32_t param )
{
	//波特率19200
	port.SetBaudRate( 115200, 2, 2 );

	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	xTaskCreate( YT_SimpleBGCv1_Server, "YT_SimpleBGCv1", 812, (void*)driver_info, SysPriority_UserTask, NULL);
	return true;
}

void init_drv_YT_SimpleBGCv1()
{
	PortFunc_Register( 103, YT_SimpleBGCv1_DriverInit );
}