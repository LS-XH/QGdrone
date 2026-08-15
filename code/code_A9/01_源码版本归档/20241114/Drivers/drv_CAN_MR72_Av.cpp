#include "drv_CAN_MR72_Av.hpp"
#include "drv_CAN.hpp"
#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"
#include "AuxFuncs.hpp"
#include "Avoidance.hpp"
#include "ControlSystem.hpp"

#include "drv_ADC.hpp"
#include "StorageSystem.hpp"

//Baud:500k

struct DriverInfo
{
	CanMailBox* mail_box;
};

static void CAN_MR72_Av_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;

	uint32_t sensor_key1 = registere_AvTarget();
	//获取避障参数
	const AvoidanceCfg* avCfg = getAvCfg();
	
	/*状态机*/
		uint8_t targets_count = 0;
		uint8_t current_target = 0;
		#define resetState {targets_count = 0; current_target = 0;}
		
		#define maxTargetsCount 64
		float vertical_distance[maxTargetsCount];
		float horizontal_distance[maxTargetsCount];
	/*状态机*/
	
	while(1)
	{
		CanPacket mail;
		if( driver_info.mail_box->receiveMail( &mail, 2 ) )
		{
			if( mail.Identifier == 0x60a )
			{
				targets_count = mail.data[0];
				if( targets_count > maxTargetsCount )
					targets_count = maxTargetsCount;
				current_target = 0;
				if( targets_count == 0 )
					set_AvTargetInavailable(sensor_key1);
			}
			else if( mail.Identifier == 0x60b )
			{
				if( current_target < targets_count )
				{
					uint8_t target_id = mail.data[0];
					uint8_t sector_id = (mail.data[6]>>3)&0x03;
					
					vertical_distance[current_target] = (mail.data[1]*32 + (mail.data[2]>>3))*0.2f - 500.0f;
					horizontal_distance[current_target] = ((mail.data[2]&0x07)*256 + 0x0C)*0.2f - 204.6f;
					++current_target;
				}
				if( current_target >= targets_count )
				{	//目标接收完成
					float minDistance = -1;
					if( current_target > 0 )
					{
						for(uint8_t i=0; i<current_target; ++i)
						{	//
//							if( fabsf(horizontal_distance[i]) < 5 )
//							{
								if( minDistance<0 || vertical_distance[i]<minDistance )
									minDistance = vertical_distance[i];
//							}
						}
						
						if( minDistance >= 0 )
						{	//目标可用
							Quaternion att;
							get_Airframe_quat(&att);
													vector2<double> dis;
							dis.x = minDistance*100*att.get_lean_angle_cosin();
							bool res = set_AvTargetXYStraightLine_RelativeFlu( sensor_key1, dis );
						}
						else
							set_AvTargetInavailable(sensor_key1);
					}
					resetState
				}
			}
		}
		else
			set_AvTargetInavailable(sensor_key1);
	}
}



static bool CAN_MR72_Av_DriverInit()
{
	return true;
}
static bool CAN_MR72_Av_DriverRun()
{
	DriverInfo* driver_info = new DriverInfo;
	CanId Ids[2];
	Ids[0].Identifier = 0x60a;	Ids[0].IdType = 0;
	Ids[1].Identifier = 0x60b;	Ids[1].IdType = 0;
	CanMailBox* mail_box = new CanMailBox( 5, Ids, 2 );
	driver_info->mail_box = mail_box;
	xTaskCreate( CAN_MR72_Av_Server, "CAN_MR72_Av", 1024, (void*)driver_info, SysPriority_UserTask, NULL);
	return true;
}

void init_drv_CAN_MR72_Av()
{
	CanFunc_Register( 66, CAN_MR72_Av_DriverInit, CAN_MR72_Av_DriverRun );
}