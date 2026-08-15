#include "drv_CAN_QOAR1271_Av.hpp"
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

static void CAN_QOAR1271_Av_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;

	uint32_t sensor_key1 = registere_AvTarget();
	//获取避障参数
	const AvoidanceCfg* avCfg = getAvCfg();
	
	while(1)
	{
		CanPacket mail;
		if( driver_info.mail_box->receiveMail( &mail, 2 ) )
		{
			
			if(mail.Identifier==0x1C55AA00)
			{	//
				struct target_info_packet
				{
					//保留
					uint16_t rsv;
					//扇区数据
					uint16_t sectors[3];
				}__PACKED;
				target_info_packet* packet = (target_info_packet*)&mail.data;
				
				if( packet->sectors[1]>avCfg->wheelbase[0] && packet->sectors[1]<65500 ) {
					vector2<double> dis;
					dis.x = packet->sectors[1];
					set_AvTargetXYStraightLine_RelativeFlu( sensor_key1, dis );
				}
				else
					set_AvTargetInavailable(sensor_key1);
			}
		}
		else
			set_AvTargetInavailable(sensor_key1);
	}
}



static bool CAN_QOAR1271_Av_DriverInit()
{
	return true;
}
static bool CAN_QOAR1271_Av_DriverRun()
{
	DriverInfo* driver_info = new DriverInfo;
	CanId Ids[1];
	Ids[0].Identifier = 0x1C55AA00;	Ids[0].IdType = 1;
	CanMailBox* mail_box = new CanMailBox( 5, Ids, 1 );
	driver_info->mail_box = mail_box;
	xTaskCreate( CAN_QOAR1271_Av_Server, "CAN_QOAR1271_Av", 800, (void*)driver_info, SysPriority_UserTask, NULL);
	return true;
}

void init_drv_CAN_QOAR1271_Av()
{
	CanFunc_Register( 65, CAN_QOAR1271_Av_DriverInit, CAN_QOAR1271_Av_DriverRun );
}