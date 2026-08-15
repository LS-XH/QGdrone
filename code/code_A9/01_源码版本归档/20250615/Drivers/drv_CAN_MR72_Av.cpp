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

	
	/*
		改CAN ID
	*/
//	os_delay(20);
//	CanPacket mail;
//	mail.IdType = 0;
//	mail.DataLength = 8;
//	mail.FrameType = 0;
//	mail.FDFormat = 0;
//	mail.Identifier = 0x200;	//0x200+id*0x10
//	mail.data[0] = 0x82;
//	mail.data[1] = 0x00;
//	mail.data[2] = 0x00;
//	mail.data[3] = 0x00;
//	mail.data[4] = 0x01;	//id
//	mail.data[5] = 0x80;
//	mail.data[6] = 0x00;
//	mail.data[7] = 0x00;
//	driver_info.mail_box->SendMail(mail);
//	os_delay(1);

	
	//获取避障参数
	const AvoidanceCfg* avCfg = getAvCfg();
	
	/*状态机*/
		#define DIR_COUNT 4
		uint8_t targets_count[DIR_COUNT] = {0};
		uint8_t current_target[DIR_COUNT] = {0};
		#define resetState(x) {targets_count[x] = 0; current_target[x] = 0;}
		
		#define maxTargetsCount 64
		float vertical_distance[DIR_COUNT][maxTargetsCount];
		float horizontal_distance[DIR_COUNT][maxTargetsCount];
	/*状态机*/
		
	const vector2<int8_t> dirs[DIR_COUNT] = {
		vector2<int8_t>( 1, 0 ),
		vector2<int8_t>( -1, 0 ),
		vector2<int8_t>( 0, 1 ),
		vector2<int8_t>( 0, -1 ),
	};
		
	int8_t sensor_key[DIR_COUNT];
	memset( sensor_key, (int8_t)-1, sizeof(sensor_key) );
	
	while(1)
	{
		CanPacket mail;
		if( driver_info.mail_box->receiveMail( &mail, 2 ) )
		{
			uint8_t dirId = (mail.Identifier>>4) & 0xf;
			uint8_t packetId = (mail.Identifier>>0) & 0xf;
			
			if( sensor_key[dirId] < 0 )
				sensor_key[dirId] = registere_AvTarget();;
			
			if( packetId == 0xa )
			{
				targets_count[dirId] = mail.data[0];
				if( targets_count[dirId] > maxTargetsCount )
					targets_count[dirId] = maxTargetsCount;
				current_target[dirId] = 0;
				if( targets_count[dirId] == 0 )
				{
					if( sensor_key[dirId] >= 0 )
						set_AvTargetInavailable(sensor_key[dirId]);
				}
			}
			else if( packetId == 0xb )
			{
				if( current_target[dirId] < targets_count[dirId] )
				{
					uint8_t target_id = mail.data[0];
					uint8_t sector_id = (mail.data[6]>>3)&0x03;
					
					vertical_distance[dirId][current_target[dirId]] = (mail.data[1]*32 + (mail.data[2]>>3))*0.2f - 500.0f;
					horizontal_distance[dirId][current_target[dirId]] = ((mail.data[2]&0x07)*256 + 0x0C)*0.2f - 204.6f;
					++current_target[dirId];
				}
				if( current_target[dirId] >= targets_count[dirId] )
				{	//目标接收完成
					float minDistance = -1;
					if( current_target[dirId] > 0 )
					{
						for(uint8_t i=0; i<current_target[dirId]; ++i)
						{	//
								if( minDistance<0 || vertical_distance[dirId][i]<minDistance )
								{
									if( vertical_distance[dirId][i]*100 > avCfg->wheelbase[0] )
										minDistance = vertical_distance[dirId][i];
								}
						}
						
						if( minDistance >= 0 )
						{	//目标可用
							vector2<double> dis;
							dis.x = minDistance*100 * dirs[dirId].x;
							dis.y = minDistance*100 * dirs[dirId].y;
							if( sensor_key[dirId] >= 0 )
								bool res = set_AvTargetXYStraightLine_RelativeFlu( sensor_key[dirId], dis );
						}
						else if( sensor_key[dirId] >= 0 )
							set_AvTargetInavailable(sensor_key[dirId]);
					}
					resetState(dirId)
				}
			}
		}
		
		for( uint8_t i=0; i<DIR_COUNT; ++i )
		{
			AvoidanceTarget target;
			if( sensor_key[i] >= 0 )
			{
				if( get_AvTarget( sensor_key[i], &target ) )
				{
					if( target.last_update_TIME.get_pass_time() > 2 )
						set_AvTargetInavailable(sensor_key[i]);
				}
			}
		}
	}
}



static bool CAN_MR72_Av_DriverInit()
{
	return true;
}
static bool CAN_MR72_Av_DriverRun()
{
	DriverInfo* driver_info = new DriverInfo;
	CanId Ids[8];
	Ids[0].Identifier = 0x60a;	Ids[0].IdType = 0;
	Ids[1].Identifier = 0x60b;	Ids[1].IdType = 0;
	Ids[2].Identifier = 0x61a;	Ids[2].IdType = 0;
	Ids[3].Identifier = 0x61b;	Ids[3].IdType = 0;
	Ids[4].Identifier = 0x62a;	Ids[4].IdType = 0;
	Ids[5].Identifier = 0x62b;	Ids[5].IdType = 0;
	Ids[6].Identifier = 0x63a;	Ids[6].IdType = 0;
	Ids[7].Identifier = 0x63b;	Ids[7].IdType = 0;
	CanMailBox* mail_box = new CanMailBox( 5, Ids, 8 );
	driver_info->mail_box = mail_box;
	xTaskCreate( CAN_MR72_Av_Server, "CAN_MR72_Av", 1500, (void*)driver_info, SysPriority_UserTask, NULL);
	return true;
}

void init_drv_CAN_MR72_Av()
{
	CanFunc_Register( 66, CAN_MR72_Av_DriverInit, CAN_MR72_Av_DriverRun );
}