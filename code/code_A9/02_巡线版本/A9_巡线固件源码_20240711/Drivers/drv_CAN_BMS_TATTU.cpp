#include "drv_CAN_BMS_TATTU.hpp"
#include "drv_CAN.hpp"
#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"
#include "AuxFuncs.hpp"

struct DriverInfo
{
	CanMailBox* mail_box;
	uint32_t sensor_key;
};

/***** 校验 ******/
	#define CRC_CCITT_INIT 0xFFFF
	#define CRC_CCITT_POLY 0x1021U
	uint16_t CCITT_CRC16=0xFFFF;
	static void CCITT_CRC_ARRAY(uint8_t const * bytes, uint16_t len);
	void CCITT_CRC16Init(uint8_t const * bytes, uint16_t len)
	{
		CCITT_CRC16 = CRC_CCITT_INIT;
		CCITT_CRC_ARRAY(bytes,len);
	}
	void CCITT_CRCStep(uint8_t byte)
	{
		uint32_t j;
		CCITT_CRC16 ^= ((uint16_t)byte <<8);
		for (j = 0; j < 8; j++)
		{
			CCITT_CRC16=(CCITT_CRC16 & 0x8000U)?((CCITT_CRC16 << 1) ^ CRC_CCITT_POLY):(CCITT_CRC16<< 1);
		}
	}
	void CCITT_CRC_ARRAY(uint8_t const * bytes, uint16_t len)
	{
		while (len--)  CCITT_CRCStep(*bytes++);
	}
/***** 校验 ******/
	
	
static canBatteryStatus battery1;
// 获取电池状态
bool getTATTUBatteryStatus(canBatteryStatus& battery)
{
	if( battery1.upDateTime.get_pass_time() < 1 )
	{
		battery  = battery1;
		return true;
	}
	return false;
}
	
static void CAN_Radar_BMS_TATTU_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	
	struct target_info_packet_header
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
	}__PACKED;
	struct target_info_packet_tail
	{
		//电池设计容量,单位（mAh）
		uint16_t batteryCapacityExpected;		
		//电池剩余容量,单位（mAh）
		uint16_t batteryCapacityRemain;
		//电池错误信息,每位表示一种错误类型的状态
		uint32_t errorFlag;
		// 电池序列号
		//uint8_t BatteryID[16];
	}__PACKED;
	
	//状态信息
	#define MAX_DATA_LEN 100
	#define MAX_PACKS 10
	uint16_t crc=0;
	int8_t currentState = -1;	//-1:通信未开始 0-20:接收到包头等待数据
	bool packsRc[MAX_PACKS] = {0};
	bool toggle;
	Static_AXIDMABuf uint8_t payLoadData[MAX_DATA_LEN];
	while(1)
	{
		CanPacket mail;
		if( driver_info.mail_box->receiveMail( &mail, 2 ) )
		{
			if( mail.Identifier == 0x1109216 )
			{	//目标信息包	
				if( mail.data[7] & (1<<7) )
				{ //包头
					crc = *(uint16_t*)&mail.data[0];
					CCITT_CRC16Init(&mail.data[2],5);
					memcpy( payLoadData, &mail.data[2], 5 );
					memset( packsRc, 0, sizeof(packsRc) );
					packsRc[0] = true;
					currentState = 0;
				}
				else
				{	//中间
					if( mail.DataLength < 1 )
					{	//包长度太短
						currentState = -1;
						continue;
					}
					uint8_t tailByte = mail.data[mail.DataLength-1];
					uint8_t current_id = tailByte & 0x1f;
					if( current_id==0 || current_id>=MAX_PACKS )
					{	//包序号不正确
						currentState = -1;
						continue;
					}
					bool current_toggle = tailByte & (1<<5);
					if( currentState == 0 )
						toggle = current_toggle;
					else if( (tailByte & 0xc0) == 0 )
					{
						toggle = !toggle;
						if( toggle != current_toggle )
						{	//toggle位错误
							currentState = -1;
							continue;
						}
					}
					if( packsRc[current_id] == false )
					{	//确认未曾接收到这个包
						++currentState;
						packsRc[current_id] = true;
						//复制数据
						memcpy( &payLoadData[5+(current_id-1)*7], mail.data, mail.DataLength-1 );
						//校验crc
						CCITT_CRC_ARRAY(mail.data,mail.DataLength-1);
						if( tailByte & 0x40 )
						{	//收到包尾
							if( crc == CCITT_CRC16 )
							{	//crc校验通过
								uint16_t totalBytes = 5 + (current_id-1)*7 + mail.DataLength-1;
								uint16_t batS = totalBytes - sizeof(target_info_packet_header) - sizeof(target_info_packet_tail);
								if( (batS & 1) == 0 )
								{	//电池节数字节必须是偶数
									batS >>= 1;
									//读取电池信息
									target_info_packet_header* packHeader = (target_info_packet_header*)&payLoadData[0];
									target_info_packet_tail* packTail = (target_info_packet_tail*)&payLoadData[sizeof(target_info_packet_header)+batS*2];
									uint16_t* batVolts = (uint16_t*)&payLoadData[sizeof(target_info_packet_header)];
								
									float stVolt = batS*3.85f;
									float powerUsage = stVolt * (packTail->batteryCapacityExpected - packTail->batteryCapacityRemain)*0.001f;
									batteryUpdate( 0, driver_info.sensor_key,
										true,	//available
										packHeader->batteryMainVoltage*0.001f,	//totalVoltRaw
										packHeader->batteryMainVoltage*0.001f, //totalVolt
										stVolt,	//stVolt
										packHeader->current*-0.01f,	//total current
										&powerUsage,	//power usage
										stVolt * packTail->batteryCapacityExpected*0.001f, //capacity
										packHeader->batteryCapacityPercent,	//percent
										packHeader->batteryTemp,	//temperature
										packHeader->useCircleCount, //cycle count
										packTail->errorFlag,	//error flags
										0, 0 );
								}
							}
							//复位状态机
							currentState = -1;
						}
						else
						{	//数据包
							if( mail.DataLength != 8 )
							{	//包内容长度不符
								currentState = -1;
								continue;
							}
						}
					}
					else
					{	//已经接收过这个包
						currentState = -1;
						continue;
					}
				}
//				else if( (mail.data[7]&2) == 2 && payLoadCnt+7>=payLoadLen )
//				{// End of transfer
//					uint8_t remainPayLoad = payLoadLen - payLoadCnt;
//					if(remainPayLoad>0 && payLoadCnt+remainPayLoad<MAX_DATA_LEN){
//						memcpy(&payLoadData[payLoadCnt],mail.data,remainPayLoad);
//						CCITT_CRC_ARRAY(mail.data,remainPayLoad);
//					}
//					if(CCITT_CRC16 == crc){ // 校验通过
//						if( (payLoadLen-5)==packet_len_12S){ // 12S
//							target_info_packet_12S* packet = (target_info_packet_12S*)&payLoadData[0];
//							battery1.manufacturerID = packet->manufacturerID;
//							battery1.batteryTypeID = packet->batteryTypeID;
//							battery1.batteryMainVoltage = packet->batteryMainVoltage;
//							battery1.current = packet->current;
//							battery1.batteryTemp = packet->batteryTemp;
//							battery1.batteryCapacityPercent = packet->batteryCapacityPercent;
//							battery1.useCircleCount = packet->useCircleCount;
//							battery1.health = packet->health;
//							battery1.batteryCapacityExpected = packet->batteryCapacityExpected;
//							battery1.batteryCapacityRemain = packet->batteryCapacityRemain;
//							battery1.errorFlag = packet->errorFlag;
//							for(int i=0;i<12;i++)
//								battery1.batteryCellVoltage[i] = packet->batteryCellVoltage[i];
//							for(int i=0;i<16;i++)
//								battery1.BatteryID[i] = packet->BatteryID[i];	
//							battery1.upDateTime = TIME::now();
//							battery1.cellNum=12;
//						}						
//						else if( (payLoadLen-5)==packet_len_14S ){// 14S
//							target_info_packet_14S* packet = (target_info_packet_14S*)&payLoadData[0];
//							battery1.manufacturerID = packet->manufacturerID;
//							battery1.batteryTypeID = packet->batteryTypeID;
//							battery1.batteryMainVoltage = packet->batteryMainVoltage;
//							battery1.current = packet->current;
//							battery1.batteryTemp = packet->batteryTemp;
//							battery1.batteryCapacityPercent = packet->batteryCapacityPercent;
//							battery1.useCircleCount = packet->useCircleCount;
//							battery1.health = packet->health;
//							battery1.batteryCapacityExpected = packet->batteryCapacityExpected;
//							battery1.batteryCapacityRemain = packet->batteryCapacityRemain;
//							battery1.errorFlag = packet->errorFlag;
//							for(int i=0;i<14;i++)
//								battery1.batteryCellVoltage[i] = packet->batteryCellVoltage[i];
//							for(int i=0;i<16;i++)
//								battery1.BatteryID[i] = packet->BatteryID[i];
//							battery1.upDateTime = TIME::now();
//							battery1.cellNum=14;
//						}
//					}					
//					state=0;
//					crc=CRC_CCITT_INIT;
//					payLoadCnt=0;
//				}
//				else
//				{
//					state=0;
//					crc=CRC_CCITT_INIT;
//					payLoadCnt=0;				
//				}
			}
		}
	}
}

static bool CAN_Radar_BMS_TATTU_DriverInit()
{
	return true;
}
static bool CAN_Radar_BMS_TATTU_DriverRun()
{
	//注册电池
	uint32_t batId = batteryRegister(0);
	if( batId == 0 )
		return false;
	
	CanId Ids[2];
	Ids[0].Identifier = 0x1109216;	Ids[0].IdType = 1;
	CanMailBox* mail_box = new CanMailBox( 5, Ids, 1 );
	DriverInfo* driver_info = new DriverInfo;
	driver_info->mail_box = mail_box;
	driver_info->sensor_key = batId;
	xTaskCreate( CAN_Radar_BMS_TATTU_Server, "CAN_Radar_BMS_TATTU", 800, (void*)driver_info, SysPriority_UserTask, NULL);
	return true;
}

void init_drv_CAN_BMS_TATTU()
{
	CanFunc_Register( 7, CAN_Radar_BMS_TATTU_DriverInit, CAN_Radar_BMS_TATTU_DriverRun );
}