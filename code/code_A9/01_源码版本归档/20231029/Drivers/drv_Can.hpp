#pragma once

#include <stdint.h>
#include "freertos.h"
#include "queue.h"

struct CanPacket
{
	uint8_t IdType;	//0-standard 1-extended
	uint8_t DataLength;
	uint8_t FrameType;	//0-data_frame 1-remote_frame
	uint8_t FDFormat;	//0-classic 1-FD
	uint32_t Identifier;		/*!< Specifies the identifier.
														 This parameter must be a number between:
															- 0 and 0x7FF, if IdType is FDCAN_STANDARD_ID
															- 0 and 0x1FFFFFFF, if IdType is FDCAN_EXTENDED_ID               */
	uint8_t data[64];
};

struct CanId
{
	uint8_t IdType;	//0-standard 1-extended
	uint32_t Identifier;		/*!< Specifies the identifier.
														 This parameter must be a number between:
															- 0 and 0x7FF, if IdType is FDCAN_STANDARD_ID
															- 0 and 0x1FFFFFFF, if IdType is FDCAN_EXTENDED_ID   */
};

class CanMailBox
{
	private:
		uint16_t rx_packets_cnt;
		QueueHandle_t packet_queue;
		uint32_t mail_overflow_cnt;
	
	public:
		/*初始化Can邮箱
				Capacity：邮箱容量
				Ids：要订阅的消息Id列表
				Ids_count：要订阅的消息个数
		*/
		CanMailBox( uint16_t Capacity, 
								CanId* Ids, uint16_t Ids_count );
	
		//获取溢出的邮件个数
		uint32_t get_mail_overflow_cnt(){ return mail_overflow_cnt; }
	
		//接收邮件
		bool receiveMail( CanPacket* mail, double waitTime );
	
		//发送邮件
		static bool SendMail( CanPacket mail, double Send_waitTime=0, double Sync_waitTime=-1 );
	
		//Can消息接收处理回调函数
		//仅内部使用
		static void CanRxTCB( void *pvParameter1, uint32_t ulParameter2 );
};

void init_drv_Can(void);
bool WaitSent_Can( double waitTime );
bool ResetRx_Can( double Sync_waitTime );
//uint16_t Read_Can( Can_Frame *CanData, uint16_t NbrOfPack, double Rc_waitTime, double Sync_waitTime );
uint16_t Write_Can( uint8_t *Data, uint16_t Length, uint16_t MsgId, double Send_waitTime, double Sync_waitTime);
