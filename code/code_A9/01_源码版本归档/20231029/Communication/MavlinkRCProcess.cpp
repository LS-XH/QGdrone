#include "MavlinkRCProcess.hpp"
#include "mavlink.h"
#include "Commulink.hpp"
#include "Parameters.hpp"
#include "MavlinkCMDProcess.hpp"
#include "NavCmdProcess.hpp"
#include "fatfs.h"

#include "Basic.hpp"
#include "AC_Math.hpp"
#include "MeasurementSystem.hpp"
#include "Modes.hpp"
#include "Missions.hpp"
#include "Fences.hpp"
#include "ControlSystem.hpp"
#include "Sensors.hpp"
#include "SensorsBackend.hpp"
#include "AuxFuncs.hpp"
#include "ReceiverBackend.hpp"
#include "StorageSystem.hpp"
#include "drv_CRC.hpp"
#include "usb_composite.h"
#include "followTarget.hpp"
#include "precLand.hpp"

bool GCS_is_MP = false;
static void Msg0_HEARTBEAT( uint8_t Port_index , const mavlink_message_t* msg )
{
	const mavlink_heartbeat_t* msg_rd = (mavlink_heartbeat_t*)msg->payload64;
	GCS_is_MP = msg_rd->base_mode == 0;
	//对方是mavlink1就用mavlink1协议
	if( msg->magic == MAVLINK_STX )
		mavlink_set_proto_version( Port_index , 2 );
	else
		mavlink_set_proto_version( Port_index , 1 );
}

static void Msg183_AUTOPILOT_VERSION_REQUEST( uint8_t Port_index , const mavlink_message_t* msg )
{
	const mavlink_autopilot_version_request_t* msg_rd = (mavlink_autopilot_version_request_t*)msg->payload64;
	if( msg_rd->target_system==0 || 
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==get_CommulinkCompId()) ||
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==0) )
	{
		send_AutoPilot_Version(Port_index);
	}
}

//切换模式
static void Msg11_SET_MODE( uint8_t Port_index , const mavlink_message_t* msg )
{
	const mavlink_set_mode_t* msg_rd = (mavlink_set_mode_t*)msg->payload64;
	if( msg_rd->target_system==0 || (msg_rd->target_system==get_CommulinkSysId()) )
	{
		px4_custom_mode t_mav_mode;
		t_mav_mode.data = msg_rd->custom_mode;
		
		ModeMsg mode_msg;
		mode_msg.cmd_type = CMD_TYPE_MAVLINK | Port_index;
		mode_msg.sd_sysid = msg->sysid;
		mode_msg.sd_compid = msg->compid;
		mode_msg.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
		mode_msg.cmd = 176;
		mode_msg.params[0] = msg_rd->base_mode;
		mode_msg.params[1] = t_mav_mode.main_mode;
		mode_msg.params[2] = t_mav_mode.sub_mode;
		mode_msg.params[3] = 0;
		mode_msg.params[4] = 0;
		mode_msg.params[5] = 0;
		mode_msg.params[6] = 0;
		SendMsgToMode( mode_msg, 0.01 );
	}
}

/*FTP协议*/
	#define MAX_PAYLOADDATA_SIZE 239
	struct __attribute__((__packed__)) ftpPack
	{
		uint16_t seq;
		uint8_t session;
		uint8_t opCode;
		uint8_t size;
		uint8_t req_opCode;
		uint8_t burst_complete;
		uint8_t padding;
		uint32_t offset;
		uint8_t data[MAX_PAYLOADDATA_SIZE];
	};
	static ftpPack ftpRes;

	#define FTPMAXFILECOUNT 2
	static FIL* ftpFils[FTPMAXFILECOUNT] = {0};
	/*Burst read*/
		struct ftpBurstInfo
		{
			//当前burst位置offset
			uint32_t offset;
			//当前burst剩余字节
			uint32_t remain;
			//burst端口号
			uint8_t portIndex;
			//target sys
			uint8_t targetSysId;
			//target component
			uint8_t targetCompId;
		};
		static ftpBurstInfo ftpBurstInfos[FTPMAXFILECOUNT] = {0};
	/*Burst read*/
	/*crc*/
		#define FTPCRC32BUFFERSIZE (4096)
		struct _ftpCrc32Info
		{
			uint8_t port;
			uint8_t targetSysId;
			uint8_t targetCompId;
			uint8_t session;
			uint32_t crcRs;
			
			uint32_t offset;
			uint8_t buffer[FTPCRC32BUFFERSIZE+32];
		};
		static _ftpCrc32Info* ftpCrc32Info = 0;
//		static uint8_t ftpCrc32Port = 0;
//		static uint8_t ftpCrc32_targetSysId = 0;
//		static uint8_t ftpCrc32_targetCompId = 0;
//		static uint8_t* ftpCrc32Buffer = 0;
//		static int8_t ftpCrc32Session = -1;
//		static uint32_t ftpCrc32Rs;
	/*crc*/
	static inline int8_t getFtpFreeSection()
	{
		int8_t res = -1;
		for( uint8_t i=0; i<FTPMAXFILECOUNT; ++i )
		{
			if( ftpFils[i] == 0 )
			{
				res = i;
				break;
			}
		}
		return res;
	}
	static inline int8_t getFtpFreeSectionFromBehind()
	{
		int8_t res = -1;
		for( uint8_t i=FTPMAXFILECOUNT-1; i>=0; ++i )
		{
			if( ftpFils[i] == 0 )
			{
				res = i;
				break;
			}
		}
		return res;
	}
	void ftpProtocolTask()
	{
		if( Get_SD_Init_Success() )
		{
			/*Burst read*/
				for( uint8_t i=0; i<FTPMAXFILECOUNT; ++i )
				{
					ftpBurstInfo* burst = &ftpBurstInfos[i];
					if( burst->remain )
					{
						if( ftpFils[i] )
						{	//文件已打开
							const Port* port = get_CommuPort( burst->portIndex );
							if( port && port->write )
							{
								if( mavlink_lock_chan( burst->portIndex, 0.01 ) )
								{
									ftpRes.opCode = opCodeRspAck;
									ftpRes.session = i;
									ftpRes.offset = burst->offset;
									ftpRes.req_opCode = opCodeBurstReadFile;
									ftpRes.padding = 0;
									
									f_lseek( ftpFils[i], burst->offset );
									while( port->txSpacesAvailable && port->txSpacesAvailable()>384 )
									{
										ftpRes.seq += 1;
										uint8_t cReadData = burst->remain < MAX_PAYLOADDATA_SIZE ? burst->remain : MAX_PAYLOADDATA_SIZE;
										size_t readSize=0;
										FRESULT fres = f_read( ftpFils[i], ftpRes.data, cReadData, &readSize );
										if( fres == FR_OK )
										{	//读取文件成功
											burst->remain -= readSize;
											
											ftpRes.offset = burst->offset;
											ftpRes.size = readSize;
											if( f_eof(ftpFils[i]) || burst->remain==0 )
											{
												burst->remain = 0;
												ftpRes.burst_complete = 1;
											}
											else
												ftpRes.burst_complete = 0;
											memset( &ftpRes.data[ftpRes.size], 0, MAX_PAYLOADDATA_SIZE-ftpRes.size );
											burst->offset += readSize;
											
											mavlink_message_t msg_sd;
											mavlink_msg_file_transfer_protocol_pack_chan( 
												get_CommulinkSysId() ,	//system id
												get_CommulinkCompId() ,	//component id
												burst->portIndex ,	//chan
												&msg_sd,
												0,	//target network
												burst->targetSysId,	//target system
												burst->targetCompId,	//target component
												(uint8_t*)&ftpRes	//payload
											);
											mavlink_msg_to_send_buffer(port->write, 
																								 port->lock,
																								 port->unlock,
																								 &msg_sd, 0, 0.01);
											if( ftpRes.burst_complete )
												break;
										}
										else
										{	//读取文件失败
											ftpRes.offset = burst->offset;
											ftpRes.size = 0;
											burst->remain = 0;
											ftpRes.burst_complete = 1;
											memset( &ftpRes.data[ftpRes.size], 0, MAX_PAYLOADDATA_SIZE-ftpRes.size );
											
											mavlink_message_t msg_sd;
											mavlink_msg_file_transfer_protocol_pack_chan( 
												get_CommulinkSysId() ,	//system id
												get_CommulinkCompId() ,	//component id
												burst->portIndex ,	//chan
												&msg_sd,
												0,	//target network
												burst->targetSysId,	//target system
												burst->targetCompId,	//target component
												(uint8_t*)&ftpRes	//payload
											);
											mavlink_msg_to_send_buffer(port->write, 
																								 port->lock,
																								 port->unlock,
																								 &msg_sd, 0, 0.01);
											break;
										}
									}
									mavlink_unlock_chan(burst->portIndex);
								}
							}
						}
						else
						{	//文件未打开
							burst->remain = 0;
						}
					}
				}
			/*Burst read*/
				
			/*crc32*/
				if( ftpCrc32Info )
				{
					if( ftpCrc32Info->session<FTPMAXFILECOUNT && ftpFils[ftpCrc32Info->session] )
					{
						size_t bytesRead;
						uint8_t* crcBuffer = (uint8_t*)( ( (uint32_t)ftpCrc32Info->buffer & ~(uint32_t)0x1f ) + 32 );
						f_lseek( ftpFils[ftpCrc32Info->session], ftpCrc32Info->offset );
						if( f_read( ftpFils[ftpCrc32Info->session], crcBuffer, FTPCRC32BUFFERSIZE, &bytesRead ) == FR_OK )
						{
							CRC_Calc( cg_CRC_CONFIG_CRC32_MAVLINK, crcBuffer, bytesRead, &ftpCrc32Info->crcRs, ftpCrc32Info->crcRs, ftpCrc32Info->offset==0 ? false : true );
							ftpCrc32Info->offset += bytesRead;
							if( f_eof( ftpFils[ftpCrc32Info->session] ) )
							{	//crc32计算完成
								const Port* port = get_CommuPort( ftpCrc32Info->port );
								if( port!=0 && port->write )
								{
									//回应包
									mavlink_message_t msg_sd;
									ftpRes.seq += 1;
									ftpRes.session = ftpCrc32Info->session;
									ftpRes.offset = 0;
									ftpRes.req_opCode = opCodeCalcFileCRC32;
									ftpRes.burst_complete = 0;
									ftpRes.padding = 0;
									//操作结果
									ftpRes.opCode = opCodeRspAck;
									ftpRes.size = 4;
									*(uint32_t*)ftpRes.data = ftpCrc32Info->crcRs;
									memset( &ftpRes.data[ftpRes.size], 0, MAX_PAYLOADDATA_SIZE-ftpRes.size );
									if( mavlink_lock_chan( ftpCrc32Info->port, 0.01 ) )
									{
										mavlink_msg_file_transfer_protocol_pack_chan( 
											get_CommulinkSysId() ,	//system id
											get_CommulinkCompId() ,	//component id
											ftpCrc32Info->port ,	//chan
											&msg_sd,
											0,	//target network
											ftpCrc32Info->targetSysId,	//target system
											ftpCrc32Info->targetCompId,	//target component
											(uint8_t*)&ftpRes	//payload
										);
										mavlink_msg_to_send_buffer(port->write, 
																							 port->lock,
																							 port->unlock,
																							 &msg_sd, 0, 0.01);
										mavlink_unlock_chan(ftpCrc32Info->port);
									}
								}
								f_close(ftpFils[ftpCrc32Info->session]);
								delete ftpFils[ftpCrc32Info->session];
								ftpFils[ftpCrc32Info->session] = 0;
								
								delete ftpCrc32Info;
								ftpCrc32Info = 0;
							}
						}
						else
						{	//文件读取失败
							const Port* port = get_CommuPort( ftpCrc32Info->port );
							if( port!=0 && port->write )
							{
								//回应包
								mavlink_message_t msg_sd;
								ftpRes.seq += 1;
								ftpRes.session = 0;
								ftpRes.offset = 0;
								ftpRes.req_opCode = opCodeCalcFileCRC32;
								ftpRes.burst_complete = 0;
								ftpRes.padding = 0;
								//操作结果
								ftpRes.opCode = opCodeRspNak;
								ftpRes.size = 1;
								ftpRes.data[0] = opCodeErrFail;
								memset( &ftpRes.data[ftpRes.size], 0, MAX_PAYLOADDATA_SIZE-ftpRes.size );
								if( mavlink_lock_chan( ftpCrc32Info->port, 0.01 ) )
								{
									mavlink_msg_file_transfer_protocol_pack_chan( 
										get_CommulinkSysId() ,	//system id
										get_CommulinkCompId() ,	//component id
										ftpCrc32Info->port ,	//chan
										&msg_sd,
										0,	//target network
										ftpCrc32Info->targetSysId,	//target system
										ftpCrc32Info->targetCompId,	//target component
										(uint8_t*)&ftpRes	//payload
									);
									mavlink_msg_to_send_buffer(port->write, 
																						 port->lock,
																						 port->unlock,
																						 &msg_sd, 0, 0.01);
									mavlink_unlock_chan(ftpCrc32Info->port);
								}
							}
							f_close(ftpFils[ftpCrc32Info->session]);
							delete ftpFils[ftpCrc32Info->session];
							ftpFils[ftpCrc32Info->session] = 0;
							
							delete ftpCrc32Info;
							ftpCrc32Info = 0;
						}
					}
					else
					{	//文件不存在
						delete ftpCrc32Info;
						ftpCrc32Info = 0;
					}
				}
			/*crc32*/
		}
		else
		{	//sd卡未初始化
			
			/*Burst read*/
				for( uint8_t i=0; i<FTPMAXFILECOUNT; ++i )
				{
					if( ftpFils[i] )
					{
						f_close(ftpFils[i]);
						delete ftpFils[i];
						ftpFils[i] = 0;
					}
					ftpBurstInfos[i].remain = 0;
				}
			/*Burst read*/
				
			/*crc32*/
				if( ftpCrc32Info )
				{
					delete ftpCrc32Info;
					ftpCrc32Info = 0;
				}
			/*crc32*/
		}
	}
	static void Msg110_FILE_TRANSFER_PROTOCOL( uint8_t Port_index , const mavlink_message_t* msg )
	{
		const mavlink_file_transfer_protocol_t* msg_rd = (mavlink_file_transfer_protocol_t*)msg->payload64;
		if( msg_rd->target_system==0 || 
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==get_CommulinkCompId()) ||
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==0) )
		{
			const ftpPack* ftpReq = (ftpPack*)msg_rd->payload;
			
			switch(ftpReq->opCode)
			{
				case opCodeOpenFileRO:
				case opCodeOpenFileWO:
				case opCodeCreateFile:
				{	//打开文件
					const Port* port = get_CommuPort( Port_index );
					if( port!=0 && port->write )
					{
						//回应包
						mavlink_message_t msg_sd;
						ftpRes.seq = ftpReq->seq+1;
						ftpRes.session = 0;
						ftpRes.offset = 0;
						ftpRes.req_opCode = ftpReq->opCode;
						ftpRes.burst_complete = 0;
						ftpRes.padding = 0;
						//操作结果
						OpErrorCode opErr = opCodeErrNone;
						size_t filSize = 0;
						if( Get_SD_Init_Success() )
						{	//sd卡初始化成功
							
							//寻找空闲session
							int8_t freeFilSession = getFtpFreeSection();
							if( freeFilSession >= 0 )
							{	//有空闲session
								//文件名称
								char filename[255];
								filename[0] = 0;
								strcat( filename, SDPath );
								strncat( filename, (char*)&(ftpReq->data[1]), ftpReq->size );
								ftpFils[freeFilSession] = new FIL;
								//打开文件
								if( ftpFils[freeFilSession] )
								{	//new文件成功
									FRESULT fres;
									if( ftpReq->opCode == opCodeOpenFileRO )
										fres = f_open( ftpFils[freeFilSession], filename, FA_OPEN_EXISTING | FA_READ );
									else
									{	//usb存在时不进行写入操作
										if( !get_is_usb_connected() )
										{
											if( ftpReq->opCode == opCodeOpenFileWO )
												fres = f_open( ftpFils[freeFilSession], filename, FA_OPEN_ALWAYS | FA_WRITE );
											else if( ftpReq->opCode == opCodeCreateFile )
												fres = f_open( ftpFils[freeFilSession], filename, FA_CREATE_ALWAYS | FA_WRITE );
										}
										else
											fres = FR_DENIED;
									}
									if( fres == FR_OK )
									{	//打开文件成功
										ftpRes.session = freeFilSession;
										filSize = f_size(ftpFils[freeFilSession]);
									}
									else
									{	//打开文件失败
										if( fres==FR_NO_FILE || fres==FR_NO_PATH )
											opErr = opCodeErrFileNotFound;
										else
											opErr = opCodeErrFail;
										//删除新建的文件
										delete ftpFils[freeFilSession];
										ftpFils[freeFilSession] = 0;
									}
								}
								else	//new文件失败
									opErr = opCodeErrNoSessionsAvailable;
							}
							else
							{	//无可用session
								opErr = opCodeErrNoSessionsAvailable;
							}
						}
						else
						{	//sd卡初始化失败
							opErr = opCodeErrFail;
						}
						
						if( opErr == opCodeErrNone )
						{	//无错误
							ftpRes.opCode = opCodeRspAck;
							ftpRes.size = 4;
							*(uint32_t*)ftpRes.data = filSize;
						}
						else
						{
							ftpRes.opCode = opCodeRspNak;
							ftpRes.size = 1;
							ftpRes.data[0] = opErr;
						}
						memset( &ftpRes.data[ftpRes.size], 0, MAX_PAYLOADDATA_SIZE-ftpRes.size );
						if( mavlink_lock_chan( Port_index, 0.01 ) )
						{
							mavlink_msg_file_transfer_protocol_pack_chan( 
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								Port_index ,	//chan
								&msg_sd,
								0,	//target network
								msg->sysid,	//target system
								msg->compid,	//target component
								(uint8_t*)&ftpRes	//payload
							);
							mavlink_msg_to_send_buffer(port->write, 
																				 port->lock,
																				 port->unlock,
																				 &msg_sd, 0, 0.01);
							mavlink_unlock_chan(Port_index);
						}
					}
					break;
				}
				
				case opCodeCreateDirectory:
				{	//创建文件夹
					const Port* port = get_CommuPort( Port_index );
					if( port!=0 && port->write )
					{
						//回应包
						mavlink_message_t msg_sd;
						ftpRes.seq = ftpReq->seq+1;
						ftpRes.session = 0;
						ftpRes.offset = 0;
						ftpRes.req_opCode = ftpReq->opCode;
						ftpRes.burst_complete = 0;
						ftpRes.padding = 0;
						//操作结果
						OpErrorCode opErr = opCodeErrNone;
						if( Get_SD_Init_Success() && get_is_usb_connected()==false )
						{	//sd卡初始化成功
							
							//文件名称
							char filename[255];
							filename[0] = 0;
							strcat( filename, SDPath );
							strncat( filename, (char*)&(ftpReq->data[1]), ftpReq->size );
							//新建文件夹
							FRESULT fres = f_mkdir(filename);
							if( fres != FR_OK )
							{	//创建失败
								opErr = opCodeErrFail;
							}
						}
						else
						{	//sd卡初始化失败
							opErr = opCodeErrFail;
						}
						
						if( opErr == opCodeErrNone )
						{	//无错误
							ftpRes.opCode = opCodeRspAck;
							ftpRes.size = 0;
						}
						else
						{
							ftpRes.opCode = opCodeRspNak;
							ftpRes.size = 1;
							ftpRes.data[0] = opErr;
						}
						memset( &ftpRes.data[ftpRes.size], 0, MAX_PAYLOADDATA_SIZE-ftpRes.size );
						if( mavlink_lock_chan( Port_index, 0.01 ) )
						{
							mavlink_msg_file_transfer_protocol_pack_chan( 
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								Port_index ,	//chan
								&msg_sd,
								0,	//target network
								msg->sysid,	//target system
								msg->compid,	//target component
								(uint8_t*)&ftpRes	//payload
							);
							mavlink_msg_to_send_buffer(port->write, 
																				 port->lock,
																				 port->unlock,
																				 &msg_sd, 0, 0.01);
							mavlink_unlock_chan(Port_index);
						}
					}
					break;
				}
				
				case opCodeRename:
				{	//重命名
					const Port* port = get_CommuPort( Port_index );
					if( port!=0 && port->write )
					{
						//回应包
						mavlink_message_t msg_sd;
						ftpRes.seq = ftpReq->seq+1;
						ftpRes.session = 0;
						ftpRes.offset = 0;
						ftpRes.req_opCode = ftpReq->opCode;
						ftpRes.burst_complete = 0;
						ftpRes.padding = 0;
						//操作结果
						OpErrorCode opErr = opCodeErrNone;
						if( Get_SD_Init_Success() && get_is_usb_connected()==false )
						{	//sd卡初始化成功
							
							//文件名称
							char filename1[200];
							char filename2[200];
							int16_t strSp = -1;
							for( uint8_t i=1; i<ftpReq->size; ++i )
							{	//寻找分隔符位置
								if( ftpReq->data[i] == 0 )
								{
									strSp = i;
									break;
								}
							}
							if( strSp >= 0 )
							{	//存在分隔符
								//源文件路径
								filename1[0] = 0;
								strcat( filename1, SDPath );
								strncat( filename1, (char*)&(ftpReq->data[1]), ftpReq->size );
								//新文件路径
								filename2[0] = 0;
								strcat( filename2, SDPath );
								strncat( filename2, (char*)&(ftpReq->data[strSp+1]), ftpReq->size-strSp-1 );
								//新建文件夹
								FRESULT fres = f_rename( filename1, filename2 );
								if( fres != FR_OK )
								{	//失败
									opErr = opCodeErrFail;
								}
							}
							else	//不存在0分隔符
								opErr = opCodeErrFail;
						}
						else
						{	//sd卡初始化失败
							opErr = opCodeErrFail;
						}
						
						if( opErr == opCodeErrNone )
						{	//无错误
							ftpRes.opCode = opCodeRspAck;
							ftpRes.size = 0;
						}
						else
						{
							ftpRes.opCode = opCodeRspNak;
							ftpRes.size = 1;
							ftpRes.data[0] = opErr;
						}
						memset( &ftpRes.data[ftpRes.size], 0, MAX_PAYLOADDATA_SIZE-ftpRes.size );
						if( mavlink_lock_chan( Port_index, 0.01 ) )
						{
							mavlink_msg_file_transfer_protocol_pack_chan( 
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								Port_index ,	//chan
								&msg_sd,
								0,	//target network
								msg->sysid,	//target system
								msg->compid,	//target component
								(uint8_t*)&ftpRes	//payload
							);
							mavlink_msg_to_send_buffer(port->write, 
																				 port->lock,
																				 port->unlock,
																				 &msg_sd, 0, 0.01);
							mavlink_unlock_chan(Port_index);
						}
					}
					break;
				}
				
				case opCodeWriteFile:
				{	//写文件
					const Port* port = get_CommuPort( Port_index );
					if( port!=0 && port->write )
					{
						//回应包
						mavlink_message_t msg_sd;
						ftpRes.seq = ftpReq->seq+1;
						ftpRes.session = ftpReq->session;
						ftpRes.offset = ftpReq->offset;
						ftpRes.req_opCode = ftpReq->opCode;
						ftpRes.burst_complete = 0;
						ftpRes.padding = 0;
						//操作结果
						OpErrorCode opErr = opCodeErrNone;
						size_t writeSize = 0;
						if( Get_SD_Init_Success() && ftpReq->size<=MAX_PAYLOADDATA_SIZE && get_is_usb_connected()==false )
						{	//sd卡初始化成功
							if( ftpReq->session<FTPMAXFILECOUNT && ftpFils[ftpReq->session] )
							{	//session存在且文件已打开
								f_lseek( ftpFils[ftpReq->session], ftpReq->offset );
								FRESULT fres = f_write( ftpFils[ftpReq->session], ftpReq->data, ftpReq->size, &writeSize );
								if( fres != FR_OK )
									opErr = opCodeErrFail;
							}
							else
							{	//session不存在或文件未打开
								opErr = opCodeErrInvalidSession;
							}
						}
						else
						{	//sd卡初始化失败或需求的字节数量过多
							opErr = opCodeErrFail;
						}
						
						if( opErr == opCodeErrNone )
						{	//无错误
							ftpRes.opCode = opCodeRspAck;
							ftpRes.size = 0;
						}
						else
						{
							ftpRes.opCode = opCodeRspNak;
							ftpRes.size = 1;
							ftpRes.data[0] = opErr;
						}
						memset( &ftpRes.data[ftpRes.size], 0, MAX_PAYLOADDATA_SIZE-ftpRes.size );
						if( mavlink_lock_chan( Port_index, 0.01 ) )
						{
							mavlink_msg_file_transfer_protocol_pack_chan( 
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								Port_index ,	//chan
								&msg_sd,
								0,	//target network
								msg->sysid,	//target system
								msg->compid,	//target component
								(uint8_t*)&ftpRes	//payload
							);
							mavlink_msg_to_send_buffer(port->write, 
																				 port->lock,
																				 port->unlock,
																				 &msg_sd, 0, 0.01);
							mavlink_unlock_chan(Port_index);
						}
					}
					break;
				}
				
				case opCodeTruncateFile:
				{	//截短文件
					const Port* port = get_CommuPort( Port_index );
					if( port!=0 && port->write )
					{
						//回应包
						mavlink_message_t msg_sd;
						ftpRes.seq = ftpReq->seq+1;
						ftpRes.session = ftpReq->session;
						ftpRes.offset = ftpReq->offset;
						ftpRes.req_opCode = ftpReq->opCode;
						ftpRes.burst_complete = 0;
						ftpRes.padding = 0;
						//操作结果
						OpErrorCode opErr = opCodeErrNone;
						if( Get_SD_Init_Success() && ftpReq->size<=MAX_PAYLOADDATA_SIZE && get_is_usb_connected()==false )
						{	//sd卡初始化成功
							if( ftpReq->session<FTPMAXFILECOUNT && ftpFils[ftpReq->session] )
							{	//session存在且文件已打开
								FRESULT fres;
								fres= f_lseek( ftpFils[ftpReq->session], ftpReq->offset );
								if( fres != FR_OK )
									opErr = opCodeErrFail;
								else
									fres = f_truncate(ftpFils[ftpReq->session]);
								if( fres != FR_OK )
									opErr = opCodeErrFail;
							}
							else
							{	//session不存在或文件未打开
								opErr = opCodeErrInvalidSession;
							}
						}
						else
						{	//sd卡初始化失败或需求的字节数量过多
							opErr = opCodeErrFail;
						}
						
						if( opErr == opCodeErrNone )
						{	//无错误
							ftpRes.opCode = opCodeRspAck;
							ftpRes.size = 0;
						}
						else
						{
							ftpRes.opCode = opCodeRspNak;
							ftpRes.size = 1;
							ftpRes.data[0] = opErr;
						}
						memset( &ftpRes.data[ftpRes.size], 0, MAX_PAYLOADDATA_SIZE-ftpRes.size );
						if( mavlink_lock_chan( Port_index, 0.01 ) )
						{
							mavlink_msg_file_transfer_protocol_pack_chan( 
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								Port_index ,	//chan
								&msg_sd,
								0,	//target network
								msg->sysid,	//target system
								msg->compid,	//target component
								(uint8_t*)&ftpRes	//payload
							);
							mavlink_msg_to_send_buffer(port->write, 
																				 port->lock,
																				 port->unlock,
																				 &msg_sd, 0, 0.01);
							mavlink_unlock_chan(Port_index);
						}
					}
					break;
				}
				
				case opCodeTerminateSession:
				{	//关闭文件
					const Port* port = get_CommuPort( Port_index );
					if( port!=0 && port->write )
					{
						//回应包
						mavlink_message_t msg_sd;
						ftpRes.seq = ftpReq->seq+1;
						ftpRes.session = ftpReq->session;
						ftpRes.offset = 0;
						ftpRes.req_opCode = ftpReq->opCode;
						ftpRes.burst_complete = 0;
						ftpRes.padding = 0;
						//操作结果
						OpErrorCode opErr = opCodeErrNone;
						if( Get_SD_Init_Success() )
						{	//sd卡初始化成功
							if( ftpReq->session<FTPMAXFILECOUNT && ftpFils[ftpReq->session] )
							{	//session存在且文件已打开
								f_close(ftpFils[ftpReq->session]);
								delete ftpFils[ftpReq->session];
								ftpFils[ftpReq->session] = 0;
							}
							else
							{	//session不存在或文件未打开
								opErr = opCodeErrInvalidSession;
							}
						}
						else
						{	//sd卡初始化失败
							opErr = opCodeErrFail;
						}
						
						if( opErr == opCodeErrNone )
						{	//无错误
							ftpRes.opCode = opCodeRspAck;
							ftpRes.size = 0;
						}
						else
						{
							ftpRes.opCode = opCodeRspNak;
							ftpRes.size = 1;
							ftpRes.data[0] = opErr;
						}
						memset( &ftpRes.data[ftpRes.size], 0, MAX_PAYLOADDATA_SIZE-ftpRes.size );
						if( mavlink_lock_chan( Port_index, 0.01 ) )
						{
							mavlink_msg_file_transfer_protocol_pack_chan( 
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								Port_index ,	//chan
								&msg_sd,
								0,	//target network
								msg->sysid,	//target system
								msg->compid,	//target component
								(uint8_t*)&ftpRes	//payload
							);
							mavlink_msg_to_send_buffer(port->write, 
																				 port->lock,
																				 port->unlock,
																				 &msg_sd, 0, 0.01);
							mavlink_unlock_chan(Port_index);
						}
					}
					break;
				}
				case opCodeResetSessions:
				{	//关闭所有文件
					const Port* port = get_CommuPort( Port_index );
					if( port!=0 && port->write )
					{
						//回应包
						mavlink_message_t msg_sd;
						ftpRes.seq = ftpReq->seq+1;
						ftpRes.session = 0;
						ftpRes.offset = 0;
						ftpRes.req_opCode = ftpReq->opCode;
						ftpRes.burst_complete = 0;
						ftpRes.padding = 0;
						//操作结果
						OpErrorCode opErr = opCodeErrNone;
						if( Get_SD_Init_Success() )
						{	//sd卡初始化成功
							for( uint8_t i=0; i<FTPMAXFILECOUNT; ++i )
							{
								if( ftpFils[i] )
								{
									f_close(ftpFils[i]);
									delete ftpFils[i];
									ftpFils[i] = 0;
								}
							}
						}
						else
						{	//sd卡初始化失败
							opErr = opCodeErrFail;
						}
						
						if( opErr == opCodeErrNone )
						{	//无错误
							ftpRes.opCode = opCodeRspAck;
							ftpRes.size = 0;
						}
						else
						{
							ftpRes.opCode = opCodeRspNak;
							ftpRes.size = 1;
							ftpRes.data[0] = opErr;
						}
						memset( &ftpRes.data[ftpRes.size], 0, MAX_PAYLOADDATA_SIZE-ftpRes.size );
						if( mavlink_lock_chan( Port_index, 0.01 ) )
						{
							mavlink_msg_file_transfer_protocol_pack_chan( 
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								Port_index ,	//chan
								&msg_sd,
								0,	//target network
								msg->sysid,	//target system
								msg->compid,	//target component
								(uint8_t*)&ftpRes	//payload
							);
							mavlink_msg_to_send_buffer(port->write, 
																				 port->lock,
																				 port->unlock,
																				 &msg_sd, 0, 0.01);
							mavlink_unlock_chan(Port_index);
						}
					}
					break;
				}
				
				case opCodeListDirectory:
				{	//罗列文件夹
					//文件名长度过大
					if( ftpReq->size > MAX_PAYLOADDATA_SIZE )
						break;
					
					const Port* port = get_CommuPort( Port_index );
					if( port!=0 && port->write )
					{
						//回应包
						mavlink_message_t msg_sd;
						ftpRes.seq = ftpReq->seq+1;
						ftpRes.session = 0;
						ftpRes.offset = ftpReq->offset;
						ftpRes.req_opCode = ftpReq->opCode;
						ftpRes.burst_complete = 0;
						ftpRes.padding = 0;
						//操作结果
						OpErrorCode opErr = opCodeErrNone;
						uint32_t cOffset = ftpReq->offset;
						if( Get_SD_Init_Success() )
						{	//sd卡初始化成功
							//文件夹名称
							char filename[255];
							filename[0] = 0;
							strcat( filename, SDPath );
							strncat( filename, (char*)&(ftpReq->data[1]), ftpReq->size );
							//打开文件夹
							DIR dir;
							FRESULT fres = f_opendir(&dir,filename);
							if( fres == FR_OK )
							{	//打开文件夹成功
								FILINFO fno;
								//跳过offset个文件(夹)
								while(cOffset--)
									f_readdir( &dir, &fno );
								//复位offset
								cOffset = 0;
								while(1)
								{	//遍历文件夹
									fres = f_readdir( &dir, &fno );
									if( fres != FR_OK )
									{	//读取文件夹错误
										opErr = opCodeErrFail;
										break;
									}
									uint8_t filename_len = strlen(fno.fname);
									if( filename_len == 0 )
									{	//文件夹末尾
										if( cOffset == 0 )
											opErr = opCodeErrEOF;
										break;
									}
									else
									{
										uint8_t exp_len;
										if (fno.fattrib & AM_DIR)
										{	//文件夹
											exp_len = snprintf( filename, 255, "D%s", fno.fname );
										}
										else
										{	//文件
											exp_len = snprintf( filename, 255, "F%s\t%lld", fno.fname, fno.fsize );
										}
										//尝试添加新文件名
										if( exp_len >= MAX_PAYLOADDATA_SIZE-1 )
										{	//文件名过长
											ftpRes.data[cOffset++] = 'S';
											ftpRes.data[cOffset++] = 0;
										}
										else if( exp_len >= MAX_PAYLOADDATA_SIZE-cOffset-1 )
										{	//文件名超出本次发送长度范围
											break;
										}
										else
										{	//添加此文件名
											memcpy( &ftpRes.data[cOffset], filename, exp_len+1 );
											cOffset += exp_len+1;
										}
									}
									//剩余长度太短退出
									if( MAX_PAYLOADDATA_SIZE - cOffset < 16 )
										break;
								}
								f_closedir(&dir);
							}
							else if( fres == FR_NO_PATH )
								opErr = opCodeErrFileNotFound;
							else
								opErr = opCodeErrFail;
						}
						else
						{	//sd卡初始化失败
							opErr = opCodeErrFail;
						}
						
						if( opErr == opCodeErrNone )
						{	//无错误
							ftpRes.opCode = opCodeRspAck;
							ftpRes.size = cOffset;
						}
						else
						{
							ftpRes.opCode = opCodeRspNak;
							ftpRes.size = 1;
							ftpRes.data[0] = opErr;
						}
						memset( &ftpRes.data[ftpRes.size], 0, MAX_PAYLOADDATA_SIZE-ftpRes.size );
						if( mavlink_lock_chan( Port_index, 0.01 ) )
						{
							mavlink_msg_file_transfer_protocol_pack_chan( 
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								Port_index ,	//chan
								&msg_sd,
								0,	//target network
								msg->sysid,	//target system
								msg->compid,	//target component
								(uint8_t*)&ftpRes	//payload
							);
							mavlink_msg_to_send_buffer(port->write, 
																				 port->lock,
																				 port->unlock,
																				 &msg_sd, 0, 0.01);
							mavlink_unlock_chan(Port_index);
						}
					}
					break;
				}
				
				case opCodeReadFile:
				{	//读取文件
					const Port* port = get_CommuPort( Port_index );
					if( port!=0 && port->write )
					{
						//回应包
						mavlink_message_t msg_sd;
						ftpRes.seq = ftpReq->seq+1;
						ftpRes.session = ftpReq->session;
						ftpRes.offset = ftpReq->offset;
						ftpRes.req_opCode = ftpReq->opCode;
						ftpRes.burst_complete = 0;
						ftpRes.padding = 0;
						//操作结果
						OpErrorCode opErr = opCodeErrNone;
						size_t readSize = 0;
						if( Get_SD_Init_Success() && ftpReq->size<=MAX_PAYLOADDATA_SIZE )
						{	//sd卡初始化成功
							if( ftpReq->session<FTPMAXFILECOUNT && ftpFils[ftpReq->session] )
							{	//session存在且文件已打开
								f_lseek( ftpFils[ftpReq->session], ftpReq->offset );
								FRESULT fres = f_read( ftpFils[ftpReq->session], ftpRes.data, ftpReq->size, &readSize );
								if( fres != FR_OK )
									opErr = opCodeErrFail;
							}
							else
							{	//session不存在或文件未打开
								opErr = opCodeErrInvalidSession;
							}
						}
						else
						{	//sd卡初始化失败或需求的字节数量过多
							opErr = opCodeErrFail;
						}
						
						if( opErr == opCodeErrNone )
						{	//无错误
							ftpRes.opCode = opCodeRspAck;
							ftpRes.size = readSize;
						}
						else
						{
							ftpRes.opCode = opCodeRspNak;
							ftpRes.size = 1;
							ftpRes.data[0] = opErr;
						}
						memset( &ftpRes.data[ftpRes.size], 0, MAX_PAYLOADDATA_SIZE-ftpRes.size );
						if( mavlink_lock_chan( Port_index, 0.01 ) )
						{
							mavlink_msg_file_transfer_protocol_pack_chan( 
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								Port_index ,	//chan
								&msg_sd,
								0,	//target network
								msg->sysid,	//target system
								msg->compid,	//target component
								(uint8_t*)&ftpRes	//payload
							);
							mavlink_msg_to_send_buffer(port->write, 
																				 port->lock,
																				 port->unlock,
																				 &msg_sd, 0, 0.01);
							mavlink_unlock_chan(Port_index);
						}
					}
					break;
				}
				
				case opCodeCalcFileCRC32:
				{	//计算crc32
					const Port* port = get_CommuPort( Port_index );
					if( port!=0 && port->write )
					{
						//回应包
						//操作结果
						OpErrorCode opErr = opCodeErrNone;
						if( Get_SD_Init_Success() && ftpCrc32Info==0 )
						{	//sd卡初始化成功且当前crc计算空闲
							
							//寻找空闲session
							int8_t freeFilSession = getFtpFreeSectionFromBehind();
							if( freeFilSession >= 0 )
							{	//有空闲session
								//文件名称
								Aligned_DMABuf char filename[256];
								filename[0] = 0;
								strcat( filename, SDPath );
								strncat( filename, (char*)&(ftpReq->data[1]), ftpReq->size );
								ftpFils[freeFilSession] = new FIL;
								//打开文件
								if( ftpFils[freeFilSession] )
								{	//new文件成功
									FRESULT fres = f_open( ftpFils[freeFilSession], filename, FA_OPEN_EXISTING | FA_READ );
									if( fres == FR_OK )
									{	//打开文件成功
										
										//申请FTPCRC32BUFFERSIZE+DCACHE_SIZE+4的存储空间
										//用于DCACHE对齐 最后4字节存放offset信息
										ftpCrc32Info = new _ftpCrc32Info;
										if( ftpCrc32Info )
										{	//申请内存成功
											uint8_t* crcBuffer = (uint8_t*)( ( (uint32_t)ftpCrc32Info->buffer & ~(uint32_t)0x1f ) + 32 );
											ftpCrc32Info->offset = 0;
											ftpCrc32Info->port = Port_index;
											ftpCrc32Info->targetSysId = msg->sysid;
											ftpCrc32Info->targetCompId = msg->compid;
											ftpCrc32Info->session = freeFilSession;
										}
										else
										{	//申请内存失败
											opErr = opCodeErrFail;
											
											//关闭文件
											f_close(ftpFils[freeFilSession]);
											delete ftpFils[freeFilSession];
											ftpFils[freeFilSession] = 0;
										}
									}
									else
									{	//打开文件失败
										if( fres==FR_NO_FILE || fres==FR_NO_PATH )
											opErr = opCodeErrFileNotFound;
										else
											opErr = opCodeErrFail;
										//删除新建的文件
										delete ftpFils[freeFilSession];
										ftpFils[freeFilSession] = 0;
									}
								}
								else	//new文件失败
									opErr = opCodeErrNoSessionsAvailable;
							}
							else
							{	//无可用session
								opErr = opCodeErrNoSessionsAvailable;
							}
						}
						else
						{	//sd卡初始化失败或当前crc计算不空闲
							opErr = opCodeErrFail;
						}
						
						if( opErr == opCodeErrNone )
						{	//无错误
							//在task分段计算crc结果
							//防止卡死
//							ftpRes.opCode = opCodeRspAck;
//							ftpRes.size = 4;
//							*(uint32_t*)ftpRes.data = crc;
						}
						else
						{	//错误返回错误消息
							ftpRes.opCode = opCodeRspNak;
							ftpRes.size = 1;
							ftpRes.data[0] = opErr;
							
							mavlink_message_t msg_sd;
							ftpRes.seq = ftpReq->seq+1;
							ftpRes.session = 0;
							ftpRes.offset = 0;
							ftpRes.req_opCode = ftpReq->opCode;
							ftpRes.burst_complete = 0;
							ftpRes.padding = 0;
							
							memset( &ftpRes.data[ftpRes.size], 0, MAX_PAYLOADDATA_SIZE-ftpRes.size );
							if( mavlink_lock_chan( Port_index, 0.01 ) )
							{
								mavlink_msg_file_transfer_protocol_pack_chan( 
									get_CommulinkSysId() ,	//system id
									get_CommulinkCompId() ,	//component id
									Port_index ,	//chan
									&msg_sd,
									0,	//target network
									msg->sysid,	//target system
									msg->compid,	//target component
									(uint8_t*)&ftpRes	//payload
								);
								mavlink_msg_to_send_buffer(port->write, 
																					 port->lock,
																					 port->unlock,
																					 &msg_sd, 0, 0.01);
								mavlink_unlock_chan(Port_index);
							}
						}
					}
					break;
				}
				
				case opCodeBurstReadFile:
				{	//读文件Burst
					const Port* port = get_CommuPort( Port_index );
					if( port!=0 && port->write )
					{
						//回应包
						mavlink_message_t msg_sd;
						ftpRes.seq = ftpReq->seq+1;
						ftpRes.session = ftpReq->session;
						ftpRes.offset = ftpReq->offset;
						ftpRes.req_opCode = ftpReq->opCode;
						ftpRes.burst_complete = 0;
						ftpRes.padding = 0;
						//操作结果
						OpErrorCode opErr = opCodeErrNone;
						if( Get_SD_Init_Success() && ftpReq->size<=MAX_PAYLOADDATA_SIZE )
						{	//sd卡初始化成功
							if( ftpReq->session<FTPMAXFILECOUNT && ftpFils[ftpReq->session] )
							{	//session存在且文件已打开
								//if( ftpReq->size == 4 )
								if(1)
								{	//burst长度数据正确
									ftpBurstInfo* burst = &ftpBurstInfos[ftpReq->session];
									burst->portIndex = Port_index;
									burst->targetSysId = msg->sysid;
									burst->targetCompId = msg->compid;
									burst->offset = ftpReq->offset;
									
									uint8_t cReadData = MAX_PAYLOADDATA_SIZE;
									size_t readSize=0;
									f_lseek( ftpFils[ftpReq->session], burst->offset );
									FRESULT fres = f_read( ftpFils[ftpReq->session], ftpRes.data, cReadData, &readSize );
									if( fres == FR_OK )
									{	//读取文件成功
										ftpRes.offset = burst->offset;
										ftpRes.size = readSize;
										if( f_eof(ftpFils[ftpReq->session]) )
										{
											burst->remain = 0;
											ftpRes.burst_complete = 1;
										}
										else
										{
											burst->offset += readSize;
											size_t filSize = f_size(ftpFils[ftpReq->session]);
											burst->remain = filSize - burst->offset;
											ftpRes.burst_complete = 0;
										}
									}
									else
									{	//读取文件失败
										ftpRes.offset = burst->offset;
										burst->remain = 0;
										ftpRes.burst_complete = 1;
										opErr = opCodeErrFail;
									}
								}
								else
									opErr = opCodeErrFail;
							}
							else
							{	//session不存在或文件未打开
								opErr = opCodeErrInvalidSession;
							}
						}
						else
						{	//sd卡初始化失败或需求的字节数量过多
							opErr = opCodeErrFail;
						}
						
						if( opErr == opCodeErrNone )
						{	//无错误
							ftpRes.opCode = opCodeRspAck;
						}
						else
						{
							ftpRes.opCode = opCodeRspNak;
							ftpRes.size = 1;
							ftpRes.data[0] = opErr;
						}
						memset( &ftpRes.data[ftpRes.size], 0, MAX_PAYLOADDATA_SIZE-ftpRes.size );
						if( mavlink_lock_chan( Port_index, 0.01 ) )
						{
							mavlink_msg_file_transfer_protocol_pack_chan( 
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								Port_index ,	//chan
								&msg_sd,
								0,	//target network
								msg->sysid,	//target system
								msg->compid,	//target component
								(uint8_t*)&ftpRes	//payload
							);
							mavlink_msg_to_send_buffer(port->write, 
																				 port->lock,
																				 port->unlock,
																				 &msg_sd, 0, 0.01);
							mavlink_unlock_chan(Port_index);
						}
					}
					break;
				}
				
				case opCodeRemoveFile:
				{	//删除文件
					const Port* port = get_CommuPort( Port_index );
					if( port!=0 && port->write )
					{
						//回应包
						mavlink_message_t msg_sd;
						ftpRes.seq = ftpReq->seq+1;
						ftpRes.session = 0;
						ftpRes.offset = 0;
						ftpRes.req_opCode = ftpReq->opCode;
						ftpRes.burst_complete = 0;
						ftpRes.padding = 0;
						//操作结果
						OpErrorCode opErr = opCodeErrNone;
						if( Get_SD_Init_Success() && get_is_usb_connected()==false )
						{	//sd卡初始化成功
							
							//文件名称
							char filename[255];
							filename[0] = 0;
							strcat( filename, SDPath );
							strncat( filename, (char*)&(ftpReq->data[1]), ftpReq->size );
							//获取文件信息
							FILINFO fno;
							FRESULT fres = f_stat( filename, &fno );
							if( fres == FR_OK )
							{	//文件(夹)存在
								if( fno.fattrib & AM_DIR )
								{	//这个是文件夹
									opErr = opCodeErrFileNotFound;
								}
								else
								{	//文件存在
									fres = f_unlink(filename);
									if( fres != FR_OK )
										opErr = opCodeErrFail;
								}
							}
							else
							{
								if( fres==FR_NO_FILE || fres==FR_NO_PATH )
									opErr = opCodeErrFileNotFound;
								else
									opErr = opCodeErrFail;
							}
						}
						else
						{	//sd卡初始化失败
							opErr = opCodeErrFail;
						}
						
						if( opErr == opCodeErrNone )
						{	//无错误
							ftpRes.opCode = opCodeRspAck;
							ftpRes.size = 0;
						}
						else
						{
							ftpRes.opCode = opCodeRspNak;
							ftpRes.size = 1;
							ftpRes.data[0] = opErr;
						}
						memset( &ftpRes.data[ftpRes.size], 0, MAX_PAYLOADDATA_SIZE-ftpRes.size );
						if( mavlink_lock_chan( Port_index, 0.01 ) )
						{
							mavlink_msg_file_transfer_protocol_pack_chan( 
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								Port_index ,	//chan
								&msg_sd,
								0,	//target network
								msg->sysid,	//target system
								msg->compid,	//target component
								(uint8_t*)&ftpRes	//payload
							);
							mavlink_msg_to_send_buffer(port->write, 
																				 port->lock,
																				 port->unlock,
																				 &msg_sd, 0, 0.01);
							mavlink_unlock_chan(Port_index);
						}
					}
					break;
				}
				
				default:
				{
					const Port* port = get_CommuPort( Port_index );
					if( port!=0 && port->write )
					{
						//回应包
						mavlink_message_t msg_sd;
						ftpRes.seq = ftpReq->seq+1;
						ftpRes.session = 0;
						ftpRes.offset = 0;
						ftpRes.req_opCode = ftpReq->opCode;
						ftpRes.burst_complete = 0;
						ftpRes.padding = 0;
						ftpRes.opCode = opCodeRspNak;
						ftpRes.size = 1;
						ftpRes.data[0] = opCodeErrUnknownCommand;
						memset( &ftpRes.data[ftpRes.size], 0, MAX_PAYLOADDATA_SIZE-ftpRes.size );
						if( mavlink_lock_chan( Port_index, 0.01 ) )
						{
							mavlink_msg_file_transfer_protocol_pack_chan( 
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								Port_index ,	//chan
								&msg_sd,
								0,	//target network
								msg->sysid,	//target system
								msg->compid,	//target component
								(uint8_t*)&ftpRes	//payload
							);
							mavlink_msg_to_send_buffer(port->write, 
																				 port->lock,
																				 port->unlock,
																				 &msg_sd, 0, 0.01);
							mavlink_unlock_chan(Port_index);
						}
					}
					break;
				}
			}
		}
	}
/*FTP协议*/

/*参数协议*/
	static void Msg20_PARAM_REQUEST_READ( uint8_t Port_index , const mavlink_message_t* msg )
	{
		const mavlink_param_request_read_t* msg_rd = (mavlink_param_request_read_t*)msg->payload64;
		if( msg_rd->target_system==0 || 
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==get_CommulinkCompId()) ||
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==0) )
		{
			if( msg_rd->param_index < 0 )
			{
				uint32_t index;	MAV_PARAM_TYPE param_type; uint64_t param_value;
				if( ReadParam( msg_rd->param_id, &index, &param_type, &param_value, 0 ) == PR_OK )
				{
					const Port* port = get_CommuPort( Port_index );
					
					if( port != 0 )
					{
						//参数值
						float value = *(float*)&param_value;
						//参数个数
						uint32_t params_count;
						GetParametersCount(&params_count);
						
						mavlink_message_t msg_sd;
						if( mavlink_lock_chan( Port_index, 0.01 ) )
						{
							mavlink_msg_param_value_pack_chan( 
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								Port_index ,	//chan
								&msg_sd,
								msg_rd->param_id,	//param id
								value ,	//param value
								param_type ,	//param type
								params_count ,	//param count
								index	//param index
							);
							mavlink_msg_to_send_buffer(port->write, 
																				 port->lock,
																				 port->unlock,
																				 &msg_sd, 0, 0.01);
							mavlink_unlock_chan(Port_index);
						}
					}
				}
			}
			else
			{
				SName param_name;	MAV_PARAM_TYPE param_type; uint64_t param_value;
				if( ReadParam( (uint32_t)msg_rd->param_index, &param_name, &param_type, &param_value, 0 ) == PR_OK )
				{
					const Port* port = get_CommuPort( Port_index );
					
					if( port && port->write )
					{
						//参数名
						char pname[17];
						param_name.get_CharStr(pname);
						//参数值					
						float value = *(float*)&param_value;
						//参数个数
						uint32_t params_count;
						GetParametersCount(&params_count);
						
						mavlink_message_t msg_sd;
						if( mavlink_lock_chan( Port_index, 0.01 ) )
						{
							mavlink_msg_param_value_pack_chan( 
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								Port_index ,	//chan
								&msg_sd,
								pname,	//param id
								value ,	//param value
								param_type ,	//param type
								params_count ,	//param count
								msg_rd->param_index	//param index
							);
							mavlink_msg_to_send_buffer(port->write, 
																				 port->lock,
																				 port->unlock,
																				 &msg_sd, 0, 0.01);
							mavlink_unlock_chan(Port_index);
						}
					}
				}
			}
		}
	}

  
	static TIME message_send_time(1);
	static parametersIterator paramsIterator[MAVLINK_COMM_NUM_BUFFERS];
	void paramProtocolTask()
	{
		for( uint8_t i=0; i<MAVLINK_COMM_NUM_BUFFERS; ++i )
		{
			if( paramsIterator[i].available() )
			{
				const Port* port = get_CommuPort(i);
				if( port && port->write )
				{
					while( port->txSpacesAvailable && port->txSpacesAvailable()>384 )
					{
						uint32_t param_ind; SName param_name; MAV_PARAM_TYPE param_type; uint64_t param_value;
						if( paramsIterator[i].read( &param_name, &param_ind, &param_type, &param_value, 0 ) == PR_OK )
						{
							//参数名
							char pname[17];
							param_name.get_CharStr(pname);
							//参数值
							float value = *(float*)&param_value;
							//参数个数
							uint32_t params_count;
							GetParametersCount(&params_count);
							
							if( mavlink_lock_chan( i, 0.01 ) )
							{
								mavlink_message_t msg_sd;
								mavlink_msg_param_value_pack_chan( 
									get_CommulinkSysId() ,	//system id
									get_CommulinkCompId() ,	//component id
									i ,	//chan
									&msg_sd,
									pname,	//param id
									value ,	//param value
									param_type ,	//param type
									params_count ,	//param count
									param_ind	//param index
								);
								uint16_t writeLen = mavlink_msg_to_send_buffer( port->write, 
																																port->lock,
																																port->unlock,
																																&msg_sd, 0, 0.01 );
								mavlink_unlock_chan(i);
								
								if( writeLen )
									++paramsIterator[i];
								else
									break;
							}
							else	//锁定通道失败
								break;
							if( paramsIterator[i].isEnd() )
							{
								paramsIterator[i].setInAvailable();
								break;
							}
						}
						else	//参数读取失败
							paramsIterator[i].setInAvailable();
					}
				}
			}
		}
	}
	static void Msg21_PARAM_REQUEST_LIST( uint8_t Port_index , const mavlink_message_t* msg )
	{	
		const mavlink_param_request_list_t* msg_rd = (mavlink_param_request_list_t*)msg->payload64;
		if( msg_rd->target_system==0 || 
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==get_CommulinkCompId()) ||
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==0) )
		{
			char text[100];
			IMU_Sensor imu_sensors;
			Position_Sensor pos_sensors;
			if(	message_send_time.get_pass_time() > 2	)
			{
				for( uint8_t j = 0 ; j < 5 ; ++j )
				{
					if( j == 0 ){
						sprintf(text,"Welcome to ACFLY!");
					}else if( j == 1 ){
						float Firmware_V[2]={0};					
						ReadParam( "Init_Firmware_V", 0, 0, (uint64_t*)Firmware_V, 0 );
						sprintf(text,"Firware Version:A9 %g - official",Firmware_V[0]);
					}else if( j == 2 ){
						if( !GetAccelerometer(0, &imu_sensors) )
							continue;	
						sprintf(text,"IMU:");
						imu_sensors.name.get_CharStr(&text[4]);				
					}else if( j == 3 ){
						if( !GetMagnetometer(Internal_Magnetometer_Index, &imu_sensors) )
							continue;
						sprintf(text,"MagInternal:");
						imu_sensors.name.get_CharStr(&text[12]);
					}else if( j == 4 ){
						if( !GetMagnetometer(External_Magnetometer_Index, &imu_sensors) )
							continue;
						sprintf(text,"MagExternal:");
						imu_sensors.name.get_CharStr(&text[12]);				
					}
											
					const Port* port = get_CommuPort(Port_index);
					if( port && port->write )
					{	
						mavlink_message_t msg_sd;
						if(mavlink_lock_chan( Port_index, 0.01 )){
							mavlink_msg_statustext_pack_chan( 
							get_CommulinkSysId() ,	//system id
							get_CommulinkCompId() ,	//component id
							Port_index ,	//chan
							&msg_sd,
							MAV_SEVERITY_INFO,
							text,
							0,0
							);				
							mavlink_msg_to_send_buffer(port->write, 
								 port->lock,
								 port->unlock,
								 &msg_sd, 0, 0.01);
							mavlink_unlock_chan(Port_index);	
						}
					}
				}
				message_send_time = TIME::now();
			}
			if( Port_index < MAVLINK_COMM_NUM_BUFFERS )
				paramsIterator[Port_index].reset();
		}
	}

	static void Msg23_PARAM_SET( uint8_t Port_index , const mavlink_message_t* msg )
	{
		const mavlink_param_set_t* msg_rd = (mavlink_param_set_t*)msg->payload64;
		if( msg_rd->target_system==0 || 
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==get_CommulinkCompId()) ||
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==0) )
		{
			uint32_t index;	MAV_PARAM_TYPE param_type; uint64_t param_value;
			if( ReadParam( msg_rd->param_id, &index, &param_type, &param_value, 0 ) == PR_OK )
			{
				float value_f = msg_rd->param_value;
				uint32_t value = *(uint32_t*)&value_f;
				
				if(!strcmp(msg_rd->param_id,"Init_Boot_Count") || !strcmp(msg_rd->param_id,"Init_Firmware_V"))
				{
					return;
				}
				
				if( UpdateParam( msg_rd->param_id, value ) == PR_OK )
				{		
					//参数个数
					uint32_t params_count;
					GetParametersCount(&params_count);
					
					//向每个端口发送PARAM_VALUE
					for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
					{
						const Port* port = get_CommuPort(i);
						if( port->write != 0 )
						{	
							mavlink_message_t msg_sd;
							if( mavlink_lock_chan( i, 0.01 ) )
							{
								mavlink_msg_param_value_pack_chan( 
									get_CommulinkSysId() ,	//system id
									get_CommulinkCompId() ,	//component id
									i ,	//chan
									&msg_sd,
									msg_rd->param_id,	//param id
									msg_rd->param_value ,	//param value
									param_type ,	//param type
									params_count ,	//param count
									index	//param index
								);				
								mavlink_msg_to_send_buffer(port->write, 
																					 port->lock,
																					 port->unlock,
																					 &msg_sd, 0, 0.01);
								mavlink_unlock_chan(i);
							}
						}
					}
				}
			}
		}
	}
/*参数协议*/
	
/*航点任务协议*/
	static void Msg43_MISSION_REQUEST_LIST( uint8_t Port_index , const mavlink_message_t* msg )
	{
		const mavlink_mission_request_list_t* msg_rd = (mavlink_mission_request_list_t*)msg->payload64;
		if( msg_rd->target_system==0 || 
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==get_CommulinkCompId()) ||
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==0) )
		{
			const Port* port = get_CommuPort(Port_index);
			if( port->write != 0 )
			{
				mavlink_message_t msg_sd;
				if( mavlink_lock_chan( Port_index, 0.01 ) )
				{
					switch(msg_rd->mission_type)
					{
						default:
						case MAV_MISSION_TYPE_MISSION:
						{
							mavlink_msg_mission_count_pack_chan( 
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								Port_index ,	//chan
								&msg_sd,
								msg->sysid ,	//target system
								msg->compid ,	//target component
								getMissionsCount() ,	//count
								MAV_MISSION_TYPE_MISSION	//mission type
							);
							break;
						}
						case MAV_MISSION_TYPE_FENCE:
						{
							mavlink_msg_mission_count_pack_chan( 
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								Port_index ,	//chan
								&msg_sd,
								msg->sysid ,	//target system
								msg->compid ,	//target component
								getFencesCount() ,	//count
								MAV_MISSION_TYPE_FENCE	//mission type
							);
							break;
						}
					}
					mavlink_msg_to_send_buffer(port->write, 
																		 port->lock,
																		 port->unlock,
																		 &msg_sd, 0, 0.01);
					
					mavlink_unlock_chan(Port_index);
				}
			}
		}
	}
	
	static void Msg40_MISSION_REQUEST( uint8_t Port_index , const mavlink_message_t* msg )
	{
		const mavlink_mission_request_t* msg_rd = (mavlink_mission_request_t*)msg->payload64;
		if( msg_rd->target_system==0 || 
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==get_CommulinkCompId()) ||
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==0) )
		{
			const Port* port = get_CommuPort(Port_index);
			if( port->write != 0 )
			{
				switch(msg_rd->mission_type)
				{
					default:
					case MAV_MISSION_TYPE_MISSION:
					{
						MissionInf wp_inf;
						if( ReadMission( msg_rd->seq, &wp_inf ) )
						{		
			//				mavlink_message_t msg_sd;
			//				mavlink_lock_chan( Port_index, 0.01 );
			//				mavlink_msg_mission_item_pack_chan( 
			//					get_CommulinkSysId() ,	//system id
			//					get_CommulinkCompId() ,	//component id
			//					Port_index ,	//chan
			//					&msg_sd,
			//					msg->sysid ,	//target system
			//					msg->compid ,	//target component
			//					msg_rd->seq ,	//seq
			//					wp_inf.frame, //frame
			//					wp_inf.cmd , //command
			//					(msg_rd->seq==getCurrentMissionInd()) ? 1:0 ,	//current
			//					wp_inf.autocontinue ,	//autocontinue
			//					wp_inf.params[0] ,	//hold time(decimal seconds)
			//					wp_inf.params[1] ,	//acceptance radius in meters(hit waypoint)
			//					wp_inf.params[2] ,	//radius in meters to pass the waypoint
			//					wp_inf.params[3] ,	//desired yaw angle
			//					wp_inf.params[4], wp_inf.params[5], wp_inf.params[6], //x y z
			//					MAV_MISSION_TYPE_MISSION
			//				);				
			//				mavlink_msg_to_send_buffer(port->write, 
			//																	 port->lock,
			//																	 port->unlock,
			//																	 &msg_sd, 0, 0.01);
			//				mavlink_unlock_chan(Port_index);


							double x, y;
							x = wp_inf.params[4]*1e7;
							y = wp_inf.params[5]*1e7;
							
							mavlink_message_t msg_sd;
							if( mavlink_lock_chan( Port_index, 0.01 ) )
							{
								mavlink_msg_mission_item_int_pack_chan( 
									get_CommulinkSysId() ,	//system id
									get_CommulinkCompId() ,	//component id
									Port_index ,	//chan
									&msg_sd,
									msg->sysid ,	//target system
									msg->compid ,	//target component
									msg_rd->seq ,	//seq
									wp_inf.frame , //frame
									wp_inf.cmd , //command
									(msg_rd->seq==getCurrentMissionInd()) ? 1:0 ,	//current
									wp_inf.autocontinue ,	//autocontinue
									wp_inf.params[0] ,	//hold time(decimal seconds)
									wp_inf.params[1] ,	//acceptance radius in meters(hit waypoint)
									wp_inf.params[2] ,	//radius in meters to pass the waypoint
									wp_inf.params[3] ,	//desired yaw angle
									x, y, wp_inf.params[6], //x y z
									MAV_MISSION_TYPE_MISSION
								);
								mavlink_msg_to_send_buffer(port->write, 
																					 port->lock,
																					 port->unlock,
																					 &msg_sd, 0, 0.01);
								mavlink_unlock_chan(Port_index);
							}
						}
						break;
					}	//mission
					
					case MAV_MISSION_TYPE_FENCE:
					{
						FenceInf fence_inf;
						if( ReadFence( msg_rd->seq, &fence_inf ) )
						{
							double x, y;
							x = fence_inf.params[4]*1e7;
							y = fence_inf.params[5]*1e7;
							
							mavlink_message_t msg_sd;
							if( mavlink_lock_chan( Port_index, 0.01 ) )
							{
								mavlink_msg_mission_item_int_pack_chan( 
									get_CommulinkSysId() ,	//system id
									get_CommulinkCompId() ,	//component id
									Port_index ,	//chan
									&msg_sd,
									msg->sysid ,	//target system
									msg->compid ,	//target component
									msg_rd->seq ,	//seq
									fence_inf.frame , //frame
									fence_inf.cmd , //command
									(msg_rd->seq==getCurrentMissionInd()) ? 1:0 ,	//current
									fence_inf.autocontinue ,	//autocontinue
									fence_inf.params[0] ,	//hold time(decimal seconds)
									fence_inf.params[1] ,	//acceptance radius in meters(hit waypoint)
									fence_inf.params[2] ,	//radius in meters to pass the waypoint
									fence_inf.params[3] ,	//desired yaw angle
									x, y, fence_inf.params[6], //x y z
									MAV_MISSION_TYPE_FENCE
								);
								mavlink_msg_to_send_buffer(port->write, 
																					 port->lock,
																					 port->unlock,
																					 &msg_sd, 0, 0.01);
								mavlink_unlock_chan(Port_index);
							}
						}	//fence
						break;
					}
				}
			}
		}
	}
	
	static void Msg51_MISSION_REQUEST_INT( uint8_t Port_index , const mavlink_message_t* msg )
	{
		const mavlink_mission_request_int_t* msg_rd = (mavlink_mission_request_int_t*)msg->payload64;
		if( msg_rd->target_system==0 || 
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==get_CommulinkCompId()) ||
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==0) )
		{
			const Port* port = get_CommuPort(Port_index);
			if( port->write != 0 )
			{
				switch(msg_rd->mission_type)
				{
					default:
					case MAV_MISSION_TYPE_MISSION:
					{
						MissionInf wp_inf;
						if( ReadMission( msg_rd->seq, &wp_inf ) )
						{		
							double x, y;
							x = wp_inf.params[4]*1e7;
							y = wp_inf.params[5]*1e7;
							
							mavlink_message_t msg_sd;
							if( mavlink_lock_chan( Port_index, 0.01 ) )
							{
								mavlink_msg_mission_item_int_pack_chan( 
									get_CommulinkSysId() ,	//system id
									get_CommulinkCompId() ,	//component id
									Port_index ,	//chan
									&msg_sd,
									msg->sysid ,	//target system
									msg->compid ,	//target component
									msg_rd->seq ,	//seq
									wp_inf.frame , //frame
									wp_inf.cmd , //command
									(msg_rd->seq==getCurrentMissionInd()) ? 1:0 ,	//current
									wp_inf.autocontinue ,	//autocontinue
									wp_inf.params[0] ,	//hold time(decimal seconds)
									wp_inf.params[1] ,	//acceptance radius in meters(hit waypoint)
									wp_inf.params[2] ,	//radius in meters to pass the waypoint
									wp_inf.params[3] ,	//desired yaw angle
									x, y, wp_inf.params[6], //x y z
									MAV_MISSION_TYPE_MISSION
								);				
								mavlink_msg_to_send_buffer(port->write, 
																					 port->lock,
																					 port->unlock,
																					 &msg_sd, 0, 0.01);
								mavlink_unlock_chan(Port_index);
							}
						}
						break;
					}	//mission
					
					case MAV_MISSION_TYPE_FENCE:
					{
						FenceInf fence_inf;
						if( ReadFence( msg_rd->seq, &fence_inf ) )
						{
							double x, y;
							x = fence_inf.params[4]*1e7;
							y = fence_inf.params[5]*1e7;
							
							mavlink_message_t msg_sd;
							if( mavlink_lock_chan( Port_index, 0.01 ) )
							{
								mavlink_msg_mission_item_int_pack_chan( 
									get_CommulinkSysId() ,	//system id
									get_CommulinkCompId() ,	//component id
									Port_index ,	//chan
									&msg_sd,
									msg->sysid ,	//target system
									msg->compid ,	//target component
									msg_rd->seq ,	//seq
									fence_inf.frame , //frame
									fence_inf.cmd , //command
									(msg_rd->seq==getCurrentMissionInd()) ? 1:0 ,	//current
									fence_inf.autocontinue ,	//autocontinue
									fence_inf.params[0] ,	//hold time(decimal seconds)
									fence_inf.params[1] ,	//acceptance radius in meters(hit waypoint)
									fence_inf.params[2] ,	//radius in meters to pass the waypoint
									fence_inf.params[3] ,	//desired yaw angle
									x, y, fence_inf.params[6], //x y z
									MAV_MISSION_TYPE_FENCE
								);
								mavlink_msg_to_send_buffer(port->write, 
																					 port->lock,
																					 port->unlock,
																					 &msg_sd, 0, 0.01);
								mavlink_unlock_chan(Port_index);
							}
						}
						break;
					}	//fence
				}
			}
		}
	}
	
	static void Msg41_MISSION_SET_CURRENT( uint8_t Port_index , const mavlink_message_t* msg )
	{
		const mavlink_mission_set_current_t* msg_rd = (mavlink_mission_set_current_t*)msg->payload64;
		if( msg_rd->target_system==0 || 
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==get_CommulinkCompId()) ||
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==0) )
		{
			if( setCurrentMission(msg_rd->seq) )
			{
				//向每个端口发送MISSION_CURRENT
				for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
				{
					const Port* port = get_CommuPort(i);
					if( port->write != 0 )
					{
						mavlink_message_t msg_sd;
						if( mavlink_lock_chan( i, 0.01 ) )
						{
							uint16_t flag = 0;
							
							bool inFlight;
							get_is_inFlight(&inFlight);
							extern bool inMissionFlight;
							if( getCurrentFlyMode()==AFunc_Mission && inFlight && inMissionFlight )
								flag |= MAV_MISSION_CURRENT_IS_IN_MISSION_FLIGHT;
							
							RestoreWpInf restoreWpInf;
							uint16_t resumeSeq = 0;
							vector3<float> vec;
							float line_fs = 0;
							float trigInt = 0;
							float speed = 0;
							float radius = 0;
							if( ReadParamGroup( "RestoreWpInf", (uint64_t*)&restoreWpInf, 0 ) == PR_OK )
							{
								if( restoreWpInf.CurrentWp[0] > 0 )
								{
									flag |= MAV_MISSION_CURRENT_IS_RESUME_WAYPOINT_EXIST;
									
									resumeSeq = restoreWpInf.CurrentWp[0];
									vec.x = restoreWpInf.line_y*0.01;
									vec.y = restoreWpInf.line_x*0.01;
									vec.z = -restoreWpInf.line_z*0.01;
									line_fs = restoreWpInf.line_fs*0.01;
									trigInt = restoreWpInf.CamTrigDist*0.01;
									speed = restoreWpInf.vel*0.01;
									radius = restoreWpInf.wpR*0.01;
								}
							}
							
							mavlink_msg_mission_current_pack_chan(
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								i ,	//chan
								&msg_sd,
								getCurrentMissionInd(),	//current mission ind
								resumeSeq,	//resume ind
								flag,	//flag
								0,	//heading
								vec.x, vec.y, vec.z,
								line_fs,
								trigInt,
								speed,
								radius
							);
							mavlink_msg_to_send_buffer(port->write, 
																				 port->lock,
																				 port->unlock,
																				 &msg_sd, 0, 0.01);
							
							
							
							
							
							
							mavlink_unlock_chan(i);
						}
					}
				}
			}
		}
	}
	
	//下载的任务数量
	static uint16_t DnlMissionsCount = 0;
	//任务超时再次请求变量
	bool RqMissionInt[MAVLINK_COMM_NUM_BUFFERS];
	int32_t RqMissionInd[MAVLINK_COMM_NUM_BUFFERS];
	int32_t RqMissionCounter[MAVLINK_COMM_NUM_BUFFERS] = {0};
	uint8_t RqMissiontarget_sysid[MAVLINK_COMM_NUM_BUFFERS];
	uint8_t RqMissiontarget_compid[MAVLINK_COMM_NUM_BUFFERS];
	
	//下载的围栏数量
	static uint16_t DnlFencesCount = 0;
	//围栏超时再次请求变量
	bool RqFenceInt[MAVLINK_COMM_NUM_BUFFERS];
	int32_t RqFenceInd[MAVLINK_COMM_NUM_BUFFERS];
	int32_t RqFenceCounter[MAVLINK_COMM_NUM_BUFFERS] = {0};
	uint8_t RqFencetarget_sysid[MAVLINK_COMM_NUM_BUFFERS];
	uint8_t RqFencetarget_compid[MAVLINK_COMM_NUM_BUFFERS];
	
	static void Msg44_MISSION_COUNT( uint8_t Port_index , const mavlink_message_t* msg )
	{
		const mavlink_mission_count_t* msg_rd = (mavlink_mission_count_t*)msg->payload64;
		if( msg_rd->target_system==0 || 
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==get_CommulinkCompId()) ||
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==0) )
		{
			const Port* port = get_CommuPort(Port_index);
			if( port->write != 0 )
			{
				switch(msg_rd->mission_type)
				{
					default:
					case MAV_MISSION_TYPE_MISSION:
					{
						DnlMissionsCount = msg_rd->count;
						if( DnlMissionsCount > 0 )
						{
							clearMissions();
							
							mavlink_message_t msg_sd;
							if( mavlink_lock_chan( Port_index, 0.01 ) )
							{
								mavlink_msg_mission_request_int_pack_chan( 
									get_CommulinkSysId() ,	//system id
									get_CommulinkCompId() ,	//component id
									Port_index ,	//chan
									&msg_sd,
									msg->sysid ,	//target system
									msg->compid ,	//target component
									0 ,	//seq
									MAV_MISSION_TYPE_MISSION	//mission type
								
								);
								mavlink_msg_to_send_buffer(port->write, 
																					 port->lock,
																					 port->unlock,
																					 &msg_sd, 0, 0.01);

								mavlink_msg_mission_request_pack_chan( 
									get_CommulinkSysId() ,	//system id
									get_CommulinkCompId() ,	//component id
									Port_index ,	//chan
									&msg_sd,
									msg->sysid ,	//target system
									msg->compid ,	//target component
									0 ,	//seq
									MAV_MISSION_TYPE_MISSION	//mission type
								
								);
								mavlink_msg_to_send_buffer(port->write, 
																					 port->lock,
																					 port->unlock,
																					 &msg_sd, 0, 0.01);
								mavlink_unlock_chan(Port_index);
							}
							
							//超时请求
							RqMissionCounter[Port_index] = 100;
							RqMissionInd[Port_index] = 0;
							RqMissiontarget_sysid[Port_index] = msg->sysid;
							RqMissiontarget_compid[Port_index] = msg->compid;
							
							--DnlMissionsCount;
						}
						else
						{
							mavlink_message_t msg_sd;
							if( mavlink_lock_chan( Port_index, 0.01 ) )
							{
								mavlink_msg_mission_ack_pack_chan( 
									get_CommulinkSysId() ,	//system id
									get_CommulinkCompId() ,	//component id
									Port_index ,	//chan
									&msg_sd,
									msg->sysid ,	//target system
									msg->compid ,	//target component
									MAV_MISSION_ACCEPTED ,	//type
									MAV_MISSION_TYPE_MISSION	//mission type						
								);
								mavlink_msg_to_send_buffer(port->write, 
																					 port->lock,
																					 port->unlock,
																					 &msg_sd, 0, 0.01);
								mavlink_unlock_chan(Port_index);
							}
						}
						break;
					}	//mission
					
					case MAV_MISSION_TYPE_FENCE:
					{
						DnlFencesCount = msg_rd->count;
						if( DnlFencesCount > 0 )
						{
							clearFences();
							
							mavlink_message_t msg_sd;
							if( mavlink_lock_chan( Port_index, 0.01 ) )
							{
								mavlink_msg_mission_request_int_pack_chan( 
									get_CommulinkSysId() ,	//system id
									get_CommulinkCompId() ,	//component id
									Port_index ,	//chan
									&msg_sd,
									msg->sysid ,	//target system
									msg->compid ,	//target component
									0 ,	//seq
									MAV_MISSION_TYPE_FENCE	//mission type
								
								);
								mavlink_msg_to_send_buffer(port->write, 
																					 port->lock,
																					 port->unlock,
																					 &msg_sd, 0, 0.01);

								mavlink_msg_mission_request_pack_chan( 
									get_CommulinkSysId() ,	//system id
									get_CommulinkCompId() ,	//component id
									Port_index ,	//chan
									&msg_sd,
									msg->sysid ,	//target system
									msg->compid ,	//target component
									0 ,	//seq
									MAV_MISSION_TYPE_FENCE	//mission type
								
								);
								mavlink_msg_to_send_buffer(port->write, 
																					 port->lock,
																					 port->unlock,
																					 &msg_sd, 0, 0.01);
								mavlink_unlock_chan(Port_index);
							}
							
							//超时请求
							RqFenceCounter[Port_index] = 100;
							RqFenceInd[Port_index] = 0;
							RqFencetarget_sysid[Port_index] = msg->sysid;
							RqFencetarget_compid[Port_index] = msg->compid;
							
							--DnlFencesCount;
						}
						else
						{
							mavlink_message_t msg_sd;
							if( mavlink_lock_chan( Port_index, 0.01 ) )
							{
								mavlink_msg_mission_ack_pack_chan( 
									get_CommulinkSysId() ,	//system id
									get_CommulinkCompId() ,	//component id
									Port_index ,	//chan
									&msg_sd,
									msg->sysid ,	//target system
									msg->compid ,	//target component
									MAV_MISSION_ACCEPTED ,	//type
									MAV_MISSION_TYPE_FENCE	//mission type						
								);
								mavlink_msg_to_send_buffer(port->write, 
																					 port->lock,
																					 port->unlock,
																					 &msg_sd, 0, 0.01);
								mavlink_unlock_chan(Port_index);
							}
						}
						break;
					}	//fence
				}
			}
		}
	}
	
	static void Msg39_MISSION_ITEM( uint8_t Port_index , const mavlink_message_t* msg )
	{
		const mavlink_mission_item_t* msg_rd = (mavlink_mission_item_t*)msg->payload64;
		if( msg_rd->target_system==0 || 
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==get_CommulinkCompId()) ||
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==0) )
		{
			switch(msg_rd->mission_type)
			{
				default:
				case MAV_MISSION_TYPE_MISSION:
				{
					if( msg_rd->seq == getUploadingMissionsCount() )
					{
						//添加航点任务
						MissionInf wp_inf;
						wp_inf.cmd = msg_rd->command;
						wp_inf.frame = msg_rd->frame;
						wp_inf.autocontinue = msg_rd->autocontinue;
						wp_inf.params[0] = msg_rd->param1;
						wp_inf.params[1] = msg_rd->param2;
						wp_inf.params[2] = msg_rd->param3;
						wp_inf.params[3] = msg_rd->param4;
						wp_inf.params[4] = msg_rd->x;
						wp_inf.params[5] = msg_rd->y;
						wp_inf.params[6] = msg_rd->z;
						addMission( wp_inf, false );
						
						const Port* port = get_CommuPort(Port_index);
						if( port->write != 0 )
						{		
							if( DnlMissionsCount > 0 )
							{	
								--DnlMissionsCount;
								//发送航点申请
								mavlink_message_t msg_sd;
								if( mavlink_lock_chan( Port_index, 0.01 ) )
								{
									mavlink_msg_mission_request_pack_chan( 
										get_CommulinkSysId() ,	//system id
										get_CommulinkCompId() ,	//component id
										Port_index ,	//chan
										&msg_sd,
										msg->sysid ,	//target system
										msg->compid ,	//target component
										getUploadingMissionsCount() ,	//seq
										MAV_MISSION_TYPE_MISSION	//mission type					
									);
									mavlink_msg_to_send_buffer(port->write, 
																						 port->lock,
																						 port->unlock,
																						 &msg_sd, 0, 0.01);
									mavlink_unlock_chan(Port_index);
								}

								//超时请求
								RqMissionCounter[Port_index] = 100;
								RqMissionInd[Port_index] = getUploadingMissionsCount();
								RqMissionInt[Port_index] = false;
								RqMissiontarget_sysid[Port_index] = msg->sysid;
								RqMissiontarget_compid[Port_index] = msg->compid;
							}
							else
							{	//发送ACK			
								mavlink_message_t msg_sd;
								if( mavlink_lock_chan( Port_index, 0.01 ) )
								{
									mavlink_msg_mission_ack_pack_chan( 
										get_CommulinkSysId() ,	//system id
										get_CommulinkCompId() ,	//component id
										Port_index ,	//chan
										&msg_sd,
										msg->sysid ,	//target system
										msg->compid ,	//target component
										MAV_MISSION_ACCEPTED ,	//type
										MAV_MISSION_TYPE_MISSION	//mission type						
									);
									mavlink_msg_to_send_buffer(port->write, 
																						 port->lock,
																						 port->unlock,
																						 &msg_sd, 0, 0.01);
									mavlink_unlock_chan(Port_index);
								}
								
								//复位超时请求
								RqMissionCounter[Port_index] = 0;
								
								//保存任务到存储器
								saveMissions();
							}
						}
					}
					break;
				}	//mission
				
				case MAV_MISSION_TYPE_FENCE:
				{
					if( msg_rd->seq == getUploadingFencesCount() )
					{
						//添加航点任务
						FenceInf wp_inf;
						wp_inf.cmd = msg_rd->command;
						wp_inf.frame = msg_rd->frame;
						wp_inf.autocontinue = msg_rd->autocontinue;
						wp_inf.params[0] = msg_rd->param1;
						wp_inf.params[1] = msg_rd->param2;
						wp_inf.params[2] = msg_rd->param3;
						wp_inf.params[3] = msg_rd->param4;
						wp_inf.params[4] = msg_rd->x;
						wp_inf.params[5] = msg_rd->y;
						wp_inf.params[6] = msg_rd->z;
						addFence( wp_inf, false );
						
						const Port* port = get_CommuPort(Port_index);
						if( port->write != 0 )
						{		
							if( DnlFencesCount > 0 )
							{	
								--DnlFencesCount;
								//发送航点申请
								mavlink_message_t msg_sd;
								if( mavlink_lock_chan( Port_index, 0.01 ) )
								{
									mavlink_msg_mission_request_pack_chan( 
										get_CommulinkSysId() ,	//system id
										get_CommulinkCompId() ,	//component id
										Port_index ,	//chan
										&msg_sd,
										msg->sysid ,	//target system
										msg->compid ,	//target component
										getUploadingFencesCount() ,	//seq
										MAV_MISSION_TYPE_FENCE	//mission type					
									);
									mavlink_msg_to_send_buffer(port->write, 
																						 port->lock,
																						 port->unlock,
																						 &msg_sd, 0, 0.01);
									mavlink_unlock_chan(Port_index);
								}

								//超时请求
								RqFenceCounter[Port_index] = 100;
								RqFenceInd[Port_index] = getUploadingFencesCount();
								RqFenceInt[Port_index] = false;
								RqFencetarget_sysid[Port_index] = msg->sysid;
								RqFencetarget_compid[Port_index] = msg->compid;
							}
							else
							{	//发送ACK			
								mavlink_message_t msg_sd;
								if( mavlink_lock_chan( Port_index, 0.01 ) )
								{
									mavlink_msg_mission_ack_pack_chan( 
										get_CommulinkSysId() ,	//system id
										get_CommulinkCompId() ,	//component id
										Port_index ,	//chan
										&msg_sd,
										msg->sysid ,	//target system
										msg->compid ,	//target component
										MAV_MISSION_ACCEPTED ,	//type
										MAV_MISSION_TYPE_FENCE	//mission type						
									);
									mavlink_msg_to_send_buffer(port->write, 
																						 port->lock,
																						 port->unlock,
																						 &msg_sd, 0, 0.01);
									mavlink_unlock_chan(Port_index);
								}
								
								//复位超时请求
								RqFenceCounter[Port_index] = 0;
								
								//保存任务到存储器
								saveFences();
							}
						}
					}
					break;
				}	//fence
			}
		}
	}
	
	static void Msg73_MISSION_ITEM_INT( uint8_t Port_index , const mavlink_message_t* msg )
	{
		const mavlink_mission_item_int_t* msg_rd = (mavlink_mission_item_int_t*)msg->payload64;
		if( msg_rd->target_system==0 || 
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==get_CommulinkCompId()) ||
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==0) )
		{
			switch(msg_rd->mission_type)
			{
				default:
				case MAV_MISSION_TYPE_MISSION:
				{
					if( msg_rd->seq == getUploadingMissionsCount() )
					{
						//添加航点任务
						MissionInf wp_inf;
						wp_inf.cmd = msg_rd->command;
						wp_inf.frame = msg_rd->frame;
						wp_inf.autocontinue = msg_rd->autocontinue;
						wp_inf.params[0] = msg_rd->param1;
						wp_inf.params[1] = msg_rd->param2;
						wp_inf.params[2] = msg_rd->param3;
						wp_inf.params[3] = msg_rd->param4;
						wp_inf.params[4] = msg_rd->x*1e-7;
						wp_inf.params[5] = msg_rd->y*1e-7;
						wp_inf.params[6] = msg_rd->z;
						addMission( wp_inf, false );
						
						const Port* port = get_CommuPort(Port_index);
						if( port->write != 0 )
						{		
							if( DnlMissionsCount > 0 )
							{	
								--DnlMissionsCount;
								//发送航点申请
								mavlink_message_t msg_sd;
								if( mavlink_lock_chan( Port_index, 0.01 ) )
								{
									mavlink_msg_mission_request_int_pack_chan( 
										get_CommulinkSysId() ,	//system id
										get_CommulinkCompId() ,	//component id
										Port_index ,	//chan
										&msg_sd,
										msg->sysid ,	//target system
										msg->compid ,	//target component
										getUploadingMissionsCount() ,	//seq
										MAV_MISSION_TYPE_MISSION	//mission type
									
									);
									mavlink_msg_to_send_buffer(port->write, 
																						 port->lock,
																						 port->unlock,
																						 &msg_sd, 0, 0.01);
									mavlink_unlock_chan(Port_index);
								}

								//超时请求
								RqMissionCounter[Port_index] = 100;
								RqMissionInd[Port_index] = getUploadingMissionsCount();
								RqMissionInt[Port_index] = true;
								RqMissiontarget_sysid[Port_index] = msg->sysid;
								RqMissiontarget_compid[Port_index] = msg->compid;
							}
							else
							{	//发送ACK			
								mavlink_message_t msg_sd;
								if( mavlink_lock_chan( Port_index, 0.01 ) )
								{
									mavlink_msg_mission_ack_pack_chan( 
										get_CommulinkSysId() ,	//system id
										get_CommulinkCompId() ,	//component id
										Port_index ,	//chan
										&msg_sd,
										msg->sysid ,	//target system
										msg->compid ,	//target component
										MAV_MISSION_ACCEPTED ,	//type
										MAV_MISSION_TYPE_MISSION	//mission type						
									);
									mavlink_msg_to_send_buffer(port->write, 
																						 port->lock,
																						 port->unlock,
																						 &msg_sd, 0, 0.01);
									mavlink_unlock_chan(Port_index);
								}
								
								//复位超时请求
								RqMissionCounter[Port_index] = 0;
								
								//保存任务到存储器
								saveMissions();
							}
						}
					}
					break;
				}	//mission
				
				case MAV_MISSION_TYPE_FENCE:
				{
					if( msg_rd->seq == getUploadingFencesCount() )
					{
						//添加航点任务
						FenceInf wp_inf;
						wp_inf.cmd = msg_rd->command;
						wp_inf.frame = msg_rd->frame;
						wp_inf.autocontinue = msg_rd->autocontinue;
						wp_inf.params[0] = msg_rd->param1;
						wp_inf.params[1] = msg_rd->param2;
						wp_inf.params[2] = msg_rd->param3;
						wp_inf.params[3] = msg_rd->param4;
						wp_inf.params[4] = msg_rd->x*1e-7;
						wp_inf.params[5] = msg_rd->y*1e-7;
						wp_inf.params[6] = msg_rd->z;
						addFence( wp_inf, false );
						
						const Port* port = get_CommuPort(Port_index);
						if( port->write != 0 )
						{		
							if( DnlFencesCount > 0 )
							{	
								--DnlFencesCount;
								//发送航点申请
								mavlink_message_t msg_sd;
								if( mavlink_lock_chan( Port_index, 0.01 ) )
								{
									mavlink_msg_mission_request_int_pack_chan( 
										get_CommulinkSysId() ,	//system id
										get_CommulinkCompId() ,	//component id
										Port_index ,	//chan
										&msg_sd,
										msg->sysid ,	//target system
										msg->compid ,	//target component
										getUploadingFencesCount() ,	//seq
										MAV_MISSION_TYPE_FENCE	//mission type
									
									);
									mavlink_msg_to_send_buffer(port->write, 
																						 port->lock,
																						 port->unlock,
																						 &msg_sd, 0, 0.01);
									mavlink_unlock_chan(Port_index);
								}

								//超时请求
								RqFenceCounter[Port_index] = 100;
								RqFenceInd[Port_index] = getUploadingFencesCount();
								RqFenceInt[Port_index] = true;
								RqFencetarget_sysid[Port_index] = msg->sysid;
								RqFencetarget_compid[Port_index] = msg->compid;
							}
							else
							{	//发送ACK			
								mavlink_message_t msg_sd;
								if( mavlink_lock_chan( Port_index, 0.01 ) )
								{
									mavlink_msg_mission_ack_pack_chan( 
										get_CommulinkSysId() ,	//system id
										get_CommulinkCompId() ,	//component id
										Port_index ,	//chan
										&msg_sd,
										msg->sysid ,	//target system
										msg->compid ,	//target component
										MAV_MISSION_ACCEPTED ,	//type
										MAV_MISSION_TYPE_FENCE	//mission type						
									);
									mavlink_msg_to_send_buffer(port->write, 
																						 port->lock,
																						 port->unlock,
																						 &msg_sd, 0, 0.01);
									mavlink_unlock_chan(Port_index);
								}
								
								//复位超时请求
								RqFenceCounter[Port_index] = 0;
								
								//保存任务到存储器
								saveFences();
							}
						}
					}
					break;
				}	//fence
			}
		}
	}
	
	static void Msg45_MISSION_CLEAR_ALL( uint8_t Port_index , const mavlink_message_t* msg )
	{
		const mavlink_mission_clear_all_t* msg_rd = (mavlink_mission_clear_all_t*)msg->payload64;
		if( msg_rd->target_system==0 || 
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==get_CommulinkCompId()) ||
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==0) )
		{
			switch(msg_rd->mission_type)
			{
				case MAV_MISSION_TYPE_MISSION:
					clearMissions();
					break;
				case MAV_MISSION_TYPE_FENCE:
					clearFences();
					break;
			}
			
			
			const Port* port = get_CommuPort(Port_index);
			if( port->write != 0 )
			{	//发送ACK			
				mavlink_message_t msg_sd;
				if( mavlink_lock_chan( Port_index, 0.01 ) )
				{
					mavlink_msg_mission_ack_pack_chan( 
						get_CommulinkSysId() ,	//system id
						get_CommulinkCompId() ,	//component id
						Port_index ,	//chan
						&msg_sd,
						msg->sysid ,	//target system
						msg->compid ,	//target component
						MAV_MISSION_ACCEPTED ,	//type
						msg_rd->mission_type	//mission type						
					);
					mavlink_msg_to_send_buffer(port->write, 
																		 port->lock,
																		 port->unlock,
																		 &msg_sd, 0, 0.01);
					mavlink_unlock_chan(Port_index);
				}
			}
		}
	}
/*航点任务协议*/
	
static void Msg66_REQUEST_DATA_STREAM( uint8_t Port_index , const mavlink_message_t* msg )
{
	const mavlink_request_data_stream_t* msg_rd = (mavlink_request_data_stream_t*)msg->payload64;
	if( msg_rd->target_system==0 || 
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==get_CommulinkCompId()) ||
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==0) )
	{
		if( msg_rd->req_message_rate == 0 || msg_rd->start_stop == 0 )
			SetMsgRate( Port_index , msg_rd->req_stream_id , 0 );
		else
			SetMsgRate( Port_index , msg_rd->req_stream_id , msg_rd->req_message_rate );
	}
}

static void Msg69_MANUAL_CONTROL( uint8_t Port_index , const mavlink_message_t* msg )
{
	const mavlink_manual_control_t* msg_rd = (mavlink_manual_control_t*)msg->payload64;
	
	if((msg_rd->enabled_extensions&(1<<7)) == (1<<7))
	{
		float raw_data[16];
		raw_data[0] = msg_rd->x*0.05 + 50;
		raw_data[1] = msg_rd->y*0.05 + 50;
		raw_data[2] = msg_rd->z*0.05 + 50;
		raw_data[3] = msg_rd->r*0.05 + 50;
		if( fabs(msg_rd->x)>1050 || fabs(msg_rd->y)>1050 || fabs(msg_rd->z)>1050 || fabs(msg_rd->r)>1050 )
			ReceiverUpdate( "JoyStickACFLy", false, raw_data, 4, 0.01 );
		else
			ReceiverUpdate( "JoyStickACFLy", true, raw_data, 4, 0.01 );
		return;
	}
	
	float raw_data[16];
	raw_data[0] = msg_rd->x*0.05 + 50;
	raw_data[1] = msg_rd->y*0.05 + 50;
	raw_data[2] = msg_rd->z*0.05 + 50;
	raw_data[3] = msg_rd->r*0.05 + 50;
	
	uint8_t bias = 0;
	if( (msg_rd->enabled_extensions&(1<<2))==(1<<2) )
	{	//扩展通道
		const int16_t* extAxis = &msg_rd->extendedAxis1;
		for( uint8_t i=0; i<4; ++i )
		{
			if( (msg->len>20+i*2) && extAxis[i]!=INT16_MAX )
			{
				raw_data[4+bias] = extAxis[i]*0.05 + 50;
				++bias;
			}
		}
	}
	for( uint8_t i = 4+bias; i < 16; ++i ){
		raw_data[i] = ( msg_rd->buttons&(1<<(i-4-bias)) ) ? 100 : 0;
	}
	if( msg_rd->x>1200 || msg_rd->y>1200 || msg_rd->z>1200 || msg_rd->r>1200 )
		ReceiverUpdate( "JoyStickMv", false, raw_data, 16, 0.01 );
	else
		ReceiverUpdate( "JoyStickMv", true, raw_data, 16, 0.01 );
}

static void Msg75_COMMAND_INT( uint8_t Port_index , const mavlink_message_t* msg )
{
	const mavlink_command_int_t* msg_rd = (mavlink_command_int_t*)msg->payload64;
	if( msg_rd->target_system==0 || 
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==get_CommulinkCompId()) ||
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==0) ||
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==MAV_COMP_ID_CAMERA))
	{
		if( msg_rd->command < Mavlink_CMD_Process_Count )
		{ 		
			if( Mavlink_CMD_Process[ msg_rd->command ] != 0 )
				Mavlink_CMD_Process[ msg_rd->command ]( Port_index , msg );
			else
			{
				ModeMsg mode_msg;
				mode_msg.cmd_type = CMD_TYPE_MAVLINK | Port_index;
				mode_msg.sd_sysid = msg->sysid;
				mode_msg.sd_compid = msg->compid;
				mode_msg.frame = default_NavCmd_frame;
				mode_msg.cmd = msg_rd->command;
				mode_msg.params[0] = msg_rd->param1;
				mode_msg.params[1] = msg_rd->param2;
				mode_msg.params[2] = msg_rd->param3;
				mode_msg.params[3] = msg_rd->param4;
				mode_msg.params[4] = msg_rd->x*1e-7;
				mode_msg.params[5] = msg_rd->y*1e-7;
				mode_msg.params[6] = msg_rd->z;
				SendMsgToMode( mode_msg, 0.01 );
			}
		}
	}
}
static void Msg76_COMMAND_LONG( uint8_t Port_index , const mavlink_message_t* msg )
{
	const mavlink_command_long_t* msg_rd = (mavlink_command_long_t*)msg->payload64;
	if( msg_rd->target_system==0 || 
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==get_CommulinkCompId()) ||
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==0) ||
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==MAV_COMP_ID_CAMERA))
	{
		if( msg_rd->command < Mavlink_CMD_Process_Count )
		{ 		
			if( Mavlink_CMD_Process[ msg_rd->command ] != 0 )
				Mavlink_CMD_Process[ msg_rd->command ]( Port_index , msg );
			else
			{
				ModeMsg mode_msg;
				mode_msg.cmd_type = CMD_TYPE_MAVLINK | Port_index;
				mode_msg.sd_sysid = msg->sysid;
				mode_msg.sd_compid = msg->compid;
				mode_msg.frame = default_NavCmd_frame;
				mode_msg.cmd = msg_rd->command;
				mode_msg.params[0] = msg_rd->param1;
				mode_msg.params[1] = msg_rd->param2;
				mode_msg.params[2] = msg_rd->param3;
				mode_msg.params[3] = msg_rd->param4;
				mode_msg.params[4] = msg_rd->param5;
				mode_msg.params[5] = msg_rd->param6;
				mode_msg.params[6] = msg_rd->param7;
				SendMsgToMode( mode_msg, 0.01 );
			}
		}	
		else if( msg_rd->command == MAV_CMD_IMAGE_START_CAPTURE)
		{
				Cmd2000_MAV_CMD_IMAGE_START_CAPTURE(Port_index , msg );
		}
		else if( msg_rd->command == MAV_CMD_VIDEO_START_CAPTURE)
		{
				Cmd2500_MAV_CMD_VIDEO_START_CAPTURE(Port_index , msg );
		}
		else if( msg_rd->command == MAV_CMD_VIDEO_STOP_CAPTURE)
		{
				Cmd2501_MAV_CMD_VIDEO_STOP_CAPTURE(Port_index , msg );
		}
		else if( msg_rd->command == MAV_CMD_SET_CAMERA_FOCUS)
		{
				Cmd532_MAV_CMD_SET_CAMERA_FOCUS(Port_index , msg );
		}
		else if( msg_rd->command == MAV_CMD_SET_CAMERA_MODE)
		{
				Cmd530_MAV_CMD_SET_CAMERA_MODE(Port_index , msg );
		}
		else if( msg_rd->command == MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW)
		{
				Cmd1000_MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW(Port_index , msg );
		}
		else if( msg_rd->command == MAV_CMD_DO_GIMBAL_MANAGER_LASER)
		{
				Cmd1002_MAV_CMD_DO_GIMBAL_MANAGER_LASER(Port_index , msg );
		}
		else if( msg_rd->command == MAV_CMD_DO_GIMBAL_MANAGER_THERMAL)
		{
				Cmd1003_MAV_CMD_DO_GIMBAL_MANAGER_THERMAL(Port_index , msg );
		}
		else if( msg_rd->command == MAV_CMD_CAMERA_TRACK_POINT)
		{
				Cmd2004_MAV_CMD_CAMERA_TRACK_POINT(Port_index , msg );
		}	
		else if( msg_rd->command == MAV_CMD_CAMERA_STOP_TRACKING)
		{
			Cmd2010_MAV_CMD_CAMERA_STOP_TRACKING(Port_index , msg );
		}		
		else if( msg_rd->command == MAV_CMD_DO_GIMBAL_MANAGER_CENTER)
		{
				Cmd1004_MAV_CMD_DO_GIMBAL_MANAGER_CENTER(Port_index , msg );
		}		
		else if( msg_rd->command == MAV_CMD_DO_GIMBAL_MANAGER_DOWN)
		{
				Cmd1006_MAV_CMD_DO_GIMBAL_MANAGER_DOWN(Port_index , msg );
		}	
		else if( msg_rd->command == MAV_CMD_DO_GIMBAL_START_TARGET_DETECTION)
		{
				Cmd1007_MAV_CMD_DO_GIMBAL_START_TARGET_DETECTION(Port_index , msg );
		}	
		else if( msg_rd->command == MAV_CMD_DO_GIMBAL_STOP_TARGET_DETECTION)
		{
				Cmd1008_MAV_CMD_DO_GIMBAL_STOP_TARGET_DETECTION(Port_index , msg );
		}	
		else if( msg_rd->command == MAV_CMD_DO_GIMBAL_ENABLE_OSD)
		{
				Cmd1009_MAV_CMD_DO_GIMBAL_ENABLE_OSD(Port_index , msg );
		}		
		else if( msg_rd->command == MAV_CMD_DO_GIMBAL_DISABLE_OSD)
		{
				Cmd1010_MAV_CMD_DO_GIMBAL_DISABLE_OSD(Port_index , msg );
		}
		else
		{
			ModeMsg mode_msg;
			mode_msg.cmd_type = CMD_TYPE_MAVLINK | Port_index;
			mode_msg.sd_sysid = msg->sysid;
			mode_msg.sd_compid = msg->compid;
			mode_msg.frame = default_NavCmd_frame;
			mode_msg.cmd = msg_rd->command;
			mode_msg.params[0] = msg_rd->param1;
			mode_msg.params[1] = msg_rd->param2;
			mode_msg.params[2] = msg_rd->param3;
			mode_msg.params[3] = msg_rd->param4;
			mode_msg.params[4] = msg_rd->param5;
			mode_msg.params[5] = msg_rd->param6;
			mode_msg.params[6] = msg_rd->param7;
			SendMsgToMode( mode_msg, 0.01 );
		}
	}
}


static void Msg84_SET_POSITION_TARGET_LOCAL_NED( uint8_t Port_index , const mavlink_message_t* msg )
{
	const __mavlink_set_position_target_local_ned_t* msg_rd = (__mavlink_set_position_target_local_ned_t*)msg->payload64;
	ModeMsg mode_msg;
	mode_msg.cmd_type = CMD_TYPE_MAVLINK | Port_index;
	mode_msg.sd_sysid = msg->sysid;
	mode_msg.sd_compid = msg->compid;
	mode_msg.frame = msg_rd->coordinate_frame;
	mode_msg.cmd = 1084;
	mode_msg.params[0] = msg_rd->type_mask;
	mode_msg.params[1] = msg_rd->x;
	mode_msg.params[2] = msg_rd->y;
	mode_msg.params[3] = msg_rd->z;
	mode_msg.params[4] = msg_rd->vx;
	mode_msg.params[5] = msg_rd->vy;
	mode_msg.params[6] = msg_rd->vz;
	mode_msg.params[7] = msg_rd->afx;
	mode_msg.params[8] = msg_rd->afy;
	mode_msg.params[9] = msg_rd->afz;
	mode_msg.params[10] = msg_rd->yaw;
	mode_msg.params[11] = msg_rd->yaw_rate;
	SendMsgToMode( mode_msg, 0.01 );
}
static void Msg86_SET_POSITION_TARGET_GLOBAL_INT( uint8_t Port_index , const mavlink_message_t* msg )
{
	const mavlink_set_position_target_global_int_t* msg_rd = (mavlink_set_position_target_global_int_t*)msg->payload64;
	ModeMsg mode_msg;
	mode_msg.cmd_type = CMD_TYPE_MAVLINK | Port_index;
	mode_msg.sd_sysid = msg->sysid;
	mode_msg.sd_compid = msg->compid;
	mode_msg.frame = msg_rd->coordinate_frame;
	mode_msg.cmd = 1086;
	mode_msg.params[0] = msg_rd->type_mask;
	mode_msg.params[1] = msg_rd->lat_int*1e-7;
	mode_msg.params[2] = msg_rd->lon_int*1e-7;
	mode_msg.params[3] = msg_rd->alt;
	mode_msg.params[4] = msg_rd->vx;
	mode_msg.params[5] = msg_rd->vy;
	mode_msg.params[6] = msg_rd->vz;
	mode_msg.params[7] = msg_rd->afx;
	mode_msg.params[8] = msg_rd->afy;
	mode_msg.params[9] = msg_rd->afz;
	mode_msg.params[10] = msg_rd->yaw;
	mode_msg.params[11] = msg_rd->yaw_rate;
	SendMsgToMode( mode_msg, 0.01 );
}

static void Msg233_GPS_RTCM_DATA( uint8_t Port_index , const mavlink_message_t* msg )
{
	const mavlink_gps_rtcm_data_t* msg_rd = (mavlink_gps_rtcm_data_t*)msg->payload64;
	
	static uint8_t RTCM_buffer[MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN*4];
	static uint8_t bf_sequence = 0;
	static uint8_t bf_fragments_received = 0;
	
	if( (msg_rd->flags & 1) == 0 )
	{	// it is not fragmented, pass direct
		inject_RtkPorts( msg_rd->data, msg_rd->len );
		bf_fragments_received = 0xf;
		return;
	}
	
	uint8_t fragment = (msg_rd->flags >> 1U) & 0x03;
	uint8_t sequence = (msg_rd->flags >> 3U) & 0x1F;
	
	if ( bf_sequence!=sequence || bf_fragments_received&(1U<<fragment) ) 
	{	// we have one or more partial fragments already received
		// which conflict with the new fragment, discard previous fragments
		bf_fragments_received = 0;
	}
	
	// add this fragment
	bf_sequence = sequence;
	bf_fragments_received |= (1U << fragment);
	
	// copy the data
	memcpy(&RTCM_buffer[MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN*(uint16_t)fragment], msg_rd->data, msg_rd->len);
	
	// when we get a fragment of less than max size then we know the
	// number of fragments. Note that this means if you want to send a
	// block of RTCM data of an exact multiple of the buffer size you
	// need to send a final packet of zero length
	uint8_t fragment_count = 0;
	uint16_t total_length = 0;
	if (msg_rd->len < MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN) {
			fragment_count = fragment+1;
			total_length = (MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN*fragment) + msg_rd->len;
	} else if (bf_fragments_received == 0x0F) {
			// special case of 4 full fragments
			fragment_count = 4;
			total_length = MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN*4;
	}
	
	// see if we have all fragments
	if (fragment_count != 0 &&
			bf_fragments_received == (1U << fragment_count) - 1) {
			// we have them all, inject
			inject_RtkPorts(RTCM_buffer, total_length);
	}
}


	
static void Msg243_SET_HOME_POSITION( uint8_t Port_index , const mavlink_message_t* msg )
{
//	const mavlink_set_home_position_t* msg_rd = (mavlink_set_home_position_t*)msg->payload64;
//	vector2<double> HomeLatLon;
//	HomeLatLon.y = msg_rd->longitude*1e-7;
//	HomeLatLon.x = msg_rd->latitude*1e-7;
////	setHomeLatLon(&HomeLatLon);
}


const uint8_t default_vslam_pos_index = 11;
const uint8_t default_vslam_vel_index = 12;
const uint8_t default_lidar_slam_pos_index = 13;

uint32_t vslam_pos_register_key = 0;
bool vslam_vel_register_key = 0;
bool lidar_slam_pos_register_key = 0;

static void Msg102_VISION_POSITION_ESTIMATE( uint8_t Port_index , const mavlink_message_t* msg )
{
	if(!vslam_pos_register_key)
	{
		if(get_Attitude_MSStatus() != MS_Ready)
			return;
		
		Quaternion quat;
		get_Attitude_quat(&quat);
		double initYaw = quat.getYaw();
		
		vslam_pos_register_key = PositionSlamSensorRegister(default_vslam_pos_index,
																												"VSlam" ,
																												Position_Sensor_Type_RelativePositioning,
																												Position_Sensor_DataType_s_xyz,
																												Position_Sensor_frame_SLAM,
																												0.1 , 
																												initYaw, 
																												20,
																												20
																												);
	}
	
	const mavlink_vision_position_estimate_t* msg_rd = (mavlink_vision_position_estimate_t*)msg->payload64;
	
	vector3<double> pos;
	pos.x = msg_rd->x*100;
	pos.y = -msg_rd->y*100;
	pos.z = -msg_rd->z*100;

	PositionSensorUpdatePosition( default_vslam_pos_index,vslam_pos_register_key, pos, true );
}

static void Msg138_ATT_POS_MOCAP( uint8_t Port_index , const mavlink_message_t* msg )
{
	static double initYaw, sinYaw, cosYaw;
	
	if(!lidar_slam_pos_register_key)
	{
		if(get_Attitude_MSStatus() != MS_Ready)
			return;
		
		Quaternion quat;
		get_Attitude_quat(&quat);
		initYaw = quat.getYaw();
		fast_sin_cos(initYaw, &sinYaw, &cosYaw);
		
		lidar_slam_pos_register_key = PositionSensorRegister( default_lidar_slam_pos_index,
																													 "VSlam" ,
																													 Position_Sensor_Type_RelativePositioning,
																													 Position_Sensor_DataType_s_xy,
																													 Position_Sensor_frame_ENU,
																													 0.1,
																													 25
																													 );
	}
	
	const mavlink_att_pos_mocap_t* msg_rd = (mavlink_att_pos_mocap_t*)msg->payload64;
	
	//º½Ïò¶Ô×¼
	vector3<double> posLidarSlam;
	posLidarSlam.x = BodyHeading2ENU_x(msg_rd->x * 100 , msg_rd->y * 100 , sinYaw , cosYaw);
	posLidarSlam.y = BodyHeading2ENU_y(msg_rd->x * 100 , msg_rd->y * 100 , sinYaw , cosYaw);
	PositionSensorUpdatePosition(default_lidar_slam_pos_index,lidar_slam_pos_register_key, posLidarSlam, true);
}
static void Msg103_VISION_SPEED_ESTIMATE( uint8_t Port_index , const mavlink_message_t* msg )
{
	static double initYaw, sinYaw, cosYaw;
	
	if(!vslam_vel_register_key)
	{
		if(get_Attitude_MSStatus() != MS_Ready)
			return;
		
		Quaternion quat;
		get_Attitude_quat(&quat);
		initYaw = quat.getYaw();
		fast_sin_cos(initYaw, &sinYaw, &cosYaw);
		
		vslam_vel_register_key = PositionSensorRegister( default_vslam_vel_index,
																										  "VSlam" ,
																											Position_Sensor_Type_RelativePositioning,
																											Position_Sensor_DataType_v_xy,
																											Position_Sensor_frame_ENU,
																											0.1,
																											50
																											);
	}
	const mavlink_vision_speed_estimate_t* msg_rd = (mavlink_vision_speed_estimate_t*)msg->payload64;
	
	static vector3<double> velVSlam;
	velVSlam.x = BodyHeading2ENU_x(msg_rd->x * 100 , msg_rd->y * 100 , sinYaw , cosYaw);
	velVSlam.y = BodyHeading2ENU_y(msg_rd->x * 100 , msg_rd->y * 100 , sinYaw , cosYaw);
  velVSlam.z = msg_rd->z * 100;
	PositionSensorUpdateVel(default_vslam_vel_index,vslam_vel_register_key, velVSlam, true);
}

static void Msg111_TIMESYNC( uint8_t Port_index , const mavlink_message_t* msg )
{
	const mavlink_timesync_t* msg_rd = (mavlink_timesync_t*)msg->payload64;
	if( msg_rd->tc1 == 0 ) {
		mavlink_message_t msg_sd;
		const Port* port = get_CommuPort(Port_index);
		if( port->write != 0 ) {
			if( mavlink_lock_chan( Port_index, 0.01 ) )
			{
				mavlink_msg_timesync_pack_chan( 
					get_CommulinkSysId() ,	//system id
					get_CommulinkCompId() ,	//component id
					Port_index ,	//chan
					&msg_sd,
					TIME::get_System_Run_Time()*1e9,	//tc1
					msg_rd->ts1
				);
				mavlink_msg_to_send_buffer(port->write, 
																	 port->lock,
																	 port->unlock,
																	 &msg_sd, 0, 0.01);
				mavlink_unlock_chan(Port_index);
			}
		}
	}
}


static uint32_t registered_sensorAvailable[Position_Sensors_Count] = {0};
static int8_t posYaw_sensor = -1;
static uint32_t posYaw_key = 0;
struct posSensorStruct
{
	uint32_t key;
	bool available;
	bool yawExist;
	Position_Sensor_frame frame;
};
static posSensorStruct registered_sensors[Position_Sensors_Count] = {0};
static void Msg208_ACFly_RegeisterPosSensor( uint8_t Port_index , const mavlink_message_t* msg )
{
	const mavlink_acfly_regeisterpossensor_t* msg_rd = (mavlink_acfly_regeisterpossensor_t*)msg->payload64;
	if( msg_rd->target_system==0 || 
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==get_CommulinkCompId()) ||
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==0) )
	{
		if( msg_rd->ind<0 || msg_rd->ind>=Position_Sensors_Count )
			return;
		
		bool unReg = msg_rd->DataFrame & (1<<6);
		if(!unReg && registered_sensors[msg_rd->ind].key==0)
		{	//可以注册传感器
			Position_Sensor_DataType dataType;
			Position_Sensor_frame frame;
			uint8_t frameInt = msg_rd->DataFrame & 0x3f;
			switch(frameInt)
			{
				case 0:
					frame = Position_Sensor_frame_ENU;
					break;
				case 4:
					frame = Position_Sensor_frame_SLAM;
					break;
				case 1:
					frame = Position_Sensor_frame_BodyHeading;
					break;
				case 5:
					frame = Position_Sensor_frame_PSlamVFlu;
					break;
				default:
					return;
			}
			
			uint32_t key;
			switch(frame)
			{
				case Position_Sensor_frame_ENU:
				case Position_Sensor_frame_BodyHeading:
				{
					key = PositionSensorRegister( msg_rd->ind,
																				msg_rd->sensor_name ,
																				(Position_Sensor_Type)msg_rd->type,
																				(Position_Sensor_DataType)msg_rd->DataType,
																				frame,
																				msg_rd->delay,
																				msg_rd->trustXY*100,
																				msg_rd->trustZ*100
																				);
					registered_sensors[msg_rd->ind].yawExist = false;
					break;
				}
				case Position_Sensor_frame_SLAM:
				case Position_Sensor_frame_PSlamVFlu:
				{
					//记录初始偏航
					Quaternion quat;
					get_Attitude_quat(&quat);
					double iniYaw = quat.getYaw();
					key = PositionSlamSensorRegister( msg_rd->ind,
																						msg_rd->sensor_name ,
																						(Position_Sensor_Type)msg_rd->type,
																						(Position_Sensor_DataType)msg_rd->DataType,
																						frame ,
																						msg_rd->delay , 
																						iniYaw, 
																						msg_rd->trustXY*100,
																						msg_rd->trustZ*100
																						);
					registered_sensors[msg_rd->ind].yawExist = true;
					break;
				}
				default:
					return;
			}
			
			if( key > 0 )
			{	//注册成功
				registered_sensors[msg_rd->ind].key = key;
				registered_sensors[msg_rd->ind].available = false;
				registered_sensors[msg_rd->ind].frame = frame;
			}
		}
		else if(unReg && registered_sensors[msg_rd->ind].key!=0)
		{	//取消注册传感器
			bool res = PositionSensorUnRegister( msg_rd->ind, registered_sensors[msg_rd->ind].key );
			if(res)
			{
				registered_sensors[msg_rd->ind].key = 0;
				if( posYaw_sensor==msg_rd->ind )
				{
					posYaw_sensor = -1;
					IMUMagnetometerUnRegister(Slam_Magnetometer_Index, posYaw_key);
				}
			}
		}
	}
}
static void Msg209_ACFly_UpdatePosSensor( uint8_t Port_index , const mavlink_message_t* msg )
{
	const mavlink_acfly_updatepossensor_t* msg_rd = (mavlink_acfly_updatepossensor_t*)msg->payload64;
	if( msg_rd->target_system==0 || 
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==get_CommulinkCompId()) ||
			(msg_rd->target_system==get_CommulinkSysId() && msg_rd->target_component==0) )
	{
		if( msg_rd->ind<0 || msg_rd->ind>=Position_Sensors_Count )
			return;
		
		if( registered_sensors[msg_rd->ind].key>0 )
		{	//传感器已注册
			uint32_t key = registered_sensors[msg_rd->ind].key;
			
			//传感器类型
			uint8_t dataType = msg_rd->DataType;
			PositionSensorChangeDataType( msg_rd->ind,key, (Position_Sensor_DataType)dataType );
			
			bool available = !(msg_rd->reset & (1<<7));
			if( available==true && registered_sensors[msg_rd->ind].available==false )
			{	//slam传感器可用时重置slam航向
				Quaternion quat(
					msg_rd->AttQuat[0],
					msg_rd->AttQuat[1],
					msg_rd->AttQuat[2],
					msg_rd->AttQuat[3]
				);
				double yaw = quat.getYaw();
				
				extern float debug_test[30];
				debug_test[12] = rad2degree(yaw);
				
				Quaternion fcQuat;
				get_Attitude_quat(&fcQuat);
				double fcYaw = fcQuat.getYaw();
				
				double iniYaw = fcYaw - yaw;
				while( iniYaw > Pi )
					iniYaw -= 2*Pi;
				while( iniYaw < -Pi )
					iniYaw += 2*Pi;
				PositionSlamSensorResetAngleOffset( msg_rd->ind,key, iniYaw );
			}
			registered_sensors[msg_rd->ind].available = available;
			
			if( available )
			{
				if( posYaw_sensor == -1 )
				{
					posYaw_key = IMUMagnetometerSlamRegister( Slam_Magnetometer_Index, SName("SLAM"), 0.0001 );
					if( posYaw_key )
						posYaw_sensor = msg_rd->ind;
				}
				if( posYaw_sensor==msg_rd->ind )
				{
					Quaternion quat(
						msg_rd->AttQuat[0],
						msg_rd->AttQuat[1],
						msg_rd->AttQuat[2],
						msg_rd->AttQuat[3]
					);
					vector3<double> vMag = quat.reverse_rotate(vector3<double>(0,6000,0));
					IMUMagnetometerUpdate( Slam_Magnetometer_Index,posYaw_key, vector3<int32_t>(vMag.x,vMag.y,vMag.z), false );
				}
			}
			else
			{
				if( posYaw_sensor==msg_rd->ind )
				{
					posYaw_sensor = -1;
					IMUMagnetometerUnRegister(Slam_Magnetometer_Index,posYaw_key);
				}
			}
			
			switch(msg_rd->DataType)
			{
				case Position_Sensor_DataType_s_xy:
				case Position_Sensor_DataType_s_xyz:
				case Position_Sensor_DataType_s_z:
				case Position_Sensor_DataType_s_xy_nAC:
				case Position_Sensor_DataType_s_xyz_nAC:
				case Position_Sensor_DataType_s_z_nAC:
				{
					vector3<double> pos;
					pos.x = msg_rd->posX*100;
					pos.y = msg_rd->posY*100;
					pos.z = msg_rd->posZ*100;

					PositionSensorUpdatePosition(msg_rd->ind,key, pos, 
							available, msg_rd->delay, msg_rd->trustXY, msg_rd->trustZ );
					break;
				}
				
				case Position_Sensor_DataType_sv_xy:
				case Position_Sensor_DataType_sv_xyz:
				case Position_Sensor_DataType_sv_z:
				case Position_Sensor_DataType_sv_xy_nAC:
				case Position_Sensor_DataType_sv_xyz_nAC:
				case Position_Sensor_DataType_sv_z_nAC:
				{
					vector3<double> pos;
					pos.x = msg_rd->posX*100;
					pos.y = msg_rd->posY*100;
					pos.z = msg_rd->posZ*100;
					
					vector3<double> vel;
					vel.x = msg_rd->velX*100;
					vel.y = msg_rd->velY*100;
					vel.z = msg_rd->velZ*100;

					PositionSensorUpdatePositionVel(msg_rd->ind,key, pos, vel,
							available, msg_rd->delay, msg_rd->trustXY, msg_rd->trustZ );
					break;
				}
				
				case Position_Sensor_DataType_v_xy:
				case Position_Sensor_DataType_v_xyz:
				case Position_Sensor_DataType_v_z:
				case Position_Sensor_DataType_v_xy_nAC:
				case Position_Sensor_DataType_v_xyz_nAC:
				case Position_Sensor_DataType_v_z_nAC:
				{
					vector3<double> vel;
					vel.x = msg_rd->velX*100;
					vel.y = msg_rd->velY*100;
					vel.z = msg_rd->velZ*100;

					PositionSensorUpdateVel(msg_rd->ind,key, vel,
							available, msg_rd->delay, msg_rd->trustXY, msg_rd->trustZ );
					break;
				}
			}
		}
	}
}

static void Msg144_FOLLOW_TARGET( uint8_t Port_index , const mavlink_message_t* msg )
{
	const mavlink_follow_target_t* msg_rd = (mavlink_follow_target_t*)msg->payload64;
	
	static uint32_t followKey = 0;
	if( followKey == 0 ){
		if((msg_rd->est_capabilities&1)==1)
			followKey = followSensorRegister(followDataType_track_Gs_xy);
		else
			followKey = followSensorRegister(followDataType_nRTL_Gsv_xyz);
	}
	if( followKey )
	{
		vector3<double> pos;
		pos.x = msg_rd->lat*1e-7;
		pos.y = msg_rd->lon*1e-7;
		pos.z = msg_rd->alt*100;
		vector3<double> vel;
		vel.x = msg_rd->vel[1]*1e+2;
		vel.y = msg_rd->vel[0]*1e+2;
		vel.z = -msg_rd->vel[2]*1e+2;
		double yaw = 100;
		if( msg->len>=97 && msg_rd->yaw<500 )
			yaw = degree2rad(90.0 - msg_rd->yaw);
		update_followSensor( followKey, pos, vel, yaw, true );
	}
	
	if( msg->len>=97 && msg_rd->yaw<500 )
		update_precLandSensorYaw( degree2rad(90.0 - msg_rd->yaw), true );
}

void (*const Mavlink_RC_Process[])( uint8_t Port_index , const mavlink_message_t* msg_sd ) = 
{
	/*000-*/	Msg0_HEARTBEAT	,
	/*001-*/	0	,
	/*002-*/	0	,
	/*003-*/	0	,
	/*004-*/	0	,
	/*005-*/	0	,
	/*006-*/	0	,
	/*007-*/	0	,
	/*008-*/	0	,
	/*009-*/	0	,
	/*010-*/	0	,
	/*011-*/	Msg11_SET_MODE	,
	/*012-*/	0	,
	/*013-*/	0	,
	/*014-*/	0	,
	/*015-*/	0	,
	/*016-*/	0	,
	/*017-*/	0	,
	/*018-*/	0	,
	/*019-*/	0	,
	/*020-*/	Msg20_PARAM_REQUEST_READ	,
	/*021-*/	Msg21_PARAM_REQUEST_LIST	,
	/*022-*/	0	,
	/*023-*/	Msg23_PARAM_SET	,
	/*024-*/	0	,
	/*025-*/	0	,
	/*026-*/	0	,
	/*027-*/	0	,
	/*028-*/	0	,
	/*029-*/	0	,
	/*030-*/	0	,
	/*031-*/	0	,
	/*032-*/	0	,
	/*033-*/	0	,
	/*034-*/	0	,
	/*035-*/	0	,
	/*036-*/	0	,
	/*037-*/	0	,
	/*038-*/	0	,
	/*039-*/	Msg39_MISSION_ITEM	,
	/*040-*/	Msg40_MISSION_REQUEST	,
	/*041-*/	Msg41_MISSION_SET_CURRENT	,
	/*042-*/	0	,
	/*043-*/	Msg43_MISSION_REQUEST_LIST	,
	/*044-*/	Msg44_MISSION_COUNT	,
	/*045-*/	Msg45_MISSION_CLEAR_ALL	,
	/*046-*/	0	,
	/*047-*/	0	,
	/*048-*/	0	,
	/*049-*/	0	,
	/*050-*/	0	,
	/*051-*/	Msg51_MISSION_REQUEST_INT	,
	/*052-*/	0	,
	/*053-*/	0	,
	/*054-*/	0	,
	/*055-*/	0	,
	/*056-*/	0	,
	/*057-*/	0	,
	/*058-*/	0	,
	/*059-*/	0	,
	/*060-*/	0	,
	/*061-*/	0	,
	/*062-*/	0	,
	/*063-*/	0	,
	/*064-*/	0	,
	/*065-*/	0	,
	/*066-*/	Msg66_REQUEST_DATA_STREAM	,
	/*067-*/	0	,
	/*068-*/	0	,
	/*069-*/	Msg69_MANUAL_CONTROL	,
	/*070-*/	0	,
	/*071-*/	0	,
	/*072-*/	0	,
	/*073-*/	Msg73_MISSION_ITEM_INT	,
	/*074-*/	0	,
	/*075-*/	Msg75_COMMAND_INT	,
	/*076-*/	Msg76_COMMAND_LONG	,
	/*077-*/	0	,
	/*078-*/	0	,
	/*079-*/	0	,
	/*080-*/	0	,
	/*081-*/	0	,
	/*082-*/	0	,
	/*083-*/	0	,
	/*084-*/	Msg84_SET_POSITION_TARGET_LOCAL_NED	,
	/*085-*/	0	,
	/*086-*/	Msg86_SET_POSITION_TARGET_GLOBAL_INT	,
	/*087-*/	0	,
	/*088-*/	0	,
	/*089-*/	0	,
	/*090-*/	0	,
	/*091-*/	0	,
	/*092-*/	0	,
	/*093-*/	0	,
	/*094-*/	0	,
	/*095-*/	0	,
	/*096-*/	0	,
	/*097-*/	0	,
	/*098-*/	0	,
	/*099-*/	0	,
	/*100-*/	0	,
	/*101-*/	0	,
	/*102-*/	Msg102_VISION_POSITION_ESTIMATE	,
	/*103-*/	0	,
	/*104-*/	0	,
	/*105-*/	0	,
	/*106-*/	0	,
	/*107-*/	0	,
	/*108-*/	0	,
	/*109-*/	0	,
	/*110-*/	Msg110_FILE_TRANSFER_PROTOCOL	,
	/*111-*/	Msg111_TIMESYNC	,
	/*112-*/	0	,
	/*113-*/	0	,
	/*114-*/	0	,
	/*115-*/	0	,
	/*116-*/	0	,
	/*117-*/	0	,
	/*118-*/	0	,
	/*119-*/	0	,
	/*120-*/	0	,
	/*121-*/	0	,
	/*122-*/	0	,
	/*123-*/	0	,
	/*124-*/	0	,
	/*125-*/	0	,
	/*126-*/	0	,
	/*127-*/	0	,
	/*128-*/	0	,
	/*129-*/	0	,
	/*130-*/	0	,
	/*131-*/	0	,
	/*132-*/	0	,
	/*133-*/	0	,
	/*134-*/	0	,
	/*135-*/	0	,
	/*136-*/	0	,
	/*137-*/	0	,
	/*138-*/	0	,
	/*139-*/	0	,
	/*140-*/	0	,
	/*141-*/	0	,
	/*142-*/	0	,
	/*143-*/	0	,
	/*144-*/	Msg144_FOLLOW_TARGET	,
	/*145-*/	0	,
	/*146-*/	0	,
	/*147-*/	0	,
	/*148-*/	0	,
	/*149-*/	0	,
	/*150-*/	0	,
	/*151-*/	0	,
	/*152-*/	0	,
	/*153-*/	0	,
	/*154-*/	0	,
	/*155-*/	0	,
	/*156-*/	0	,
	/*157-*/	0	,
	/*158-*/	0	,
	/*159-*/	0	,
	/*160-*/	0	,
	/*161-*/	0	,
	/*162-*/	0	,
	/*163-*/	0	,
	/*164-*/	0	,
	/*165-*/	0	,
	/*166-*/	0	,
	/*167-*/	0	,
	/*168-*/	0	,
	/*169-*/	0	,
	/*170-*/	0	,
	/*171-*/	0	,
	/*172-*/	0	,
	/*173-*/	0	,
	/*174-*/	0	,
	/*175-*/	0	,
	/*176-*/	0	,
	/*177-*/	0	,
	/*178-*/	0	,
	/*179-*/	0	,
	/*180-*/	0	,
	/*181-*/	0	,
	/*182-*/	0	,
	/*183-*/	Msg183_AUTOPILOT_VERSION_REQUEST	,
	/*184-*/	0	,
	/*185-*/	0	,
	/*186-*/	0	,
	/*187-*/	0	,
	/*188-*/	0	,
	/*189-*/	0	,
	/*190-*/	0	,
	/*191-*/	0	,
	/*192-*/	0	,
	/*193-*/	0	,
	/*194-*/	0	,
	/*195-*/	0	,
	/*196-*/	0	,
	/*197-*/	0	,
	/*198-*/	0	,
	/*199-*/	0	,
	/*200-*/	0	,
	/*201-*/	0	,
	/*202-*/	0	,
	/*203-*/	0	,
	/*204-*/	0	,
	/*205-*/	0	,
	/*206-*/	0	,
	/*207-*/	0	,
	/*208-*/	Msg208_ACFly_RegeisterPosSensor	,
	/*209-*/	Msg209_ACFly_UpdatePosSensor	,
	/*210-*/	0	,
	/*211-*/	0	,
	/*212-*/	0	,
	/*213-*/	0	,
	/*214-*/	0	,
	/*215-*/	0	,
	/*216-*/	0	,
	/*217-*/	0	,
	/*218-*/	0	,
	/*219-*/	0	,
	/*220-*/	0	,
	/*221-*/	0	,
	/*222-*/	0	,
	/*223-*/	0	,
	/*224-*/	0	,
	/*225-*/	0	,
	/*226-*/	0	,
	/*227-*/	0	,
	/*228-*/	0	,
	/*229-*/	0	,
	/*230-*/	0	,
	/*231-*/	0	,
	/*232-*/	0	,
	/*233-*/	Msg233_GPS_RTCM_DATA	,
	/*234-*/	0	,
	/*235-*/	0	,
	/*236-*/	0	,
	/*237-*/	0	,
	/*238-*/	0	,
	/*239-*/	0	,
	/*240-*/	0	,
	/*241-*/	0	,
	/*242-*/	0	,
	/*243-*/	Msg243_SET_HOME_POSITION ,
	/*244-*/	0	,
	/*245-*/	0	,
	/*246-*/	0	,
	/*247-*/	0	,
	/*248-*/	0	,
	/*249-*/	0	,
	/*250-*/	0	,
	/*251-*/	0	,
	/*252-*/	0	,
	/*253-*/	0	,
	/*254-*/	0	,
	/*255-*/	0	,
	/*256-*/	0	,
	/*257-*/	0	,
	/*258-*/	0	,
	/*259-*/	0	,
	/*260-*/	0	,
	/*261-*/	0	,
	/*262-*/	0	,
	/*263-*/	0	,
	/*264-*/	0	,
	/*265-*/	0	,
	/*266-*/	0	,
	/*267-*/	0	,
	/*268-*/	0	,
	/*269-*/	0	,
	/*270-*/	0	,
	/*271-*/	0	,
	/*272-*/	0	,
	/*273-*/	0	,
	/*274-*/	0	,
	/*275-*/	0	,
	/*276-*/	0	,
	/*277-*/	0	,
	/*278-*/	0	,
	/*279-*/	0	,
	/*280-*/	0	,
	/*281-*/	0	,
	/*282-*/	0	,
	/*283-*/	0	,
	/*284-*/	0	,
	/*285-*/	0	,
	/*286-*/	0	,
	/*287-*/	0	,
	/*288-*/	0	,
	/*289-*/	0	,
};
const uint16_t Mavlink_RC_Process_Count = sizeof( Mavlink_RC_Process ) / sizeof( void* );