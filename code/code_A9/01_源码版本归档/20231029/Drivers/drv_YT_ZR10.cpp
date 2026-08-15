#include "drv_YT_ZR10.hpp"
#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"
#include "AuxFuncs.hpp"
#include "MavlinkCMDProcess.hpp"
#include "drv_PWMOut.hpp"
#include "Parameters.hpp"
#include "drv_CRC.hpp"
										 															 
struct DriverInfo
{
	uint32_t param;
	Port port;
};
static const unsigned char packet_ID[6] = {0x55,0x66};

static inline void cameraCheckSum( uint8_t* data, uint16_t begin, uint16_t end, uint16_t crc_pos )
{
	uint8_t sum=0;
	for(int i=begin; i<end+1; i++)
	{
		sum+=data[i];
	}
	data[crc_pos] = sum;
}

// 发送 MAV_COMP_ID_CAMERA 心跳包
static inline void send_heartbeat()
{
	for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
	{	
		const Port* port = get_CommuPort(i);
		mavlink_message_t msg_sd;
		if( port->write != 0 )
		{	
			if( mavlink_lock_chan(i,0.01) )
			{		
				mavlink_msg_heartbeat_pack_chan( 
				get_CommulinkSysId() ,	//system id
				MAV_COMP_ID_CAMERA ,	  //component id
				i	,	//chan
				&msg_sd,
				MAV_TYPE_CAMERA ,		//type
				MAV_AUTOPILOT_PX4 ,	//autopilot
				0,	//base mode
				0,	//custom mode
				0	  //sys status
			);
			mavlink_msg_to_send_buffer( port->write, 
																	port->lock,
																	port->unlock,
																 &msg_sd, 0, 0.01);		
				mavlink_unlock_chan(i);
			}	
		}
	}
}

// 发送 camera_information
static inline void send_camera_information()
{
	for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
	{	
		const Port* port = get_CommuPort(i);
		mavlink_message_t msg_sd;
		if( port->write != 0 )
		{	
			if( mavlink_lock_chan(i,0.01) )
			{		
				uint8_t vendor_name[32]={"SIYI"};
				uint8_t model_name[32]={"ZR10"};
				uint32_t flags = CAMERA_CAP_FLAGS_CAPTURE_VIDEO|
												 CAMERA_CAP_FLAGS_CAPTURE_IMAGE|
												 CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM|
												 CAMERA_CAP_FLAGS_HAS_GIMBAL_PITCH|
												 CAMERA_CAP_FLAGS_HAS_GIMBAL_YAW|
												 CAMERA_CAP_FLAGS_HAS_GIMBAL_CENTER|
												 CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS|
												 CAMERA_CAP_FLAGS_HAS_ACK;

				mavlink_msg_camera_information_pack_chan(
					get_CommulinkSysId() ,	//system id
					MAV_COMP_ID_CAMERA ,	//component id
					i ,	//chan
					&msg_sd,
					TIME::get_System_Run_Time()*1e6,	//time_boot_ms
					vendor_name ,	//vendor_name
					model_name ,	//model_name
					0 , //firmware_version
					0 , //focal_length [mm]
					0 , //sensor_size_h [mm]
					0 , //sensor_size_v [mm]
					0 , //resolution_h [pix]
					0 , //resolution_v [pix]
					0 , //lens_id
					flags , //flags
					0 , //cam_definition_version
					0   //cam_definition_uri
				);
				mavlink_msg_to_send_buffer( port->write, 
																		port->lock,
																		port->unlock,
																	 &msg_sd, 0.01, 0.01);		
				mavlink_unlock_chan(i);
			}	
		}
	}
}

// 发送 ack
uint32_t target_system=0;
uint32_t target_component=0;
static inline void send_ack(uint32_t command, bool res)
{
	for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
	{	
		const Port* port = get_CommuPort(i);
		mavlink_message_t msg_sd;
		if( port->write != 0 )
		{	
			if( mavlink_lock_chan(i,0.01) )
			{		
				mavlink_msg_command_ack_pack_chan( 
				get_CommulinkSysId() ,	//system id
				target_component ,	//component id
				i ,
				&msg_sd,
				command,	//command
				//res ? MAV_RESULT_ACCEPTED : MAV_RESULT_DENIED ,	//result
				MAV_RESULT_ACCEPTED,
				100 ,	//progress
				0 ,	  //param2
				target_system ,	 //target system
				target_component //target component
				);
				mavlink_msg_to_send_buffer( port->write, 
																		port->lock,
																		port->unlock,
																		&msg_sd, 1, 1);		
				mavlink_unlock_chan(i);
			}	
		}
	}
}

static inline void send_camera_capture_status(mavlink_camera_capture_status_t* status)
{
	for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
	{	
		const Port* port = get_CommuPort(i);
		mavlink_message_t msg_sd;
		if( port->write != 0 )
		{	
			if( mavlink_lock_chan(i,0.01) )
			{		
				mavlink_msg_camera_capture_status_pack_chan( 
				get_CommulinkSysId() ,	//system id
				target_component ,	//component id
				i ,
				&msg_sd,
				TIME::get_System_Run_Time()*1e6,	//Timestamp
				status->image_status,
				status->video_status,
				status->image_interval ,	
				status->recording_time_ms ,	  //param2
				status->available_capacity ,	 //target system
				status->image_count //target component
				);
				mavlink_msg_to_send_buffer( port->write, 
																		port->lock,
																		port->unlock,
																		&msg_sd, 1, 1);		
				mavlink_unlock_chan(i);
			}	
		}
	}
}
static inline void cmdPackSend(DriverInfo* driver_info,int8_t* data, uint16_t dataLen, uint8_t cmd, bool need_ack)
{
	static uint16_t seq=0;
	Aligned_DMABuf int8_t sdata[64];
	*(uint16_t*)&sdata[0] = 0x6655;
	*(uint8_t*) &sdata[2] = need_ack ? 1: 0;
	*(uint16_t*)&sdata[3] = dataLen;
	*(uint16_t*)&sdata[5] = seq++;
	*(uint8_t*) &sdata[7] = cmd;
	memcpy(&sdata[8],data,dataLen);
	uint32_t res =0;
	CRC_Calc( cg_CRC_CONFIG_CRC16_XMODEM, (uint8_t*)&sdata[0], dataLen+8, &res);
	*(uint16_t*)&sdata[dataLen+8] = res;	
  driver_info->port.write( (uint8_t*)&sdata, dataLen+8+2, 0.1, 0.1 );	
}


static void YT_ZR10_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;

	#define MAX_DATA_LENGHT 50
	uint8_t rc_counter = 0;
	uint8_t sum = 0;
	uint8_t cmd = 0;
	uint16_t dataLenght = 0;
	uint8_t data[MAX_DATA_LENGHT];
	
  CmdMsg msg;
	TIME heartBeatSendTime;
	TIME takePhotoTime;
	TIME videoStartTime;
	TIME secondCmdSendTime;
	TIME idleTime;
	uint8_t secondCmd=0;
	bool secondCmdSendFlag=false;
	bool takePhotoFlag=false;
	bool videoStartFlag=false;
	bool msgAvailable = false;
	bool ack_wait_send=false;
	
	AuxFuncsConfig aux_configs;
	ReadParamGroup( "AuxCfg", (uint64_t*)&aux_configs, 0 );	
	
	//遥控器状态
	int8_t pitStick=0;
	int8_t yawStick=0;
	
	enum VideoStatus {
		VIDEO_CAPTURE_STATUS_STOPPED = 0,
		VIDEO_CAPTURE_STATUS_RUNNING,
		VIDEO_CAPTURE_FAIL,
		VIDEO_CAPTURE_STATUS_LAST,
		VIDEO_CAPTURE_STATUS_UNDEFINED = 255
	};
	
	enum PhotoStatus {
		PHOTO_CAPTURE_IDLE = 0,
		PHOTO_CAPTURE_IN_PROGRESS,
		PHOTO_CAPTURE_INTERVAL_IDLE,
		PHOTO_CAPTURE_INTERVAL_IN_PROGRESS,
		PHOTO_CAPTURE_FAIL,
		PHOTO_CAPTURE_LAST,
		PHOTO_CAPTURE_STATUS_UNDEFINED = 255
	};
	
	int imageCnt=0;
	mavlink_camera_capture_status_t cameraStatus;
	while(1)
	{
		//获取接收机
		Receiver rc;
		getReceiver(&rc);
		
		/*数字云台接口*/
		bool rcInControl=false;
		PR_RESULT res = ReadParamGroup( "AuxCfg", (uint64_t*)&aux_configs, 0 );	
		for( uint8_t i = 0; i < 16; ++i )
		{	
			uint16_t aux_cfg = ((uint16_t*)&aux_configs.RcYT1Func)[i*4];
			if(aux_cfg!=0)
				rcInControl=true;
		}			
		
		for( uint8_t i = 0; i < 16; ++i )
		{	
			if(res==PR_OK && i >= 4)
			{
				uint16_t aux_cfg = ((uint16_t*)&aux_configs.RcYT1Func)[i*4];
				float aux_param1 = ((float*)&aux_configs.RcYT1Param1)[i*2];
				float aux_param2 = ((float*)&aux_configs.RcYT1Param2)[i*2];		
				if( rc.connected && rc.raw_available_channels-1 >= i)
				{		
					if(aux_cfg==DgYTPit)
					{// 俯仰
						int8_t data[2];
						pitStick = (rc.raw_data[i]-50)*2;
						data[0] = yawStick;
						data[1] = pitStick;
						cmdPackSend(&driver_info,data,2,0x07,false);
					}
					else if(aux_cfg==DgYTYaw)
					{// 偏航
						int8_t data[2];
						yawStick = (rc.raw_data[i]-50)*2;
						data[0] = yawStick;
						data[1] = pitStick;
						cmdPackSend(&driver_info,data,2,0x07,false);
					}
					else if(aux_cfg==DgYTZoom)
					{// 变倍
						int8_t data=0;
						if(rc.raw_data[i]>70) //放大
							data = 1;
						else if(rc.raw_data[i]>30 && rc.raw_data[i]<70)// 停止缩放
							data = 0;
						else if(rc.raw_data[i]<30)// 缩小
							data = -1;							
						cmdPackSend(&driver_info,&data,1,0x05,false);
					}
					else if(aux_cfg==DgYTFocus)
					{// 手动对焦
						int8_t data=0;
						if(rc.raw_data[i]>70) //远景
							data = 1;
						else if(rc.raw_data[i]>30 && rc.raw_data[i]<70)// 停止对焦
							data = 0;
						else if(rc.raw_data[i]<30)// 近景
							data = -1;				
						cmdPackSend(&driver_info,&data,1,0x06,false);		
					}
					else if(aux_cfg==DgYTAutoFocus)
					{// 自动聚焦
						static float lastChannelValue = rc.raw_data[i];
						if(fabsf(rc.raw_data[i]-lastChannelValue)>30)
						{
							int8_t data=1;					
							cmdPackSend(&driver_info,&data,1,0x04,false);	
							lastChannelValue = rc.raw_data[i];
						}							
					}	
					else if(aux_cfg==DgYTCenter)
					{// 一键回中
						static float lastChannelValue = rc.raw_data[i];
						if(fabsf(rc.raw_data[i]-lastChannelValue)>30)
						{
							int8_t data=1;					
							cmdPackSend(&driver_info,&data,1,0x08,false);	
							lastChannelValue = rc.raw_data[i];
						}							
					}						
					else if(aux_cfg==DgYTVideo)
					{// 录像	
						static float lastChannelValue = rc.raw_data[i];
						if(fabsf(rc.raw_data[i]-lastChannelValue)>50)
						{
							int8_t data=0;
							cmdPackSend(&driver_info,&data,1,0x0C,false);	
							lastChannelValue = rc.raw_data[i];		
						}							
					}		
					else if(aux_cfg==DgYTVideo)
					{// 拍照	
						static float lastChannelValue = rc.raw_data[i];
						if(fabsf(rc.raw_data[i]-lastChannelValue)>50)
						{
							int8_t data=1;
							cmdPackSend(&driver_info,&data,1,0x0C,false);	
							lastChannelValue = rc.raw_data[i];		
						}							
					}						
				}		
			}
		}
		/*数字云台接口*/		
		if(!rcInControl){
			msg.cmd=0;
			msgAvailable=TaskReceiveCmdMsg( &msg, CAMERA_GIMBAL_CONTROL_CMD_QUEUE_ID,0.1);
			if(msgAvailable)
			{
				target_system = msg.target_system;
				target_component = msg.target_component;
				driver_info.port.reset_rx(1);
				if(msg.cmd == MAV_CMD_REQUEST_CAMERA_INFORMATION)
				{// 相机信息发送
					send_camera_information();
					send_ack(msg.cmd, 1);
					ack_wait_send	= false;	
				}	
				else if(msg.cmd == MAV_CMD_REQUEST_MESSAGE)
				{// 相机信息发送
					if(msg.params[0]==MAVLINK_MSG_ID_CAMERA_INFORMATION)
					{
						send_camera_information();
						send_ack(msg.cmd, 1);
						ack_wait_send	= false;	
					}
				}	
				else if(msg.cmd == MAV_CMD_SET_CAMERA_MODE)
				{// 模式切换
	//				if(msg.params[1]==3)
	//					pack.cmd = 0x1B; // 切可见光传感器	
	//				else if(msg.params[1]==4)
	//					pack.cmd = 0x1D; // 切红外传感器		
	//				else if(msg.params[1]==5)
	//					pack.cmd = 0x1F; // 开启画中画
	//				else if(msg.params[1]==6)
	//					pack.cmd = 0x21; // 关闭画中画				
	//				ack_wait_send=true;			
				}
				else if(msg.cmd == MAV_CMD_SET_CAMERA_FOCUS)
				{// 聚焦
					int8_t data=0;
					if(msg.params[0]==FOCUS_TYPE_AUTO){
						data = 1; // 自动聚焦	
						cmdPackSend(&driver_info,&data,1,0x04,false);		
						send_ack(msg.cmd, 1);
						ack_wait_send=false;
					}							
					else 
					{
						if(msg.params[0]==FOCUS_TYPE_STEP && msg.params[1]==1)	
							data = 1;  // 调焦加		
						else if(msg.params[0]==FOCUS_TYPE_STEP && msg.params[1]==-1)
							data = -1; // 调焦减	
						else if(msg.params[0]==FOCUS_TYPE_STEP && msg.params[1]==0)
							data = 0;  // 停止对焦
						cmdPackSend(&driver_info,&data,1,0x06,false);		
					}
					ack_wait_send	= false;
					idleTime=TIME::now();
				}
				else if(msg.cmd == MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW)
				{// 俯仰，向上为正;  航向，向左为正	
					int8_t data[2];
					data[0] = -msg.params[3]*2;
					data[1] = msg.params[2]*2;
					cmdPackSend(&driver_info,data,2,0x07,false);	
					send_ack(msg.cmd, 1);
					ack_wait_send	= false;	
					idleTime=TIME::now(); 		
				}	
				else if(msg.cmd == MAV_CMD_SET_CAMERA_ZOOM)
				{// 变倍(-1 for wide, 1 for tele, 0 to stop zooming)
					int8_t data=0;
					if(msg.params[1]==-1) //放大
						data = 1;
					else if(msg.params[1]==0)// 停止缩放
						data = 0;
					else if(msg.params[1]==1)// 缩小
						data = -1;							
					cmdPackSend(&driver_info,&data,1,0x05,false);	
					ack_wait_send	= false;	
					idleTime=TIME::now(); 			
				}
				else if(msg.cmd == MAV_CMD_IMAGE_START_CAPTURE)
				{// 拍照
					int8_t data=0;
					cmdPackSend(&driver_info,&data,1,0x0C,false);
					send_ack(msg.cmd, 1);
					// 发送拍照状态获取命令
					os_delay(0.5);
					cmdPackSend(&driver_info,&data,0,0x0B,false);	
					ack_wait_send	= true;
				}
				else if(msg.cmd == MAV_CMD_VIDEO_START_CAPTURE)
				{// 开始录像
					int8_t data=2;
					cmdPackSend(&driver_info,&data,1,0x0C,false);	
					send_ack(msg.cmd, 1);
					// 发送录像状态获取命令
					os_delay(0.1);
					cmdPackSend(&driver_info,&data,0,0x0A,false);	
					ack_wait_send	= true;
				}			
				else if(msg.cmd == MAV_CMD_VIDEO_STOP_CAPTURE)
				{// 停止录像
					int8_t data=2;
					cmdPackSend(&driver_info,&data,1,0x0C,false);
					send_ack(msg.cmd, 1);
					// 发送录像状态获取命令
					os_delay(0.1);
					cmdPackSend(&driver_info,&data,0,0x0A,false);	
					ack_wait_send	= true;
				}
				else if(msg.cmd == MAV_CMD_DO_GIMBAL_MANAGER_CENTER)
				{// 一键回中
					int8_t data=1;
					cmdPackSend(&driver_info,&data,1,0x08,false);
					send_ack(msg.cmd, 1);
					ack_wait_send	= false;		
				}
				else if(msg.cmd == MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS)
				{// 请求录像状态
					send_ack(msg.cmd, 1);
				}					
			}
			
			if(secondCmdSendFlag){
				if(secondCmdSendTime.get_pass_time()>1){

					secondCmdSendFlag = false;
				}			
			}
			
			bool waitingCameraStatusPack = false;
		  if(heartBeatSendTime.get_pass_time()>0.99)
			{
				//发送heartBeat
				send_heartbeat();
				if(msg.cmd != MAV_CMD_IMAGE_START_CAPTURE && idleTime.get_pass_time()>2){
					// 发送录像状态获取命令
					int8_t data=0;
					msg.cmd = MAV_CMD_VIDEO_START_CAPTURE;
					driver_info.port.reset_rx(1);
					cmdPackSend(&driver_info,&data,0,0x0A,false);	
					waitingCameraStatusPack = true;
				}
				heartBeatSendTime=TIME::now();
			}	
			
			Aligned_DMABuf uint8_t rdata[32];
			if(ack_wait_send || waitingCameraStatusPack){
				waitingCameraStatusPack = false;
				driver_info.port.wait_sent(0.1);
				os_delay(1);
				uint8_t readLen = driver_info.port.read( rdata, 15, 0.5, 1 );
				if( readLen > 0 )
				{
					rc_counter=0;
					for(int i = 0; i < readLen; i++)
					{
						if( rc_counter == 0 )
							sum = 0;
						if( rc_counter < 2 )
						{//接收包头	
							if( rdata[i] != packet_ID[ rc_counter ] )
								rc_counter = 0;
							else
								++rc_counter;
						}
						else if( rc_counter < 3 )
						{// 此包是否为 ack 包
							if(rdata[i]==2){
							  sum += (unsigned char)rdata[i];
							  ++rc_counter;
							}
							else
								break;
						}
						else if( rc_counter < 5 )
						{	//数据长度
							if(rc_counter==3)
								dataLenght = *(uint16_t*)&rdata[i];
							if(dataLenght<=MAX_DATA_LENGHT){
								sum += (unsigned char)rdata[i];
								++rc_counter;
							}else
								break;
						}
						else if( rc_counter < 7 )
						{	//帧的序列
							sum += (unsigned char)rdata[i];
							++rc_counter;
						}
						else if( rc_counter < 8 )
						{	//命令 ID
							cmd = rdata[i];
							sum += (unsigned char)rdata[i];
							++rc_counter;
						}
						else if( rc_counter < 8+dataLenght )
						{	//数据
							data[rc_counter-8] = rdata[i];
							sum += (unsigned char)rdata[i];
							++rc_counter;
						}
						else if(rc_counter==8+dataLenght)
						{ //校验
							uint32_t res =0;
							CRC_Calc( cg_CRC_CONFIG_CRC16_XMODEM, (uint8_t*)&rdata[0], 8+dataLenght, &res);
							if(res == *(uint16_t*)&rdata[8+dataLenght])
							{// 校验通过
								if(msg.cmd==MAV_CMD_VIDEO_START_CAPTURE||msg.cmd==MAV_CMD_VIDEO_STOP_CAPTURE)
								{ // 录像反馈
									if(dataLenght>=5)
										cameraStatus.video_status = data[3]==1 ? VIDEO_CAPTURE_STATUS_RUNNING : VIDEO_CAPTURE_STATUS_STOPPED;
									cameraStatus.image_status = PHOTO_CAPTURE_IDLE;
									cameraStatus.image_count=imageCnt;
									cameraStatus.recording_time_ms=0;
								  cameraStatus.image_interval=0;
									cameraStatus.available_capacity=0;
									cameraStatus.time_boot_ms=0;
									send_camera_capture_status(&cameraStatus);
								}
								else if(msg.cmd==MAV_CMD_IMAGE_START_CAPTURE)
								{// 拍照反馈
									if(dataLenght>=1){
										if(data[0]==0){
											++imageCnt;
											cameraStatus.image_status = PHOTO_CAPTURE_IDLE;
										}else if(data[0]==1)
											cameraStatus.image_status = PHOTO_CAPTURE_FAIL;
									}
									cameraStatus.image_count=imageCnt;
									cameraStatus.recording_time_ms=0;
									cameraStatus.image_interval=0;
									cameraStatus.available_capacity=0;
									cameraStatus.time_boot_ms=0;
									send_camera_capture_status(&cameraStatus);
								}else						
									send_ack(msg.cmd, data[0]);
							}
							rc_counter=0;
						}	
					}
				}
				rc_counter=0;
				ack_wait_send=false;
			}
		}else
			os_delay(0.05);
	}
}

static bool YT_ZR10_DriverInit( Port port, uint32_t param )
{
	//波特率19200
	port.SetBaudRate( 115200, 2, 2 );

	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	xTaskCreate( YT_ZR10_Server, "YT_ZR10", 812, (void*)driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_YT_ZR10()
{
	PortFunc_Register( 107, YT_ZR10_DriverInit );
}