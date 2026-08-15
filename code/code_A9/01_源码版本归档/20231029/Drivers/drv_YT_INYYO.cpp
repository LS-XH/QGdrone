#include "drv_YT_INYYO.hpp"
#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"
#include "AuxFuncs.hpp"
#include "Parameters.hpp"
#include "MavlinkCMDProcess.hpp"
struct DriverInfo
{
	uint32_t param;
	Port port;
};

static inline void cameraCheckSum( uint8_t* data, uint16_t begin, uint16_t end, uint16_t crc_pos )
{
	uint32_t sum=0;
	for(int i=begin; i<end+1; i++)
	{
		sum+=data[i];
	}
	data[crc_pos] = uint8_t(sum%0x100);
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
				uint8_t vendor_name[32]={"INYYO"};
				uint8_t model_name[32]={"INYYO"};
				uint32_t flags = CAMERA_CAP_FLAGS_HAS_GIMBAL_PITCH|
												 CAMERA_CAP_FLAGS_HAS_GIMBAL_YAW|
												 CAMERA_CAP_FLAGS_HAS_GIMBAL_CENTER|
												 CAMERA_CAP_FLAGS_HAS_GIMBAL_DOWN|
												 CAMERA_CAP_FLAGS_GIMBAL_CTRL_SEND_WHEN_CHANGED|
												 CAMERA_CAP_FLAGS_CAPTURE_IMAGE|
												 CAMERA_CAP_FLAGS_CAPTURE_VIDEO|
												 CAMERA_CAP_FLAGS_HAS_DOUBLE_VIDEO|
												 CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM|
												 CAMERA_CAP_FLAGS_HAS_MODES|
												 CAMERA_CAP_FLAGS_HAS_TRACKING_POINT|
												 //CAMERA_CAP_FLAGS_TARGET_DETECTION|
												 CAMERA_CAP_FLAGS_SHOW_CAMERA_AREA|
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
static uint32_t target_system=0;
static uint32_t target_component=0;
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

static inline void send_camera_gimbal_status(mavlink_camera_gimbal_status_t* status)
{
	for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
	{	
		const Port* port = get_CommuPort(i);
		mavlink_message_t msg_sd;
		if( port->write != 0 )
		{	
			if( mavlink_lock_chan(i,0.2) )
			{		
				mavlink_msg_camera_gimbal_status_encode_chan(get_CommulinkSysId(),target_component,i,&msg_sd,status);
				mavlink_msg_to_send_buffer( port->write, 
																		port->lock,
																		port->unlock,
																		&msg_sd, 1, 1);		
				mavlink_unlock_chan(i);
			}	
		}
	}	
}

static void YT_INYYO_Read(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	uint8_t rc_counter=0;
	uint8_t rdata=0;
	uint8_t sum=0;
	uint8_t payLoad[50];
	mavlink_camera_gimbal_status_t camera_gimbal_status;
	while(1)
	{
		uint8_t readLen = driver_info.port.read( &rdata, 1, 1, 1 );
		if( readLen > 0 )
		{	
			if( rc_counter == 0 )//帧头
			{
				sum = 0;
				if(rdata==0xEE){
					rc_counter++;
					sum+=rdata;
				}
			}
			else if(rc_counter==1)//消息ID
			{
				if(rdata==0x01){
					rc_counter++;
					sum+=rdata;					
				}else
					rc_counter=0;
			}
			else if(rc_counter==2)//帧长,消息长度24字节
			{
				if(rdata==0x18){
					rc_counter++;
					sum+=rdata;					
				}else
					rc_counter=0;
			}
			else if(rc_counter<23)//数据
			{
				payLoad[rc_counter-3]=rdata;
				rc_counter++;
				sum+=rdata;	
			}
			else if(rc_counter==23)//校验
			{
				if(rdata==sum)
				{	/* 翔拓吊仓垂直向下为-90度，向前为0度，航向向右为正，需改成:
					   俯仰垂直向下为0度，向前为90度，NED坐标系，航向向右为正，0-360*/
					float zoomLevel=0;
					camera_gimbal_status.gimBal_pitch = (*(int16_t*)&payLoad[4])*0.1 + 90;//云台俯仰角度(单位0.1°)
					camera_gimbal_status.gimBal_yaw = (*(int16_t*)&payLoad[6])*0.1;  //云台航向角度(单位0.1°)
					camera_gimbal_status.currentZoomLevel = payLoad[8];//可见光变焦倍数(整数)
					camera_gimbal_status.zoomLevel_max=40;
					camera_gimbal_status.zoomLevel_min=1;
					camera_gimbal_status.hfov_max=61.11;
					camera_gimbal_status.hfov_min=4.12;
					camera_gimbal_status.vfov_max=35.2;
					camera_gimbal_status.vfov_min=4.12;
					camera_gimbal_status.track_mode=payLoad[9];
					camera_gimbal_status.yaw_mode=payLoad[24];
					camera_gimbal_status.flag=1;
					zoomLevel=camera_gimbal_status.currentZoomLevel;
					if(zoomLevel>=1&&zoomLevel<=10)
					{
						camera_gimbal_status.vfov_now = 0.2324*zoomLevel*zoomLevel - 6.0038*zoomLevel + 40.857;
						camera_gimbal_status.hfov_now = 0.3430*zoomLevel*zoomLevel - 9.7678*zoomLevel + 70.676;
					}
					else if(zoomLevel>10 && zoomLevel<=40)
					{
						camera_gimbal_status.vfov_now = 4.12 - 0.05*zoomLevel;
						camera_gimbal_status.hfov_now = 7.65 - 0.05*zoomLevel;		
					}
					send_camera_gimbal_status(&camera_gimbal_status);
				}
				rc_counter=0;
			}			
		}		
	}		
}


static void YT_INYYO_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	AuxFuncsConfig aux_configs;
	ReadParamGroup( "AuxCfg", (uint64_t*)&aux_configs, 0 );	
	
	int16_t pitStick=0;
	int16_t yawStick=0;
	int16_t imageCnt=0;
	mavlink_camera_capture_status_t cameraStatus;
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
	
	CmdMsg msg;
	TIME heartBeatSendTime;
	TIME cmdActiveTime;
	#define freq 50
	double h = 1.0 / freq;
	
	uint8_t sdata[7]={0,0,0,0,0,0,0};
	while(1)
	{
		Receiver rc;
		getReceiver(&rc);
		
		msg.cmd=0;
		bool msgAvailable=TaskReceiveCmdMsg( &msg, CAMERA_GIMBAL_CONTROL_CMD_QUEUE_ID,0.2);
		if(msgAvailable)
		{/*地面站控制*/	
			if(cmdActiveTime.get_pass_time()<0.01)
				continue;	
			cmdActiveTime=TIME::now();
			
			target_system = msg.target_system;
			target_component = msg.target_component;
			if(msg.cmd == MAV_CMD_REQUEST_CAMERA_INFORMATION)
			{// 相机信息发送
				send_camera_information();
				send_ack(msg.cmd, 1);
			}	
			else if(msg.cmd == MAV_CMD_REQUEST_MESSAGE)
			{// 相机信息发送
				if(msg.params[0]==MAVLINK_MSG_ID_CAMERA_INFORMATION)
				{
					send_camera_information();
					send_ack(msg.cmd, 1);	
				}
			}	
			else if(msg.cmd == MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW)
			{// 俯仰，向上为正;  航向，向左为正	
				if(msg.cmdSourceType==111)
				{// 航线规划控制
					
				}
				else
				{// 地面站控制
					//Pitch
					static bool pitch_center_send = false;
					if(fabs(msg.params[2])>0){
						sdata[0]=0xff;
						sdata[1]=0x01;
						sdata[2]=0x00;
						if(msg.params[2]>0)
							sdata[3]=0x08;
						else if(msg.params[2]==0 && msg.params[3]==0)
							sdata[3]=0x00;
						else if(msg.params[2]<0)
							sdata[3]=0x10;					
						sdata[4]=0;
						sdata[5]=msg.params[2]>0 ? msg.params[2]*1.5 : msg.params[2]*-1.5;
						cameraCheckSum(sdata,1,5,6);
						driver_info.port.write(sdata,7,0.2,1);
						pitch_center_send=false;
					}else if(!pitch_center_send){
						sdata[0]=0xff;
						sdata[1]=0x01;
						sdata[2]=0;
						sdata[3]=0;
						sdata[4]=0;
						sdata[5]=0;
						cameraCheckSum(sdata,1,5,6);
						driver_info.port.write(sdata,7,0.2,1);						
						pitch_center_send=true;
					}else{	
						//Yaw
						sdata[0]=0xff;
						sdata[1]=0x01;
						sdata[2]=0x00;
						if(msg.params[3]>0)
							sdata[3]=0x04;
						else if(msg.params[2]==0 && msg.params[3]==0)
							sdata[3]=0x00;
						else if(msg.params[3]<0)
							sdata[3]=0x02;	
						sdata[4]=msg.params[3]>0 ? msg.params[3]*1.5 : msg.params[3]*-1.5;
						sdata[5]=0;
						cameraCheckSum(sdata,1,5,6);
						driver_info.port.write(sdata,7,0.2,1);		
					}					
				}
				send_ack(msg.cmd, 1); 		
			}	
			else if(msg.cmd == MAV_CMD_DO_GIMBAL_MANAGER_CENTER)
			{// 一键回中
				sdata[0]=0xff;
				sdata[1]=0x01;
				sdata[2]=0x13;
				sdata[3]=0x03;
				sdata[4]=0;
				sdata[5]=0;
				cameraCheckSum(sdata,1,5,6);
				driver_info.port.write(sdata,7,0.2,1);		
				send_ack(msg.cmd, 1); 			
			}			
			else if(msg.cmd == MAV_CMD_DO_GIMBAL_MANAGER_DOWN)
			{// 一键向下
				sdata[0]=0xff;
				sdata[1]=0x01;
				sdata[2]=0x13;
				sdata[3]=0x02;
				sdata[4]=0;
				sdata[5]=0;
				sdata[6]=0x16;
				driver_info.port.write(sdata,7,0.1,1);			
				send_ack(msg.cmd, 1); 	
			}	
			else if(msg.cmd == MAV_CMD_IMAGE_START_CAPTURE)
			{// 拍照
				sdata[0]=0xff;
				sdata[1]=0x01;
				sdata[2]=0x12;
				sdata[3]=0x00;
				sdata[4]=0x00;
				sdata[5]=0x00;
				sdata[6]=0x13;		
				driver_info.port.write(sdata,7,0.1,1);		
				send_ack(msg.cmd, 1); 		
				cameraStatus.image_status = PHOTO_CAPTURE_IDLE;
				cameraStatus.image_count=++imageCnt;
				cameraStatus.recording_time_ms=0;
				cameraStatus.image_interval=0;
				cameraStatus.available_capacity=0;
				cameraStatus.time_boot_ms=0;
				send_camera_capture_status(&cameraStatus);							
			}
			else if(msg.cmd == MAV_CMD_VIDEO_START_CAPTURE)
			{// 开始录像
				sdata[0]=0xff;
				sdata[1]=0x01;
				sdata[2]=0x12;
				sdata[3]=0x02;
				sdata[4]=0x00;
				sdata[5]=0x00;
				sdata[6]=0x15;		
				driver_info.port.write(sdata,7,0.1,1);
				send_ack(msg.cmd, 1); 	
				cameraStatus.video_status = VIDEO_CAPTURE_STATUS_RUNNING;
				cameraStatus.image_status = PHOTO_CAPTURE_IDLE;
				cameraStatus.image_count=imageCnt;
				cameraStatus.recording_time_ms=0;
				cameraStatus.image_interval=0;
				cameraStatus.available_capacity=0;
				cameraStatus.time_boot_ms=0;
				send_camera_capture_status(&cameraStatus);				
			}
			else if(msg.cmd == MAV_CMD_VIDEO_STOP_CAPTURE)
			{// 停止录像
				sdata[0]=0xff;
				sdata[1]=0x01;
				sdata[2]=0x12;
				sdata[3]=0x01;
				sdata[4]=0x00;
				sdata[5]=0x00;
				sdata[6]=0x14;		
				driver_info.port.write(sdata,7,0.1,1);	
				send_ack(msg.cmd, 1); 					
				cameraStatus.video_status = VIDEO_CAPTURE_STATUS_STOPPED;
				cameraStatus.image_status = PHOTO_CAPTURE_IDLE;
				cameraStatus.image_count=imageCnt;
				cameraStatus.recording_time_ms=0;
				cameraStatus.image_interval=0;
				cameraStatus.available_capacity=0;
				cameraStatus.time_boot_ms=0;
				send_camera_capture_status(&cameraStatus);				 						
			}					
			else if(msg.cmd == MAV_CMD_SET_CAMERA_MODE)
			{// 模式切换
				sdata[0]=0xff;
				sdata[1]=0x01;
				sdata[2]=0x14;
				sdata[3]=0x00;
				if(msg.params[1]==3)
					sdata[4]=0x02; // 全图可见光
				else if(msg.params[1]==4)
					sdata[4]=0x03; // 全图红外光
				else if(msg.params[1]==5)
					sdata[4]=0x01; // 画中画1，大图可见光，小图红外光；
				else if(msg.params[1]==6)
					sdata[4]=0x00; // 画中画2，大图红外光、小图可见光；			
				sdata[5]=0x00;	
				cameraCheckSum(sdata,1,5,6);				
				driver_info.port.write(sdata,7,0.1,1);
				send_ack(msg.cmd, 1); 					
			}
			else if(msg.cmd == MAV_CMD_SET_CAMERA_ZOOM)
			{// 缩放
				int8_t data=0;
				if(msg.params[1]==-1){ //放大
					sdata[0]=0xff;
					sdata[1]=0x01;
					sdata[2]=0x00;
					sdata[3]=0x40;
					sdata[4]=0x04;
					sdata[5]=0x00;
					sdata[6]=0x45;
				}
				else if(msg.params[1]==0){// 停止缩放
					sdata[0]=0xff;
					sdata[1]=0x01;
					sdata[2]=0x00;
					sdata[3]=0x60;
					sdata[4]=0x00;
					sdata[5]=0x00;
					sdata[6]=0x61;
				}
				else if(msg.params[1]==1){// 缩小
					sdata[0]=0xff;
					sdata[1]=0x01;
					sdata[2]=0x00;
					sdata[3]=0x20;
					sdata[4]=0x04;
					sdata[5]=0x00;
					sdata[6]=0x25;
				}
				driver_info.port.write(sdata,7,0.1,1);							
				send_ack(msg.cmd, 1); 							
			}			
			else if(msg.cmd == MAV_CMD_DO_GIMBAL_START_TARGET_DETECTION)
			{// 开始目标检测		
				sdata[0]=0xff;
				sdata[1]=0x01;
				sdata[2]=0x11;
				sdata[3]=0x02;
				sdata[4]=0x00;
				sdata[5]=0x00;
				sdata[6]=0x14;		
				driver_info.port.write(sdata,7,0.1,1);
				send_ack(msg.cmd, 1); 					
			}
			else if(msg.cmd == MAV_CMD_DO_GIMBAL_STOP_TARGET_DETECTION)
			{// 停止目标检测		
				sdata[0]=0xff;
				sdata[1]=0x01;
				sdata[2]=0x11;
				sdata[3]=0x03;
				sdata[4]=0x00;
				sdata[5]=0x00;
				sdata[6]=0x15;		
				driver_info.port.write(sdata,7,0.1,1);
				send_ack(msg.cmd, 1); 					
			}		
			else if(msg.cmd == MAV_CMD_CAMERA_TRACK_POINT)
			{// 开始目标跟踪
				if(msg.params[0]>0&&msg.params[1]>0){
					sdata[0]=0xff;
					sdata[1]=0x01;
					sdata[2]=0x11;
					sdata[3]=0x00;
					sdata[4]=msg.params[0];
					sdata[5]=msg.params[1];	
					cameraCheckSum(sdata,1,5,6);				
					driver_info.port.write(sdata,7,0.1,1);
					send_ack(msg.cmd, 1); 
				}
			}			
			else if(msg.cmd == MAV_CMD_CAMERA_STOP_TRACKING)
			{// 停止目标跟踪
				sdata[0]=0xff;
				sdata[1]=0x01;
				sdata[2]=0x11;
				sdata[3]=0x01;
				sdata[4]=0x00;
				sdata[5]=0x00;	
			  cameraCheckSum(sdata,1,5,6);				
				driver_info.port.write(sdata,7,0.1,1);
				send_ack(msg.cmd, 1); 
			}
			else if(msg.cmd == MAV_CMD_DO_GIMBAL_ENABLE_OSD)
			{// 开启OSD	
				sdata[0]=0xff;
				sdata[1]=0x01;
				sdata[2]=0x12;
				sdata[3]=0x07;
				sdata[4]=0x00;
				sdata[5]=0x00;	
			  cameraCheckSum(sdata,1,5,6);				
				driver_info.port.write(sdata,7,0.1,1);
				send_ack(msg.cmd, 1); 					
			}	
			else if(msg.cmd == MAV_CMD_DO_GIMBAL_DISABLE_OSD)
			{// 关闭OSD	
				sdata[0]=0xff;
				sdata[1]=0x01;
				sdata[2]=0x12;
				sdata[3]=0x08;
				sdata[4]=0x00;
				sdata[5]=0x00;
				cameraCheckSum(sdata,1,5,6);	
				driver_info.port.write(sdata,7,0.1,1);
				send_ack(msg.cmd, 1); 					
			}				
		}	
		if(heartBeatSendTime.get_pass_time()>0.99)
		{
			//发送heartBeat
			send_heartbeat();
			heartBeatSendTime=TIME::now();
		}			
	}
}

static bool YT_INYYO_DriverInit( Port port, uint32_t param )
{
	//波特率19200
	port.SetBaudRate( 115200, 2, 2 );

	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	
	DriverInfo* driver_info2 = new DriverInfo;
	driver_info2->param = param;
	driver_info2->port = port;
	xTaskCreate( YT_INYYO_Server, "YT_INYYO",  812, (void*)driver_info, SysPriority_ExtSensor, NULL);
	xTaskCreate( YT_INYYO_Read, 	"YT_INYYO2", 256, (void*)driver_info2, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_YT_INYYO()
{
	PortFunc_Register( 102, YT_INYYO_DriverInit );
}