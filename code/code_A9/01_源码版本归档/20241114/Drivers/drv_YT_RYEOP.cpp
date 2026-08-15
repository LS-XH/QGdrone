#include "drv_YT_RYEOP.hpp"
#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"
#include "AuxFuncs.hpp"
#include "Parameters.hpp"
#include "MavlinkCMDProcess.hpp"
#include "ControlSystem.hpp"
#include "followTarget.hpp"
#include "StorageSystem.hpp"

static uint8_t componentID = MAV_COMP_ID_CAMERA_RYEOP;

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
				componentID ,	  //component id
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
				uint8_t vendor_name[32]={"RYEOP"};
				uint8_t model_name[32]={"RYEOP"};
				uint32_t flags = CAMERA_CAP_FLAGS_HAS_GIMBAL_PITCH|
												 CAMERA_CAP_FLAGS_HAS_GIMBAL_YAW|
												 CAMERA_CAP_FLAGS_HAS_GIMBAL_CENTER|
												 //CAMERA_CAP_FLAGS_HAS_GIMBAL_DOWN|
												 //CAMERA_CAP_FLAGS_GIMBAL_CTRL_SEND_WHEN_CHANGED|
												 //CAMERA_CAP_FLAGS_CAPTURE_IMAGE|
												 //CAMERA_CAP_FLAGS_CAPTURE_VIDEO|
												 CAMERA_CAP_FLAGS_HAS_DOUBLE_VIDEO|
												 CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM|
												 CAMERA_CAP_FLAGS_HAS_MODES|
												 CAMERA_CAP_FLAGS_HAS_TRACKING_POINT|
												 //CAMERA_CAP_FLAGS_TARGET_DETECTION|
												 CAMERA_CAP_FLAGS_SHOW_CAMERA_AREA|
												 CAMERA_CAP_FLAGS_TARGET_SELECTION_RECTANGLE|
												 CAMERA_CAP_FLAGS_HAS_ACK;

				mavlink_msg_camera_information_pack_chan(
					get_CommulinkSysId() ,	//system id
					componentID ,	//component id
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



typedef struct
{
	uint8_t header[2];
	uint16_t len;
	uint8_t id[2];
	uint8_t cmd;
	uint8_t datas[32];
}__PACKED cmdPack;
static inline uint16_t calcSum( cmdPack* cmd, uint8_t dataLen )
{
	cmd->len = dataLen + 5 + 2;
	uint16_t* sum = (uint16_t*)(&cmd->datas[dataLen]);
	*sum = 0;
	for( uint16_t i=0; i<cmd->len-2; ++i )
	{
		*sum += ((uint8_t*)cmd)[2+i];
	}
	*sum &= 0xff;
	return cmd->len + 2;
}

static void YT_RYEOP_Read(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	//状态信息
	TIME targetAvTIME(false);
	uint16_t zooms[5];
	
	//数据状态机
	uint8_t rc_counter=0;
	uint8_t rdata=0;
	uint16_t sum=0;
	cmdPack pack;
	mavlink_camera_gimbal_status_t camera_gimbal_status;
	
	uint32_t followKey = followSensorRegister(followDataType_track_s_xy);
	
	while(1)
	{
		uint8_t readLen = driver_info.port.read( &rdata, 1, 1, 1 );
		if( readLen > 0 )
		{	
			((uint8_t*)&pack)[rc_counter++] = rdata;
			if( rc_counter == 1 )//帧头
			{
				if(rdata!=0xEB){
					rc_counter = 0;
				}
			}
			else if(rc_counter==2)//帧头
			{
				if(rdata!=0x90){
					rc_counter = 0;
				}
				sum = 0;
			}
			else if(rc_counter<=4)//帧长,消息长度24字节
			{
				sum += rdata;
				if( rc_counter == 4 )
				{
					if( pack.len > sizeof(cmdPack)-2 )
						rc_counter = 0;
					if( pack.len < 7 )
						rc_counter = 0;
				}
			}
			else if(rc_counter<=pack.len)//数据
			{
				sum += rdata;
			}
			else if(rc_counter==pack.len+2)//校验
			{
				if( *(uint16_t*)(&pack.datas[pack.len-7]) == (sum&0xff) )
				{
					switch(pack.cmd)
					{
						case 0xb5:
						{	//基础信息
							typedef struct
							{
								/*当前模式
									None = 0xD1;
									DetectMode = 0xD2;
									TrackMode = 0xD3;
								*/
								uint8_t mode;
								/*云台状态
									0x00 否
									0x01 是
								*/
								uint8_t status;
								/*方位角
								*/
								int16_t yaw;
								/*俯仰角
								*/
								int16_t pit;
								/*横滚角
								*/
								int16_t rol;
								/*测距有效
								*/
								uint8_t distAvailable;
								/*测距值
								*/
								uint16_t dist;
								/*所有镜头当前变焦倍数的数组
									两位小数
								*/
								uint16_t zooms[5];
								/*画面中心温度
								*/
								int16_t tempCenter;
								/*最高温度
								*/
								int16_t tempMax;
								/*最低温度
								*/
								int16_t tempMin;
								/*云台模式
								*/
								int8_t gimbalMode;
							}__PACKED SKYNODE_BASEINFO_S;
							SKYNODE_BASEINFO_S* packData = (SKYNODE_BASEINFO_S*)pack.datas;
							
							memcpy( zooms, packData->zooms, sizeof(uint16_t)*5 );
							
							if( !targetAvTIME.is_valid() || targetAvTIME.get_pass_time()>1 )
							{
								float zoomLevel=0;
								camera_gimbal_status.gimBal_pitch = packData->pit*0.01 + 90;//云台俯仰角度(单位0.1°)
								camera_gimbal_status.gimBal_yaw = packData->yaw*0.01;  //云台航向角度(单位0.1°)
								camera_gimbal_status.currentZoomLevel = packData->zooms[0]*0.01;//可见光变焦倍数(整数)
								camera_gimbal_status.zoomLevel_max=40;
								camera_gimbal_status.zoomLevel_min=1;
								camera_gimbal_status.hfov_max=61.11;
								camera_gimbal_status.hfov_min=4.12;
								camera_gimbal_status.vfov_max=35.2;
								camera_gimbal_status.vfov_min=4.12;
								camera_gimbal_status.track_mode = 0;
								camera_gimbal_status.yaw_mode=0;
								camera_gimbal_status.flag=1;
								camera_gimbal_status.target_xMin = -1;
								camera_gimbal_status.target_yMin = -1;
								camera_gimbal_status.target_xMax = -1;
								camera_gimbal_status.target_yMax = -1;
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
							break;
						}
						
						case 0xb4:
						{	//设备信息
							typedef struct
							{
								/*设备编号
								*/
								uint8_t id;
								/*设备类型
								*/
								uint8_t type;
								/*设备类型
									554开头
								*/
								uint16_t port;
								/*设备状态
								*/
								uint8_t status;
							}__PACKED deviceInfo;
							typedef struct
							{
								/*是否测距
									0x01TRUE 0x00False
								*/
								uint8_t distAvailable;
								/*设备数量
								*/
								uint8_t devNum;
								//数据
								deviceInfo* infos;
							}__PACKED SKYNODE_DEVINFO_S;
							
							SKYNODE_DEVINFO_S* packData = (SKYNODE_DEVINFO_S*)pack.datas;
							break;
						}
						
						case 0xB7:
						{	//跟踪信息
							typedef struct
							{
								/*设备编号
								*/
								uint8_t id;
								/*设备类型
								*/
								uint8_t devType;
								/*镜头zoom
								*/
								uint16_t zoom;
								/*目标是否丢失
									0x00否
									0x01是
								*/
								uint8_t targetLost;
								/*目标类型
								*/
								uint8_t targetType;
								/*坐标百分比
								*/
								uint16_t xmin;
								uint16_t ymin;
								uint16_t xmax;
								uint16_t ymax;
								/*分值
								*/
								uint16_t score;
								/*脱靶量
									目标距离光轴中心的俯仰角度
									两位小数
								*/
								int16_t dstAndCenterPitchAngle;
								/*脱靶量
									目标距离光轴中心的航向角度
									两位小数
								*/
								int16_t dstAndCenterYawAngle;
								/*跟踪俯仰角速度
								*/
								int16_t pitRate;
								/*跟踪偏航角速度
								*/
								int16_t yawRate;
								/*方位角
								*/
								int16_t yaw;
								/*俯仰角
								*/
								int16_t pit;
								/*横滚角
								*/
								int16_t rol;
							}__PACKED SKYNODE_TRACKINFO_S;
							
							SKYNODE_TRACKINFO_S* packData = (SKYNODE_TRACKINFO_S*)pack.datas;
							packData->dstAndCenterYawAngle = -packData->dstAndCenterYawAngle;
							
							Quaternion quat;
							get_AirframeY_quat(&quat);
							
							extern float debug_test[30];
							debug_test[15] = rad2degree( quat.getPitch() + -degree2rad((packData->pit+packData->dstAndCenterPitchAngle)*0.01) );
							debug_test[16] = packData->yaw*0.01;
							
							/* 吊仓垂直向下为-90度，向前为0度，航向向左为正，需改成:
							俯仰垂直向下为0度，向前为90度，NED坐标系，航向向右为正，0-360*/
							float zoomLevel=0;
							camera_gimbal_status.gimBal_pitch = packData->pit*0.01 + 90;//云台俯仰角度(单位0.1°)
							camera_gimbal_status.gimBal_yaw = -packData->yaw*0.01;  //云台航向角度(单位0.1°)
							camera_gimbal_status.currentZoomLevel = zooms[0]*0.01;//可见光变焦倍数(整数)
							camera_gimbal_status.zoomLevel_max=40;
							camera_gimbal_status.zoomLevel_min=1;
							camera_gimbal_status.hfov_max=61.11;
							camera_gimbal_status.hfov_min=4.12;
							camera_gimbal_status.vfov_max=35.2;
							camera_gimbal_status.vfov_min=4.12;
							camera_gimbal_status.track_mode = 
								(!packData->targetLost) ? 1 : 0;
							camera_gimbal_status.yaw_mode=0;
							camera_gimbal_status.flag=1;
							if( packData->targetLost )
							{
								camera_gimbal_status.target_xMin = -1;
								camera_gimbal_status.target_yMin = -1;
								camera_gimbal_status.target_xMax = -1;
								camera_gimbal_status.target_yMax = -1;
								camera_gimbal_status.target_lat = std::nan("");
								camera_gimbal_status.target_lon = std::nan("");
								camera_gimbal_status.target_alt = std::nan("");
							}
							else
							{
								camera_gimbal_status.target_xMin = packData->xmin*255/10000;
								camera_gimbal_status.target_yMin = packData->ymin*255/10000;
								camera_gimbal_status.target_xMax = packData->xmax*255/10000;
								camera_gimbal_status.target_yMax = packData->ymax*255/10000;
								
								vector3<double> Position;
								get_Position_Ctrl(&Position);
								
								double homeZ;
								double heightAboveGround = 0;
								if( getHomeLocalZ( &homeZ, 0, 0.01 ) )
									heightAboveGround = Position.z - homeZ;
								if( heightAboveGround < 0.1 )
									heightAboveGround = 0.1;
								
								double pit = quat.getPitch() + -degree2rad((packData->pit+packData->dstAndCenterPitchAngle)*0.01);
								if( pit > 0.0001 )
								{
									double targetDist = heightAboveGround / tan(pit);
									double yaw = quat.getYaw() + degree2rad((packData->yaw+packData->dstAndCenterYawAngle)*0.01);
									double sinYaw, cosYaw;
									fast_sin_cos( yaw, &sinYaw, &cosYaw );
									double targetOfsX = BodyHeading2ENU_x( targetDist, 0 , sinYaw, cosYaw );
									double targetOfsY = BodyHeading2ENU_y( targetDist, 0 , sinYaw, cosYaw );
									
									double lat, lon;
									PosSensorHealthInf2 global_posInf;
									if( get_OptimalGlobal_XY(&global_posInf) )
									{
										map_projection_reproject( &global_posInf.mp, 
												global_posInf.PositionENU.x+targetOfsX+global_posInf.HOffset.x, 
												global_posInf.PositionENU.y+targetOfsY+global_posInf.HOffset.y,
												&lat, &lon );
									}
									camera_gimbal_status.target_lat = lat;
									camera_gimbal_status.target_lon = lon;
									camera_gimbal_status.target_alt = 0;
									
									vector3<double> pos;
									pos.x = Position.x + targetOfsX;
									pos.y = Position.y + targetOfsY;
									pos.z = homeZ;
									vector3<double> vel;
									update_followSensor( followKey, pos, vel, yaw, true );
									
									double vv[4];
									vv[0] = rad2degree(pit);
									vv[1] = rad2degree(yaw);
									vv[2] = targetDist;
									SDLog_Msg_DebugVect( "sdeww", vv, 3 );
								}
								else
								{
									camera_gimbal_status.target_lat = std::nan("");
									camera_gimbal_status.target_lon = std::nan("");
									camera_gimbal_status.target_alt = std::nan("");
								}
							}
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
							targetAvTIME = TIME::now();
							
							break;
						}
					}
					
				}
				rc_counter=0;
			}			
		}		
	}		
}

static void YT_RYEOP_Server(void* pvParameters)
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
	TIME fcInfoSendTime;
	TIME heartBeatSendTime;
	TIME cmdActiveTime;

	cmdPack cmd;
	cmd.header[0] = 0xeb; cmd.header[1] = 0x90;
	cmd.id[0] = 0x13; cmd.id[1] = 0x13;
	uint8_t sdata[32] = { 0xeb,0x90, 0x00,0x10, 0x13,0x13, 0x00 };
	
	if(0)
	{	//请求设备信息
		typedef struct
		{
			/*信息返回频率
				0 ~ 100HZ
			*/
			uint16_t freq;
		}__PACKED SKYNODE_QUERYDEV_S_CmdPack;
		cmd.cmd = 0xA0;
		SKYNODE_QUERYDEV_S_CmdPack* cmdData = (SKYNODE_QUERYDEV_S_CmdPack*)cmd.datas;
		cmdData->freq = 10;
		uint16_t n = calcSum( &cmd, sizeof(SKYNODE_QUERYDEV_S_CmdPack) );
		driver_info.port.write( (uint8_t*)&cmd, n, 0.1, 0.1 );
	}
	
	while(1)
	{
		Receiver rc;
		getReceiver(&rc);
		
//		typedef struct
//		{
//			/*是否移动方位角
//				0xff 右
//				0x00 不动
//				0x01 左
//			*/
//			int8_t moveYaw;
//			/*方位角角度
//				0~36000
//			*/
//			uint16_t angleYaw;
//			/*转动速度
//				0~10000
//			*/
//			uint16_t speedYaw;
//			
//			/*是否移动俯仰角
//				0xff 下
//				0x00 不动
//				0x01 上
//			*/
//			int8_t movePit;
//			/*角度
//				0~36000
//			*/
//			uint16_t anglePit;
//			/*转动速度
//				0~10000
//			*/
//			uint16_t speedPit;
//			
//			/*是否移动俯横滚
//				0xff 下
//				0x00 不动
//				0x01 上
//			*/
//			int8_t moveRol;
//			/*角度
//				0~36000
//			*/
//			uint16_t angleRol;
//			/*转动速度
//				0~10000
//			*/
//			uint16_t speedRol;
//		}__PACKED PTZ_MODE_AXIS_S;
//		cmd.cmd = 0xA5;
//		PTZ_MODE_AXIS_S* cmdData = (PTZ_MODE_AXIS_S*)cmd.datas;
//		cmdData->moveYaw = 0;
//		cmdData->movePit= 1;
//		cmdData->anglePit = 3000;
//		cmdData->speedPit = 1000;
//		cmdData->moveRol = 0;
//		uint16_t n = calcSum( &cmd, sizeof(PTZ_MODE_AXIS_S) );
//		driver_info.port.write( (uint8_t*)&cmd, n, 0.1, 0.1 );
		
		
		
//		typedef struct
//		{
//			/*云台模式
//				0x00:无模式
//				0x01:机头模式
//				0x02:自稳模式
//				0x03:随动模式
//			*/
//			uint8_t mode;
//			/*预留
//			*/
//			uint8_t rsv;
//		}__PACKED SKYNODE_COMMON_S;
//		cmd.cmd = 0xCB;
//		SKYNODE_COMMON_S* cmdData = (SKYNODE_COMMON_S*)cmd.datas;
//		memset( cmdData, 0, sizeof(SKYNODE_COMMON_S) );
//		cmdData->mode = 2;
//		uint16_t n = calcSum( &cmd, sizeof(SKYNODE_COMMON_S) );
//		driver_info.port.write( (uint8_t*)&cmd, n, 0.1, 0.1 );
		
		
		
		
		
		msg.cmd=0;
		bool msgAvailable=ReceiveCmdMsgFromTask( &msg, componentID,0.2);
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
					typedef struct
					{
						/*移动类型
							0xC1云台回归初始位
							0xC2方位角移动
							0xC3俯仰角移动
							0xC4横滚角移动
						*/
						uint8_t moveType;
						/*移动类型
							0x00左或者上
							0x01右或者下
						*/
						uint8_t moveDir;
						/*转动角度
							0~36000 如果为0或者超出范围默认为5度
						*/
						uint16_t angle;
						/*转动速度
							0~10000如果为0或者超出范围默认为100
						*/
						uint16_t speed;
					}__PACKED PTZ_S_CmdPack;
					
					
					if( fabs(msg.params[2]) > fabs(msg.params[3]) )
					{	//Pitch
						if(msg.params[2]>0)
						{
							cmd.cmd = 0xA1;
							PTZ_S_CmdPack* cmdData = (PTZ_S_CmdPack*)cmd.datas;
							cmdData->moveType = 0xC3;
							cmdData->moveDir = 0x00;
							cmdData->angle = 500;
							cmdData->speed = 1000;
							uint16_t n = calcSum( &cmd, sizeof(PTZ_S_CmdPack) );
							driver_info.port.write( (uint8_t*)&cmd, n, 0.1, 0.1 );
						}
						else if(msg.params[2]<0)
						{
							cmd.cmd = 0xA1;
							PTZ_S_CmdPack* cmdData = (PTZ_S_CmdPack*)cmd.datas;
							cmdData->moveType = 0xC3;
							cmdData->moveDir = 0x01;
							cmdData->angle = 500;
							cmdData->speed = 1000;
							uint16_t n = calcSum( &cmd, sizeof(PTZ_S_CmdPack) );
							driver_info.port.write( (uint8_t*)&cmd, n, 0.1, 0.1 );
						}
					}
					else
					{	//Yaw
						if(msg.params[3]>0)
						{
							cmd.cmd = 0xA1;
							PTZ_S_CmdPack* cmdData = (PTZ_S_CmdPack*)cmd.datas;
							cmdData->moveType = 0xC2;
							cmdData->moveDir = 0x00;
							cmdData->angle = 500;
							cmdData->speed = 1000;
							uint16_t n = calcSum( &cmd, sizeof(PTZ_S_CmdPack) );
							driver_info.port.write( (uint8_t*)&cmd, n, 0.1, 0.1 );
						}
						else if(msg.params[3]<0)
						{
							cmd.cmd = 0xA1;
							PTZ_S_CmdPack* cmdData = (PTZ_S_CmdPack*)cmd.datas;
							cmdData->moveType = 0xC2;
							cmdData->moveDir = 0x01;
							cmdData->angle = 500;
							cmdData->speed = 1000;
							uint16_t n = calcSum( &cmd, sizeof(PTZ_S_CmdPack) );
							driver_info.port.write( (uint8_t*)&cmd, n, 0.1, 0.1 );
						}
					}
				}
				send_ack(msg.cmd, 1); 		
			}	
			else if(msg.cmd == MAV_CMD_DO_GIMBAL_MANAGER_CENTER)
			{// 一键回中
				typedef struct
				{
					/*移动类型
						0xC1云台回归初始位
						0xC2方位角移动
						0xC3俯仰角移动
						0xC4横滚角移动
					*/
					uint8_t moveType;
					/*移动类型
						0x00左或者上
						0x01右或者下
					*/
					uint8_t moveDir;
					/*转动角度
						0~36000 如果为0或者超出范围默认为5度
					*/
					uint16_t angle;
					/*转动速度
						0~10000如果为0或者超出范围默认为100
					*/
					uint16_t speed;
				}__PACKED PTZ_S_CmdPack;
				cmd.cmd = 0xA1;
				PTZ_S_CmdPack* cmdData = (PTZ_S_CmdPack*)cmd.datas;
				cmdData->moveType = 0xC1;
				cmdData->moveDir = 0x00;
				cmdData->angle = 0;
				cmdData->speed = 0;
				uint16_t n = calcSum( &cmd, sizeof(PTZ_S_CmdPack) );
				driver_info.port.write( (uint8_t*)&cmd, n, 0.1, 0.1 );
				
				send_ack(msg.cmd, 1); 			
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
				typedef struct
				{
					/*设备编号
					*/
					uint8_t id;
					/*变倍方向
						0x00放大
						0x01缩小
						0x02指定倍率
					*/
					uint8_t dir;
					/*变倍速度
						预留变倍速度值
					*/
					uint8_t speed;
					/*目标倍率
						广角：1~10
						变焦：1~30
					*/
					uint16_t target;
				}__PACKED SKYNODE_ZOOM_S;
				cmd.cmd = 0xA2;
				SKYNODE_ZOOM_S* cmdData = (SKYNODE_ZOOM_S*)cmd.datas;
				
				cmdData->id = 0;
				if(msg.params[1]==-1){ //放大
					cmdData->dir = 0;
					uint16_t n = calcSum( &cmd, sizeof(SKYNODE_ZOOM_S) );
					driver_info.port.write( (uint8_t*)&cmd, n, 0.1, 0.1 );
				}
				else if(msg.params[1]==0){// 停止缩放
				}
				else if(msg.params[1]==1){// 缩小
					cmdData->dir = 1;
					uint16_t n = calcSum( &cmd, sizeof(SKYNODE_ZOOM_S) );
					driver_info.port.write( (uint8_t*)&cmd, n, 0.1, 0.1 );
				}				
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
				if(msg.params[0]>0&&msg.params[1]>0)
				{
					typedef struct
					{
						/*设备编号
						*/
						uint8_t id;
						/*云台跟踪
							0x00 否
							0x01 是
						*/
						uint8_t track;
						/*目标分类
							0跟踪检测框
							1画框跟踪有边界目标
							azzzzzzzzzz	`2画框跟踪无边界目标
						*/
						uint8_t targetClass;
						/*按画面比例
						*/
						uint16_t xMin;
						/*按画面比例
						*/
						uint16_t yMin;
						/*按画面比例
						*/
						uint16_t xMax;
						/*按画面比例
						*/
						uint16_t yMax;
					}__PACKED SKYNODE_SETTRACKMODE_S;
					cmd.cmd = 0xA4;
					SKYNODE_SETTRACKMODE_S* cmdData = (SKYNODE_SETTRACKMODE_S*)cmd.datas;
					memset( cmdData, 0, sizeof(SKYNODE_SETTRACKMODE_S) );
					cmdData->id = 0;
					cmdData->track = 1;
					cmdData->targetClass = 1;
					cmdData->xMin = msg.params[0]*10000/255;
					cmdData->yMin = msg.params[1]*10000/255;
					cmdData->xMax = msg.params[2]*10000/255;
					cmdData->yMax = msg.params[3]*10000/255;
					uint16_t n = calcSum( &cmd, sizeof(SKYNODE_SETTRACKMODE_S) );
					driver_info.port.write( (uint8_t*)&cmd, n, 0.1, 0.1 );
					
					send_ack(msg.cmd, 1);
				}
			}			
			else if(msg.cmd == MAV_CMD_CAMERA_STOP_TRACKING)
			{// 停止目标跟踪
				typedef struct
				{
					/*移动类型
						0xC1云台回归初始位
						0xC2方位角移动
						0xC3俯仰角移动
						0xC4横滚角移动
					*/
					uint8_t moveType;
					/*移动类型
						0x00左或者上
						0x01右或者下
					*/
					uint8_t moveDir;
					/*转动角度
						0~36000 如果为0或者超出范围默认为5度
					*/
					uint16_t angle;
					/*转动速度
						0~10000如果为0或者超出范围默认为100
					*/
					uint16_t speed;
				}__PACKED PTZ_S_CmdPack;
				
				cmd.cmd = 0xA1;
				PTZ_S_CmdPack* cmdData = (PTZ_S_CmdPack*)cmd.datas;
				cmdData->moveType = 0xC3;
				cmdData->moveDir = 0x00;
				cmdData->angle = 1;
				cmdData->speed = 1000;
				uint16_t n = calcSum( &cmd, sizeof(PTZ_S_CmdPack) );
				driver_info.port.write( (uint8_t*)&cmd, n, 0.1, 0.1 );
				
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

static bool YT_RYEOP_DriverInit( Port port, uint32_t param )
{
	// 注册消息队列
	TaskQueueRegister(componentID,30);
	
	//波特率19200
	port.SetBaudRate( 115200, 2, 2 );

	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	
	DriverInfo* driver_info2 = new DriverInfo;
	driver_info2->param = param;
	driver_info2->port = port;
	xTaskCreate( YT_RYEOP_Server, "YT_RYEOP",  812, (void*)driver_info, SysPriority_ExtSensor, NULL);
	xTaskCreate( YT_RYEOP_Read, 	"YT_RYEOP2", 256, (void*)driver_info2, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_YT_RYEOP()
{
	PortFunc_Register( 103, YT_RYEOP_DriverInit );
}