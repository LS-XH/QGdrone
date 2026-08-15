#include "AuxFuncs.hpp"
#include "Basic.hpp"
#include "Parameters.hpp"
#include "Receiver.hpp"
#include "drv_PWMOut.hpp"
#include "Commulink.hpp"
#include "StorageSystem.hpp"
#include "MeasurementSystem.hpp"
#include "ControlSystem.hpp"
#include "event_groups.h"
#include "semphr.h"
#include "usb_composite.h"
#include "MavlinkCMDProcess.hpp"



//保存之前通道值
static double last_Channel_values[16];
//通道临时变量
static uint32_t channelTemp1[16] = {0};
static float channelTemp2[16] = {0};
static float channelTemp3[16] = {0};
//云台是否自动控制
static float GimbalCtrl_LockedAtt[16];

/*热靴触发*/
	//拍照次数
	static uint16_t PhotoCnt = 0;
	//相片序号
	static uint16_t PhotoIndex = 1;

	//发送完成标志
	static EventGroupHandle_t IO_events = xEventGroupCreate();
	
	static SemaphoreHandle_t PosLogMutex = xSemaphoreCreateMutex();
	static bool SD_Pos_Record()
	{
		//获取时间
		RTC_TimeStruct RTC_Time;
		RTC_Time = Get_RTC_Time();
		//获取速度
		vector3<double> vel;
		get_VelocityENU_Ctrl(&vel);	
		
		//获取姿态
		Quaternion airframe_quat;
		get_Attitude_quat(&airframe_quat);
		airframe_quat.Enu2Ned();	
		
		static int8_t global_pos_ind = -1;
		PosSensorHealthInf3 posInf;
		if( global_pos_ind < 0 )
		{	//第一次获取最优位置传感器
			if( get_OptimalGlobal_XYZ(&posInf) )
				global_pos_ind = posInf.sensor_ind;
		}
		else
			get_PosSensorHealth_XYZ( &posInf, global_pos_ind );
		
		double lat = 0;
		double lon = 0;
		double alt = 0;
		double accN = 999999;
		double accE = 999999;
		double accD = 999999;
		uint8_t fix_type = 0;
		uint16_t week = 0;
		double TOW = 0;
		
		if( global_pos_ind >= 0 )
		{
			Position_Sensor gps_sensor;
			if( GetPositionSensor( global_pos_ind, &gps_sensor ) )
			{
				if( gps_sensor.data.available && gps_sensor.data.sensor_type==Position_Sensor_Type_GlobalPositioning )
				{
					//计算经纬度
					map_projection_reproject( &posInf.mp, 
						posInf.PositionENU.x+posInf.HOffset.x, 
						posInf.PositionENU.y+posInf.HOffset.y,
						&lat, &lon );
					//高度
					alt = posInf.PositionENU.z + posInf.HOffset.z;
					alt *= 0.01;
					//精度
					accN = gps_sensor.inf.addition_inf[4]*0.01;
					accE = gps_sensor.inf.addition_inf[4]*0.01;
					accD = gps_sensor.inf.addition_inf[5]*0.01;
					//week
					week = gps_sensor.inf.addition_inf[2];
					//TOW
					TOW = gps_sensor.inf.addition_inf[3];
					//fix
					if( gps_sensor.inf.addition_inf[1]==1 )
						fix_type=0;
					else if( gps_sensor.inf.addition_inf[1]==5 )
						fix_type=34;
					else if( gps_sensor.inf.addition_inf[1]==6 )
						fix_type=50;
					else
						fix_type=16;
				}
			}
		}		

		xSemaphoreTake( PosLogMutex, portMAX_DELAY );
			uint16_t photo_ind = PhotoIndex;
			char pos_txt_buf[200];
			int n = sprintf(
				pos_txt_buf,
				"%4d\t%6.6f\t[%4d]\t%4d,N\t%4d,E\t%4d,V \t%11.8f,Lat \t%11.8f,Lon \t%6.3f,Ellh \t%9.6f,    %9.6f,    %9.6f,    %2d,Q\r\n",
				PhotoIndex++,
				TOW,
				week,
				0,
				0,
				0,
				lat,
				lon,
				alt,
				accN,
				accE,
				accD,	
				fix_type
			);			
			
		xSemaphoreGive(PosLogMutex);
		
		if(!SDLog_Txt1( pos_txt_buf, n ))
			return false;
		
		//发送拍照信息到地面??
		double Altitude_Local=0;
		vector3<double> Position;
		get_Position_Ctrl(&Position);
		double homeZ;
		double heightAboveGround = 0;
		if( getHomeLocalZ( &homeZ, 0, 0.01 ) )
			heightAboveGround = Position.z - homeZ;
		
		mavlink_message_t msg_sd;
		for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
		{	//遍历所有端口	
			if( mavlink_lock_chan( i, 2/configTICK_RATE_HZ ) )
			{
				mavlink_msg_camera_feedback_pack_chan(
					get_CommulinkSysId() ,	// system id
					get_CommulinkCompId() ,	// component id
					i ,
					&msg_sd,	  
					TIME::get_System_Run_Time() * 1e3 , // boot ms					
					1, //target_system  System ID
					global_pos_ind, // camera_id
					photo_ind, //Image index
					lat*1e7, // lat [degE7]
					lon*1e7, // lon [degE7]
					alt,  // alt_msl [m] Altitude (MSL).
					heightAboveGround*0.01, //alt_rel [m] Altitude (Relative to HOME location).
					0, //Camera Roll
					0, //Camera Pitch 
					0, //Camera Yaw
					30, //foc_len [mm] Focal Length
					0,  //0:Shooting photos, not video
					photo_ind	//Completed image captures
				);
				const Port* port = get_CommuPort(i);
				if(port->write){
					mavlink_msg_to_send_buffer(port->write, 
																	 port->lock,
																	 port->unlock,
																	 &msg_sd, 0, 2/configTICK_RATE_HZ);
				}
				mavlink_unlock_chan(i);	
			}	
		}
		return true;
	}

	void wait_IOTrig_start()
	{
		
	}

	bool wait_IOTrig( double TIME )
	{
		return true;
	}
/*热靴触发*/

/*记录Pos*/
	bool recordPos( bool acTrig )
	{
		if(acTrig)
		{
			xSemaphoreTake( PosLogMutex, portMAX_DELAY );
				++PhotoCnt;
			xSemaphoreGive(PosLogMutex);
			
			mavlink_message_t msg_sd;
			for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
			{	//遍历所有端口
				if( mavlink_lock_chan( i, 2/configTICK_RATE_HZ ) )
				{
					mavlink_msg_camera_status_pack_chan(
						get_CommulinkSysId() ,	// system id
						get_CommulinkCompId() ,	// component id
						i ,
						&msg_sd,	  
						TIME::get_System_Run_Time() * 1e6 , // boot us	
						255,	//target_system
						1, //cam index
						PhotoCnt, //image index
						1	,	//event id
						0, // p1
						0, // p2
						0, // p3
						0 //p4
					);
					const Port* port = get_CommuPort(i);
					if(port->write){
						mavlink_msg_to_send_buffer(port->write, 
																		 port->lock,
																		 port->unlock,
																		 &msg_sd, 0, 2/configTICK_RATE_HZ);
					}
					mavlink_unlock_chan(i);	
				}	
			}
		}
		return SD_Pos_Record();
	}
/*记录Pos*/

/*拍照功能*/
	static SemaphoreHandle_t CamMutex = xSemaphoreCreateRecursiveMutex();
	bool AuxCamTakePhoto()
	{
		if( getInitializationCompleted() == false )
			return false;
		if(get_is_usb_connected())
			return false;
		uint8_t cam_chans = 0;
		AuxFuncsConfig aux_configs;
		ReadParamGroup( "AuxCfg", (uint64_t*)&aux_configs, 0 );
		if( xSemaphoreTakeRecursive( CamMutex, 0.1*configTICK_RATE_HZ ) )
		{
			uint8_t MainMotorCount = get_MainMotorCount();		
			if( aux_configs.Aux_CamTrigEna[0] )
				wait_IOTrig_start();
			
			//拉低（拉高）相机PWM
			for( uint8_t i = MainMotorCount; i < PWMChannelsCount; ++i )
			{
				uint16_t aux_cfg = ((uint16_t*)&aux_configs)[i*4];
				if( (aux_cfg>=25 && aux_cfg<=48) || (aux_cfg>=1025 && aux_cfg<=1048) )
				{
					Aux_PWM_Out( aux_configs.Aux_CamOnPwm[0]*0.1-100, i );
					++cam_chans;
				}
			}
			
			if(cam_chans==0)
			{	//无相机返回
				uint8_t res = 0;
				
				if(aux_configs.Aux_YTSurveyPic[0]>0)
				{//数字云台需要拍照
					if( !aux_configs.Aux_CamTrigEna[0] )
						wait_IOTrig_start();
					
					CmdMsg cmd_msg;
					cmd_msg.cmd = MAV_CMD_IMAGE_START_CAPTURE;
					SendCmdMsgToSingleCamera( cmd_msg, 0.01 );
					
					if( aux_configs.Aux_CamTrigEna[0]>0 )
					{	//数字云台需要等待trig引脚触发
						if( wait_IOTrig(aux_configs.Aux_CamShTime[0] > 1.5 ? 1.5 : aux_configs.Aux_CamShTime[0]) )
						 res = 10;
					}
					else
					{	//数字云台等待指令回馈
						wait_IOTrig(0);
						
						double waitT = 0;
						while(1)
						{
							CmdMsg msg;
							bool msg_rd = ReceiveCmdMsgFromTask( &msg, CAMERA_FEEDBACK_QUEUE_ID, 0.002 );
							if( msg_rd )
							{	//相机反馈成功
								res = msg.cmd;
								break;
							}
							
							if( waitT>aux_configs.Aux_CamShTime[0] || waitT>1.5 )
								break;
							waitT += 0.01;
							os_delay(0.01);
						}
					}
				}
				
				xSemaphoreGiveRecursive(CamMutex);
				if(res)
				{
					xSemaphoreTake( PosLogMutex, portMAX_DELAY );
						++PhotoCnt;
					xSemaphoreGive(PosLogMutex);
					
					mavlink_message_t msg_sd;
					for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
					{	//遍历所有端口
						if( mavlink_lock_chan( i, 2/configTICK_RATE_HZ ) )
						{
							mavlink_msg_camera_status_pack_chan(
								get_CommulinkSysId() ,	// system id
								get_CommulinkCompId() ,	// component id
								i ,
								&msg_sd,	  
								TIME::get_System_Run_Time() * 1e6 , // boot us	
								255,	//target_system
								1, //cam index
								PhotoCnt, //image index
								1	,	//event id
								0, // p1
								0, // p2
								0, // p3
								0 //p4
							);
							const Port* port = get_CommuPort(i);
							if(port->write){
								mavlink_msg_to_send_buffer(port->write, 
																				 port->lock,
																				 port->unlock,
																				 &msg_sd, 0, 2/configTICK_RATE_HZ);
							}
							mavlink_unlock_chan(i);	
						}	
					}
					
					//如果无需等待内部触发则立刻记录POS
					if( res != 3 )
						SD_Pos_Record();
					
					return true;
				}
				else
					return false;
			}
			
			//拉高（拉低）相机PWM
			os_delay(aux_configs.Aux_CamShTime[0]);
			for( uint8_t i = MainMotorCount; i < PWMChannelsCount; ++i )
			{
				uint16_t aux_cfg = ((uint16_t*)&aux_configs)[i*4];
				if( (aux_cfg>=25 && aux_cfg<=48) || (aux_cfg>=1025 && aux_cfg<=1048) )
					Aux_PWM_Out( aux_configs.Aux_CamOffPwm[0]*0.1-100, i );
			}
			
			xSemaphoreTake( PosLogMutex, portMAX_DELAY );
				++PhotoCnt;
			xSemaphoreGive(PosLogMutex);
			
			mavlink_message_t msg_sd;
			for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
			{	//遍历所有端口
				if( mavlink_lock_chan( i, 2/configTICK_RATE_HZ ) )
				{
					mavlink_msg_camera_status_pack_chan(
						get_CommulinkSysId() ,	// system id
						get_CommulinkCompId() ,	// component id
						i ,
						&msg_sd,	  
						TIME::get_System_Run_Time() * 1e6 , // boot us	
						255,	//target_system
						1, //cam index
						PhotoCnt, //image index
						1	,	//event id
						0, // p1
						0, // p2
						0, // p3
						0 //p4
					);
					const Port* port = get_CommuPort(i);
					if(port->write){
						mavlink_msg_to_send_buffer(port->write, 
																		 port->lock,
																		 port->unlock,
																		 &msg_sd, 0, 2/configTICK_RATE_HZ);
					}
					mavlink_unlock_chan(i);	
				}	
			}
			
			if( aux_configs.Aux_CamTrigEna[0] )
			{	//等待相机热靴
				if( wait_IOTrig(aux_configs.Aux_CamShTime[0] > 1.5 ? 1.5 : aux_configs.Aux_CamShTime[0]) == false )
				{
					xSemaphoreGiveRecursive(CamMutex);
					return false;
				}
			}
			
			//如果热靴没有使能则立刻记录POS
			if( !aux_configs.Aux_CamTrigEna[0] )
				SD_Pos_Record();
			
			xSemaphoreGiveRecursive(CamMutex);
			return true;
		}
		else
			return false;
	}
	
	static uint8_t camPhotoAsyncStep = 0;
	static float camPhotoAsyncT = 0;
	static uint16_t lastCamPhotoAsyncPhotoIndex = 0;
	#define resetCamPhotoAsyncStep (camPhotoAsyncStep = camPhotoAsyncT = 0)
	bool AuxCamTakePhotoAsync(double TIMEOUT)
	{
		if( xSemaphoreTakeRecursive( CamMutex, TIMEOUT*configTICK_RATE_HZ ) )
		{
			resetCamPhotoAsyncStep;
			camPhotoAsyncStep = 1;
			camPhotoAsyncT = 0;
			xSemaphoreGiveRecursive(CamMutex);
			return true;
		}
		return false;
	}
	static void AuxCamTakePhotoAsyncServer(float h, const AuxFuncsConfig& aux_configs)
	{
		if( getInitializationCompleted()==false ||
				get_is_usb_connected()
			)
		{
			if( camPhotoAsyncStep > 1 )
				xSemaphoreGiveRecursive(CamMutex);
			camPhotoAsyncStep = camPhotoAsyncT = 0;
			//清空数字云台反馈缓冲区
			CmdMsg msg;
			ReceiveCmdMsgFromTask( &msg, CAMERA_FEEDBACK_QUEUE_ID, 0 );
			
			return;
		}
		
		if( camPhotoAsyncStep == 0 )
		{	//空闲时处理数字云台拍照反馈
			CmdMsg msg;
			bool msg_rd =ReceiveCmdMsgFromTask( &msg, CAMERA_FEEDBACK_QUEUE_ID, 0 );
			if( msg_rd && msg.cmd==1 )
			{
				xSemaphoreTake( PosLogMutex, portMAX_DELAY );
					++PhotoCnt;
				xSemaphoreGive(PosLogMutex);
				SD_Pos_Record();
			}
		}
		
		if( camPhotoAsyncStep > 1 )
		{
			camPhotoAsyncT += h;
			if( camPhotoAsyncT > 10 )
				resetCamPhotoAsyncStep;
		}
		
		switch(camPhotoAsyncStep)
		{
			case 1:
			{	//准备启动触发
				if( xSemaphoreTakeRecursive( CamMutex, 0.1*configTICK_RATE_HZ ) )
				{
					uint8_t cam_chans = 0;
					lastCamPhotoAsyncPhotoIndex = PhotoIndex;
						
					uint8_t MainMotorCount = get_MainMotorCount();
					//使能相机PWM
					for( uint8_t i = MainMotorCount; i < PWMChannelsCount; ++i )
					{
						uint16_t aux_cfg = ((uint16_t*)&aux_configs)[i*4];
						if( (aux_cfg>=25 && aux_cfg<=48) || (aux_cfg>=1025 && aux_cfg<=1048) )
						{
							Aux_PWM_Out( aux_configs.Aux_CamOnPwm[0]*0.1-100, i );
							++cam_chans;
						}
					}
					camPhotoAsyncT = 0;
					
					if(cam_chans==0)
					{	//无相机尝试数字云台
						uint8_t res = 0;
						
						if(aux_configs.Aux_YTSurveyPic[0]>0)
						{//数字云台需要拍照
							CmdMsg cmd_msg;
							cmd_msg.cmd = MAV_CMD_IMAGE_START_CAPTURE;
							SendCmdMsgToSingleCamera( cmd_msg, 0.01 );
							
							if( aux_configs.Aux_CamTrigEna[0]>0 )
							{	//数字云台需要等待trig引脚触发
								camPhotoAsyncStep = 20;
							}
							else
							{	//进入数字云台等待指令反馈步骤
								camPhotoAsyncStep = 30;
							}
						}
						else
						{
							resetCamPhotoAsyncStep;
							xSemaphoreGiveRecursive(CamMutex);
						}
					}
					else
					{	//相机引脚触发完成
						camPhotoAsyncStep = 10;
					}
				}
				else
					resetCamPhotoAsyncStep;
				break;
			}
			
			case 10:
			{	//相机引脚触发完成
				//准备失效相机引脚
				
				if( camPhotoAsyncT > aux_configs.Aux_CamShTime[0] )
				{
					//失效相机PWM
					uint8_t MainMotorCount = get_MainMotorCount();
					for( uint8_t i = MainMotorCount; i < PWMChannelsCount; ++i )
					{
						uint16_t aux_cfg = ((uint16_t*)&aux_configs)[i*4];
						if( (aux_cfg>=25 && aux_cfg<=48) || (aux_cfg>=1025 && aux_cfg<=1048) )
							Aux_PWM_Out( aux_configs.Aux_CamOffPwm[0]*0.1-100, i );
					}
					
					xSemaphoreTake( PosLogMutex, portMAX_DELAY );
						++PhotoCnt;
					xSemaphoreGive(PosLogMutex);
					
					mavlink_message_t msg_sd;
					for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
					{	//遍历所有端口
						if( mavlink_lock_chan( i, 2/configTICK_RATE_HZ ) )
						{
							mavlink_msg_camera_status_pack_chan(
								get_CommulinkSysId() ,	// system id
								get_CommulinkCompId() ,	// component id
								i ,
								&msg_sd,	  
								TIME::get_System_Run_Time() * 1e6 , // boot us	
								255,	//target_system
								1, //cam index
								PhotoCnt, //image index
								1	,	//event id
								0, // p1
								0, // p2
								0, // p3
								0 //p4
							);
							const Port* port = get_CommuPort(i);
							if(port->write){
								mavlink_msg_to_send_buffer(port->write, 
																				 port->lock,
																				 port->unlock,
																				 &msg_sd, 0, 2/configTICK_RATE_HZ);
							}
							mavlink_unlock_chan(i);	
						}	
					}
					
					if( aux_configs.Aux_CamTrigEna[0] )
					{	//等待相机热靴
						camPhotoAsyncStep = 20;
					}
					else
					{	//如果热靴没有使能则立刻记录POS
						SD_Pos_Record();
						camPhotoAsyncStep = 11;
						camPhotoAsyncT = 0;
//						resetCamPhotoAsyncStep;
//						xSemaphoreGiveRecursive(CamMutex);
					}
				}
				break;
			}
			case 11:
			{	//延时等待sh时间
				if( camPhotoAsyncT > aux_configs.Aux_CamShTime[0] )
				{
					resetCamPhotoAsyncStep;
					xSemaphoreGiveRecursive(CamMutex);
				}
				break;
			}
			
			case 20:
			{	//等待热靴触发
				if( camPhotoAsyncT>aux_configs.Aux_CamShTime[0] && (lastCamPhotoAsyncPhotoIndex!=PhotoIndex || camPhotoAsyncT>1.5) )
				{
					resetCamPhotoAsyncStep;
					xSemaphoreGiveRecursive(CamMutex);
				}
				break;
			}
			case 30:
			{	//数字云台等待触发
				CmdMsg msg;
				bool msg_rd = ReceiveCmdMsgFromTask( &msg, CAMERA_FEEDBACK_QUEUE_ID, 0 );
				if( msg_rd )
					SD_Pos_Record();
				if( msg_rd || camPhotoAsyncT>aux_configs.Aux_CamShTime[0] || camPhotoAsyncT>1.5 )
				{
					resetCamPhotoAsyncStep;
					xSemaphoreGiveRecursive(CamMutex);
				}
			}
		}
	}
/*拍照功能*/
bool setAuxPWM( float PWMus, uint8_t ind )
{
	if( getInitializationCompleted() == false )
		return false;
	if( PWMus<0 || PWMus>100000 )
		return false;
	
	AuxFuncsConfig aux_configs;
	ReadParamGroup( "AuxCfg", (uint64_t*)&aux_configs, 0 );
	uint16_t aux_cfg = ((uint16_t*)&aux_configs)[ind*4];
	if( (aux_cfg>=0 && aux_cfg<=16) || (aux_cfg>=1001 && aux_cfg<=1016) )
	{
		GimbalCtrl_LockedAtt[ind] = PWMus;
		return true;
	}
	return false;
}
	
void init_process_AuxFuncs()
{
//	Receiver rc;
//	if( getReceiver(&rc) )
//	{
//		//复位保存之前通道
//		for( uint8_t i = 0; i < rc.raw_available_channels; ++i )
//			last_Channel_values[i] = rc.raw_data[i];
//		for( uint8_t i = rc.raw_available_channels; i < 16; ++i )
//			last_Channel_values[i] = -200;
//	}
//	else
//	{
		//复位保存之前通道
		for( uint8_t i = 0; i < 16; ++i )
			last_Channel_values[i] = -200;
//	}
	//复位运动自动控制标志
	for( uint8_t i = 0; i < 16; ++i )
	{
		channelTemp1[i] = 0;
		channelTemp2[i] = 0;
		channelTemp3[i] = 0;
		GimbalCtrl_LockedAtt[i] = 200000;
	}
}

void process_AuxFuncs(const Receiver* rc, double h)
{
	AuxFuncsConfig aux_configs;
	ReadParamGroup( "AuxCfg", (uint64_t*)&aux_configs, 0 );

	AuxCamTakePhotoAsyncServer( h, aux_configs );
	
	uint8_t MainMotorCount = get_MainMotorCount();
	for( uint8_t i = MainMotorCount; i < PWMChannelsCount; ++i )
	{
		uint16_t aux_cfg = ((uint16_t*)&aux_configs)[i*4];
		float aux_param1 = ((float*)&aux_configs.Aux1Param1)[i*2];
		float aux_param2 = ((float*)&aux_configs.Aux1Param2)[i*2];
		if( aux_cfg==0 )
		{	//无功能(可通过指令控制PWM输出)
			if( GimbalCtrl_LockedAtt[i] < 100000 )
				Aux_PWM_Out( (GimbalCtrl_LockedAtt[i]-1000)*0.1, i );
		}
		if( aux_cfg>=1 && aux_cfg<=16 )
		{	//映射遥控器通道
			if( GimbalCtrl_LockedAtt[i] < 100000 )
				Aux_PWM_Out( (GimbalCtrl_LockedAtt[i]-1000)*0.1, i );
			uint8_t ref_chan = aux_cfg - 1;
			if( rc->connected && rc->raw_available_channels>ref_chan )
			{
				if( fabs(GimbalCtrl_LockedAtt[i])<100000 && last_Channel_values[i]>-100 )
				{	//Aux自动控制
					//通道变化大于阈值才调整Aux
					if( fabs(rc->raw_data[ref_chan]-last_Channel_values[i])>10 )
					{
						Aux_PWM_Out( (rc->raw_data[ref_chan]-50.0)*aux_param1+50+aux_param2*0.1, i );
						last_Channel_values[i] = rc->raw_data[ref_chan];
						GimbalCtrl_LockedAtt[i] = 200000;
					}
				}
				else
				{	//Aux手动控制
					Aux_PWM_Out( (rc->raw_data[ref_chan]-50.0)*aux_param1+50+aux_param2*0.1, i );
					last_Channel_values[i] = rc->raw_data[ref_chan];
				}
			}
			else
				last_Channel_values[i] = -200;
		}
		else if( aux_cfg>=501 && aux_cfg<=516 )
		{	//映射遥控器通道（开关量）
			uint8_t ref_chan = aux_cfg - 501;
			if( rc->connected && rc->raw_available_channels>ref_chan )
			{
				if( rc->raw_data[ref_chan] > 50 )
				{
					if( aux_param1 > 0 )
						Aux_PWM_Out( 6000, i );
					else
						Aux_PWM_Out( -100, i );
				}
				else
				{
					if( aux_param1 > 0 )
						Aux_PWM_Out( -100, i );
					else
						Aux_PWM_Out( 6000, i );
				}
			}
		}
		else if( aux_cfg>=25 && aux_cfg<=48 )
		{	//用遥控器对应通道进行相机快门触发(raw_data)
			if( xSemaphoreTakeRecursive( CamMutex, 0 ) )
			{
				if( camPhotoAsyncStep == 0 )
				{
					uint8_t ref_chan = aux_cfg - 25;
					if( rc->connected && rc->raw_available_channels>ref_chan )
					{
						if( last_Channel_values[i]>-100 )
						{
							if( fabs(rc->raw_data[ref_chan] - last_Channel_values[i]) > 15 )
							{	//触发相机
								if( last_Channel_values[i] > -100 )							
								{
									AuxCamTakePhoto();					
								}
								last_Channel_values[i] = rc->raw_data[ref_chan];
							}
							else
								Aux_PWM_Out( aux_configs.Aux_CamOffPwm[0]*0.1-100, i );
						}
						else
						{	//旧通道数据不可用
							Aux_PWM_Out( aux_configs.Aux_CamOffPwm[0]*0.1-100, i );
							last_Channel_values[i] = rc->raw_data[ref_chan];
						}
					}
					else
					{	//无遥控器
						Aux_PWM_Out( aux_configs.Aux_CamOffPwm[0]*0.1-100, i );
						last_Channel_values[i] = -200;
					}
				}
				xSemaphoreGiveRecursive(CamMutex);
			}
		}
		else if( aux_cfg>=49 && aux_cfg<=72 )
		{	//用遥控器对应通道进行无刷云台俯仰控制(raw_data)
			double angle90 = (aux_configs.Aux_BsYTPit90[0]-1000)*0.1;
			double angle0 = (aux_configs.Aux_BsYTPit0[0]-1000)*0.1;
			double scale = (angle90 - angle0)/90.0;
			if( GimbalCtrl_LockedAtt[i] < 200 )
				Aux_PWM_Out( (GimbalCtrl_LockedAtt[i]-0)*scale + angle0, i );
			
			uint8_t ref_chan = aux_cfg - 49;
			if( rc->connected && rc->raw_available_channels>ref_chan )
			{
				if( fabs(GimbalCtrl_LockedAtt[i])<200 && last_Channel_values[i]>-100 )
				{	//云台自动控制
					//通道变化大于阈值才调整云台
					if( fabs(rc->raw_data[ref_chan] - last_Channel_values[i]) > 10 )
					{
						float angle = (rc->raw_data[ref_chan]-50.0)*(60.0/50.0)*aux_param1 + 45+aux_param2; 
						angle = constrain( angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0] );
						Aux_PWM_Out( (angle-0)*scale + angle0, i );
						last_Channel_values[i] = rc->raw_data[ref_chan];
						GimbalCtrl_LockedAtt[i] = 200000;
					}
				}
				else
				{	//云台手动控制
					float angle = (rc->raw_data[ref_chan]-50.0)*(60.0/50.0)*aux_param1 + 45+aux_param2; 
					angle = constrain( angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0] );
					Aux_PWM_Out( (angle-0)*scale + angle0, i );
					last_Channel_values[i] = rc->raw_data[ref_chan];
				}
			}
			else
			{	//无遥控信号
				last_Channel_values[i] = -200;
				if( GimbalCtrl_LockedAtt[i] > 200 )
				{	//非自动控制且无遥控信号
					//锁定0角度
					float angle = 0;
					Aux_PWM_Out( (angle-0)*scale + angle0, i );
				}
			}
		}
		else if( aux_cfg>=73 && aux_cfg<=96 )
 		{	//用遥控器对应通道进行舵机云台俯仰控制(raw_data)
			double angle90 = (aux_configs.Aux_StYTPit90[0]-1000)*0.1;
			double angle0 = (aux_configs.Aux_StYTPit0[0]-1000)*0.1;
			double scale = (angle90 - angle0)/90.0;
			Quaternion quat;
			get_Airframe_quat(&quat);
			double pitch = rad2degree(quat.getPitch());
			if( GimbalCtrl_LockedAtt[i] < 200 )
				Aux_PWM_Out( (GimbalCtrl_LockedAtt[i]-pitch-0)*scale + angle0, i );
			
			uint8_t ref_chan = aux_cfg - 73;
			if( rc->connected && rc->raw_available_channels>ref_chan )
			{
				if( fabs(GimbalCtrl_LockedAtt[i])<200 && last_Channel_values[i]>-100 )
				{	//云台自动控制
					//通道变化大于阈值才调整云台
					if( fabs(rc->raw_data[ref_chan] - last_Channel_values[i]) > 10 )
					{
						float angle = (rc->raw_data[ref_chan]-50.0)*(60.0/50.0)*aux_param1 + 45+aux_param2; 							
						angle -= pitch;
						angle = constrain( angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0] );
						Aux_PWM_Out( (angle-0)*scale + angle0, i );
						last_Channel_values[i] = rc->raw_data[ref_chan];
						GimbalCtrl_LockedAtt[i] = 20000;
					}
				}
				else
				{	//云台手动控制
					float angle = (rc->raw_data[ref_chan]-50.0)*(60.0/50.0)*aux_param1 + 45+aux_param2; 
					angle -= pitch;
					angle = constrain( angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0] );
					Aux_PWM_Out( (angle-0)*scale + angle0, i );
					last_Channel_values[i] = rc->raw_data[ref_chan];
				}
			}
			else 
			{	//无遥控信号
				last_Channel_values[i] = -200;
				if( GimbalCtrl_LockedAtt[i] > 200 )
				{	//非自动控制且无遥控信号
					//锁定0角度
					float angle = 0;
					Aux_PWM_Out( (angle-pitch-0)*scale + angle0, i );
				}
			}
		}
		else if( aux_cfg>=273 && aux_cfg<=296 )
		{	//用遥控器对应通道进行舵机云台2俯仰控制(raw_data)
			double angle90 = (aux_configs.Aux_StYT2Pit90[0]-1000)*0.1;
			double angle0 = (aux_configs.Aux_StYT2Pit0[0]-1000)*0.1;
			double scale = (angle90 - angle0)/90.0;
			Quaternion quat;
			get_Airframe_quat(&quat);
			double pitch = rad2degree(quat.getPitch());
			if( GimbalCtrl_LockedAtt[i] < 200 )
				Aux_PWM_Out( (GimbalCtrl_LockedAtt[i]-pitch-0)*scale + angle0, i );
			
			uint8_t ref_chan = aux_cfg - 273;
			if( rc->connected && rc->raw_available_channels>ref_chan )
			{
				if( fabs(GimbalCtrl_LockedAtt[i])<200 && last_Channel_values[i]>-100 )
				{	//云台自动控制
					//通道变化大于阈值才调整云台
					if( fabs(rc->raw_data[ref_chan] - last_Channel_values[i]) > 10 )
					{
						float angle = (rc->raw_data[ref_chan]-50.0)*(60.0/50.0)*aux_param1 + 45+aux_param2; 							
						angle -= pitch;
						angle = constrain( angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0] );
						Aux_PWM_Out( (angle-0)*scale + angle0, i );
						last_Channel_values[i] = rc->raw_data[ref_chan];
						GimbalCtrl_LockedAtt[i] = 20000;
					}
				}
				else
				{	//云台手动控制
					float angle = (rc->raw_data[ref_chan]-50.0)*(60.0/50.0)*aux_param1 + 45+aux_param2; 
					angle -= pitch;
					angle = constrain( angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0] );
					Aux_PWM_Out( (angle-0)*scale + angle0, i );
					last_Channel_values[i] = rc->raw_data[ref_chan];
				}
			}
			else 
			{	//无遥控信号
				last_Channel_values[i] = -200;
				if( GimbalCtrl_LockedAtt[i] > 200 )
				{	//非自动控制且无遥控信号
					//锁定0角度
					float angle = 0;
					Aux_PWM_Out( (angle-pitch-0)*scale + angle0, i );
				}
			}
		}
		else if( aux_cfg>=97 && aux_cfg<=120 )
		{	//用遥控器对应通道进行舵机云台横滚控制(raw_data)
			double angleN45 = (aux_configs.Aux_StYTRolN45[0]-1000)*0.1;
			double angleP45 = (aux_configs.Aux_StYTRolP45[0]-1000)*0.1;
			double angle0 = (angleN45 + angleP45) / 2;
			double scale = (angleP45 - angle0)/45.0;
			Quaternion quat;
			get_Airframe_quat(&quat);
			double roll = rad2degree(quat.getRoll());
			if( GimbalCtrl_LockedAtt[i] < 200 )
				Aux_PWM_Out( (GimbalCtrl_LockedAtt[i]-roll-0)*scale + angleN45, i );
			
			uint8_t ref_chan = aux_cfg - 97;
			if( rc->connected && rc->raw_available_channels>ref_chan )
			{
				if( fabs(GimbalCtrl_LockedAtt[i])<200 && last_Channel_values[i]>-100 )
				{	//云台自动控制
					//通道变化大于阈值才调整云台
					if( fabs(rc->raw_data[ref_chan] - last_Channel_values[i]) > 10 )
					{
						float angle = (rc->raw_data[ref_chan]-50.0)*(45.0/50.0)*aux_param1 + aux_param2; 
						angle -= roll;
						angle = constrain( angle, aux_configs.Aux_YTRollMax[0] );
						Aux_PWM_Out( (angle-0)*scale + angle0, i );
						last_Channel_values[i] = rc->raw_data[ref_chan];
						GimbalCtrl_LockedAtt[i] = 20000;
					}
				}
				else
				{	//云台手动控制
					float angle = (rc->raw_data[ref_chan]-50.0)*(45.0/50.0)*aux_param1 + aux_param2; 
					angle -= roll;
					angle = constrain( angle, aux_configs.Aux_YTRollMax[0] );
					Aux_PWM_Out( (angle-0)*scale + angle0, i );
					last_Channel_values[i] = rc->raw_data[ref_chan];
				}
			}
			else
			{	//无遥控器信号
				if( GimbalCtrl_LockedAtt[i] > 200 )
				{	//非自动控制且无遥控信号
					//锁定0角度
					float angle = 0;
					Aux_PWM_Out( (angle-roll-0)*scale + angle0, i );
				}
			}
		}
		else if( aux_cfg>=121 && aux_cfg<=137 )
		{ // 数字云台控制，在线程中处理，请勿删除
		
		}
		else if( aux_cfg>=173 && aux_cfg<=196 )
		{	//用遥控器对应通道进行舵机云台俯仰往复控制(raw_data)
			double angle90 = (aux_configs.Aux_StYTPit90[0]-1000)*0.1;
			double angle0 = (aux_configs.Aux_StYTPit0[0]-1000)*0.1;
			double scale = (angle90 - angle0)/90.0;
			
			uint32_t* speedSign = &channelTemp1[i];
			float* speed = &channelTemp2[i];
			float* currentAngle = &channelTemp3[i];
			
			uint8_t ref_chan = aux_cfg - 173;
			if( rc->connected && rc->raw_available_channels>ref_chan )
			{
				*speed = rc->raw_data[ref_chan]*aux_param1 + aux_param2;
				if( *speed < 0 )
					*speed = 0;
			}
			
			if( *speedSign )
				*currentAngle += (*speed) * h;
			else
				*currentAngle -= (*speed) * h;
			if( *currentAngle >= 90 )
			{
				*currentAngle = 90;
				*speedSign = 0;
			}
			else if( *currentAngle <= 0 )
			{
				*currentAngle = 0;
				*speedSign = 1;
			}
			Aux_PWM_Out( (*currentAngle)*scale + angle0, i );
		}
		else if( aux_cfg>=197 && aux_cfg<=220 )
		{	//用遥控器对应通道进行舵机云台横滚往复控制(raw_data)
			double angleN45 = (aux_configs.Aux_StYTRolN45[0]-1000)*0.1;
			double angleP45 = (aux_configs.Aux_StYTRolP45[0]-1000)*0.1;
			double angle0 = (angleN45 + angleP45) / 2;
			double scale = (angleP45 - angle0)/45.0;
			
			uint32_t* speedSign = &channelTemp1[i];
			float* speed = &channelTemp2[i];
			float* currentAngle = &channelTemp3[i];
			
			uint8_t ref_chan = aux_cfg - 197;
			if( rc->connected && rc->raw_available_channels>ref_chan )
			{
				*speed = rc->raw_data[ref_chan]*aux_param1 + aux_param2;
				if( *speed < 0 )
					*speed = 0;
			}
			
			if( *speedSign )
				*currentAngle += (*speed) * h;
			else
				*currentAngle -= (*speed) * h;
			if( *currentAngle >= 45 )
			{
				*currentAngle = 45;
				*speedSign = 0;
			}
			else if( *currentAngle <= -45 )
			{
				*currentAngle = -45;
				*speedSign = 1;
			}
			
			Aux_PWM_Out( (*currentAngle)*scale + angle0, i );
		}
		
		else if( aux_cfg>=1001 && aux_cfg<=1016 )
		{	//映射虚拟摇杆通道
			Receiver jrc;
			getJoyStick(&jrc,0);
			
			if( GimbalCtrl_LockedAtt[i] < 100000 )
				Aux_PWM_Out( (GimbalCtrl_LockedAtt[i]-1000)*0.1, i );
			uint8_t ref_chan = aux_cfg - 1001;
			if( jrc.connected && jrc.raw_available_channels>ref_chan )
			{
				if( fabs(GimbalCtrl_LockedAtt[i])<100000 && last_Channel_values[i]>-100 )
				{	//Aux自动控制
					//通道变化大于阈值才调整Aux
					if( fabs(jrc.raw_data[ref_chan]-last_Channel_values[i])>10 )
					{
						Aux_PWM_Out( (jrc.raw_data[ref_chan]-50.0)*aux_param1+50+aux_param2*0.1, i );
						last_Channel_values[i] = jrc.raw_data[ref_chan];
						GimbalCtrl_LockedAtt[i] = 200000;
					}
				}
				else
				{	//Aux手动控制
					Aux_PWM_Out( (jrc.raw_data[ref_chan]-50.0)*aux_param1+50+aux_param2*0.1, i );
					last_Channel_values[i] = jrc.raw_data[ref_chan];
				}
			}
			else
				last_Channel_values[i] = -200;
		}
		else if( aux_cfg>=1025 && aux_cfg<=1048 )
		{	//用虚拟摇杆对应通道进行相机快门触发(raw_data)
			if( xSemaphoreTakeRecursive( CamMutex, 0 ) )
			{
				if( camPhotoAsyncStep == 0 )
				{
					Receiver jrc;
					getJoyStick(&jrc,0);

					uint8_t ref_chan = aux_cfg - 1025;
					if( jrc.connected && jrc.raw_available_channels>ref_chan )
					{
						if( last_Channel_values[i]>-100 )
						{
							if( fabs(jrc.raw_data[ref_chan] - last_Channel_values[i]) > 15 )
							{	//触发相机
								if( last_Channel_values[i] > -100 )							
								{
									AuxCamTakePhoto();					
								}
								last_Channel_values[i] = jrc.raw_data[ref_chan];
							}
							else
								Aux_PWM_Out( aux_configs.Aux_CamOffPwm[0]*0.1-100, i );
						}
						else
						{	//旧通道数据不可用
							Aux_PWM_Out( aux_configs.Aux_CamOffPwm[0]*0.1-100, i );
							last_Channel_values[i] = jrc.raw_data[ref_chan];
						}
					}
					else
					{	//无遥控器
						Aux_PWM_Out( aux_configs.Aux_CamOffPwm[0]*0.1-100, i );
						last_Channel_values[i] = -200;
					}
				}
				xSemaphoreGiveRecursive(CamMutex);
			}
		}
		else if( aux_cfg>=1049 && aux_cfg<=1072 )
		{	//用虚拟摇杆对应通道进行无刷云台俯仰控制(raw_data)
			Receiver jrc;
			getJoyStick(&jrc,0);
			
			double angle90 = (aux_configs.Aux_BsYTPit90[0]-1000)*0.1;
			double angle0 = (aux_configs.Aux_BsYTPit0[0]-1000)*0.1;
			double scale = (angle90 - angle0)/90.0;
			if( GimbalCtrl_LockedAtt[i] < 200 )
				Aux_PWM_Out( (GimbalCtrl_LockedAtt[i]-0)*scale + angle0, i );
			
			uint8_t ref_chan = aux_cfg - 1049;
			if( jrc.connected && jrc.raw_available_channels>ref_chan )
			{
				if( fabs(GimbalCtrl_LockedAtt[i]) < 200 && last_Channel_values[i]>-100 )
				{	//云台自动控制
					//通道变化大于阈值才调整云台
					if( fabs(jrc.raw_data[ref_chan] - last_Channel_values[i]) > 10 )
					{
						float angle = (jrc.raw_data[ref_chan]-50.0)*(60.0/50.0)*aux_param1 + 45+aux_param2; 
						angle = constrain( angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0] );
						Aux_PWM_Out( (angle-0)*scale + angle0, i );
						last_Channel_values[i] = jrc.raw_data[ref_chan];
						GimbalCtrl_LockedAtt[i] = 200000;
					}
				}
				else
				{	//云台手动控制
					float angle = (jrc.raw_data[ref_chan]-50.0)*(60.0/50.0)*aux_param1 + 45+aux_param2; 
					angle = constrain( angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0] );
					Aux_PWM_Out( (angle-0)*scale + angle0, i );
					last_Channel_values[i] = jrc.raw_data[ref_chan];
				}
			}
			else
			{	//无遥控信号
				last_Channel_values[i] = -200;
				if( GimbalCtrl_LockedAtt[i] > 200 )
				{	//非自动控制且无遥控信叿
					//锁定0角度
					float angle = 0;
					Aux_PWM_Out( (angle-0)*scale + angle0, i );
				}
			}
		}
		else if( aux_cfg>=1073 && aux_cfg<=1096 )
		{	//用虚拟摇杆对应通道进行舵机云台俯仰控制(raw_data)
			Receiver jrc;
			getJoyStick(&jrc,0);

			double angle90 = (aux_configs.Aux_StYTPit90[0]-1000)*0.1;
			double angle0 = (aux_configs.Aux_StYTPit0[0]-1000)*0.1;
			double scale = (angle90 - angle0)/90.0;
			Quaternion quat;
			get_Airframe_quat(&quat);
			double pitch = rad2degree(quat.getPitch());
			if( GimbalCtrl_LockedAtt[i] < 200 )
				Aux_PWM_Out( (GimbalCtrl_LockedAtt[i]-pitch-0)*scale + angle0, i );
			
			uint8_t ref_chan = aux_cfg - 1073;
			if( jrc.connected && jrc.raw_available_channels>ref_chan )
			{
				if( fabs(GimbalCtrl_LockedAtt[i])<200 && last_Channel_values[i]>-100 )
				{	//云台自动控制
					//通道变化大于阈值才调整云台
					if( fabs(jrc.raw_data[ref_chan] - last_Channel_values[i]) > 10 )
					{
						float angle = (jrc.raw_data[ref_chan]-50.0)*(60.0/50.0)*aux_param1 + 45+aux_param2; 							
						angle -= pitch;
						angle = constrain( angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0] );
						Aux_PWM_Out( (angle-0)*scale + angle0, i );
						last_Channel_values[i] = jrc.raw_data[ref_chan];
						GimbalCtrl_LockedAtt[i] = 20000;
					}
				}
				else
				{	//云台手动控制
					float angle = (jrc.raw_data[ref_chan]-50.0)*(60.0/50.0)*aux_param1 + 45+aux_param2; 
					angle -= pitch;
					angle = constrain( angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0] );
					Aux_PWM_Out( (angle-0)*scale + angle0, i );
					last_Channel_values[i] = jrc.raw_data[ref_chan];
				}
			}
			else 
			{	//无遥控信号
				last_Channel_values[i] = -200;
				if( GimbalCtrl_LockedAtt[i] > 200 )
				{	//非自动控制且无遥控信叿
					//锁定0角度
					float angle = 0;
					Aux_PWM_Out( (angle-pitch-0)*scale + angle0, i );
				}
			}
		}
		else if( aux_cfg>=1097 && aux_cfg<=1120 )
		{	//用虚拟摇杆对应通道进行舵机云台横滚控制(raw_data)
			Receiver jrc;
			getJoyStick(&jrc,0);
			
			double angleN45 = (aux_configs.Aux_StYTRolN45[0]-1000)*0.1;
			double angleP45 = (aux_configs.Aux_StYTRolP45[0]-1000)*0.1;
			double angle0 = (angleN45 + angleP45) / 2;
			double scale = (angleP45 - angle0)/45.0;
			Quaternion quat;
			get_Airframe_quat(&quat);
			double roll = rad2degree(quat.getRoll());
			if( GimbalCtrl_LockedAtt[i] < 200 )
				Aux_PWM_Out( (GimbalCtrl_LockedAtt[i]-roll-0)*scale + angleN45, i );
			
			uint8_t ref_chan = aux_cfg - 97;
			if( jrc.connected && jrc.raw_available_channels>ref_chan )
			{
				if( fabs(GimbalCtrl_LockedAtt[i])<200 && last_Channel_values[i]>-100 )
				{	//云台自动控制
					//通道变化大于阈值才调整云台
					if( fabs(jrc.raw_data[ref_chan] - last_Channel_values[i]) > 10 )
					{
						float angle = (jrc.raw_data[ref_chan]-50.0)*(45.0/50.0)*aux_param1 + aux_param2; 
						angle -= roll;
						angle = constrain( angle, aux_configs.Aux_YTRollMax[0] );
						Aux_PWM_Out( (angle-0)*scale + angle0, i );
						last_Channel_values[i] = jrc.raw_data[ref_chan];
						GimbalCtrl_LockedAtt[i] = 20000;
					}
				}
				else
				{	//云台手动控制
					float angle = (jrc.raw_data[ref_chan]-50.0)*(45.0/50.0)*aux_param1 + aux_param2; 
					angle -= roll;
					angle = constrain( angle, aux_configs.Aux_YTRollMax[0] );
					Aux_PWM_Out( (angle-0)*scale + angle0, i );
					last_Channel_values[i] = jrc.raw_data[ref_chan];
				}
			}
			else
			{	//无遥控器信号
				if( GimbalCtrl_LockedAtt[i] > 200 )
				{	//非自动控制且无遥控信叿
					//锁定0角度
					float angle = 0;
					Aux_PWM_Out( (angle-roll-0)*scale + angle0, i );
				}
			}
		}
		
	}
	
//	/*数字云台接口*/
//		for( uint8_t i = 0; i < 8; ++i )
//		{	//横滚 俯仰 偏航 变??
//			//模式 聚焦 拍照录像 归位
//			uint16_t aux_cfg = ((uint16_t*)&aux_configs.DgYTRFunc)[i*4];
//			float aux_param1 = ((float*)&aux_configs.DgYTRParam1)[i*2];
//			float aux_param2 = ((float*)&aux_configs.DgYTRParam2)[i*2];
//			
//			if( aux_cfg>=1 && aux_cfg<=16 )
//			{	//遥控器通道控制角度
//				if( rc->connected )
//				{
//					uint8_t ref_chan = aux_cfg - 1;
//					if( rc->raw_available_channels > ref_chan )
//					{
//						float value;
//						if( i == DgYTZoom )	//缩放通道特殊处理
//							value = rc->raw_data[ref_chan]*0.3*aux_param1+aux_param2*0.1;
//						else
//							value = (rc->raw_data[ref_chan]-50.0)*aux_param1+aux_param2*0.1;
//							
//						set_DgYTCtrl( i, 
//							0, //0-角度模式 1-角速度模式
//							value,
//							0.1 );
//					}
//				}
//			}
//			else if( aux_cfg>=25 && aux_cfg<=48 )
//			{	//遥控器通道控制角度
//				if( rc->connected )
//				{
//					uint8_t ref_chan = aux_cfg - 25;
//					if( rc->raw_available_channels > ref_chan )
//					{
//						set_DgYTCtrl( i, 
//							1, //0-角度模式 1-角速度模式
//							(rc->raw_data[ref_chan]-50.0)*aux_param1+aux_param2*0.1,
//							0.1 );
//					}
//				}
//			}
//		}
//	/*数字云台接口*/
}


bool AuxGimbalSetAngle( double angle )
{
	AuxFuncsConfig aux_configs;
	ReadParamGroup( "AuxCfg", (uint64_t*)&aux_configs, 0 );
	
	uint8_t MainMotorsCount = get_MainMotorCount();
	uint8_t AuxChannelsCount = get_AuxChannelCount();
	for( uint8_t i = MainMotorsCount; i < MainMotorsCount+AuxChannelsCount; ++i )
	{
		uint8_t aux_cfg = ((uint8_t*)&aux_configs)[i*8];
		if( aux_cfg>=49 && aux_cfg<=72 )
		{	//无刷云台俯仰??
			angle = constrain( angle, (double)aux_configs.Aux_YTPitMin[0], (double)aux_configs.Aux_YTPitMax[0] );
			GimbalCtrl_LockedAtt[i] = angle;
		}
		else if( aux_cfg>=73 && aux_cfg<=96 )
		{	//无刷云台俯仰??
			angle = constrain( angle, (double)aux_configs.Aux_YTPitMin[0], (double)aux_configs.Aux_YTPitMax[0] );
			GimbalCtrl_LockedAtt[i] = angle;
		}
		else if( aux_cfg>=97 && aux_cfg<=120 )
		{	//无刷云台俯仰??
			angle = constrain( angle, (double)aux_configs.Aux_YTRollMax[0] );
			GimbalCtrl_LockedAtt[i] = angle;
		}
	}
	return true;
}

void init_AuxFuncs()
{
	// 注册消息队列
	TaskQueueRegister(CAMERA_FEEDBACK_QUEUE_ID,30);
	
	//注册通信参数	
	AuxFuncsConfig initial_cfg;
	initial_cfg.Aux1Func[0] = 0;
	initial_cfg.Aux2Func[0] = 0;
	initial_cfg.Aux3Func[0] = 0;
	initial_cfg.Aux4Func[0] = 0;
	initial_cfg.Aux5Func[0] = 0;
	initial_cfg.Aux6Func[0] = 0;
	initial_cfg.Aux7Func[0] = 0;
	initial_cfg.Aux8Func[0] = 0;
/* 遥控通道1-16对应的数字云台功能 */
	initial_cfg.RcYT1Func[0] = 0; 
	initial_cfg.RcYT2Func[0] = 0; 
	initial_cfg.RcYT3Func[0] = 0; 
	initial_cfg.RcYT4Func[0] = 0; 
	initial_cfg.RcYT5Func[0] = 0;
	initial_cfg.RcYT6Func[0] = 0;
	initial_cfg.RcYT7Func[0] = 0;
	initial_cfg.RcYT8Func[0] = 0;
	initial_cfg.RcYT9Func[0] = 0;
	initial_cfg.RcYT10Func[0] = 0;
	initial_cfg.RcYT11Func[0] = 0;
	initial_cfg.RcYT12Func[0] = 0;
	initial_cfg.RcYT13Func[0] = 0;
	initial_cfg.RcYT14Func[0] = 0;
	initial_cfg.RcYT15Func[0] = 0;
	initial_cfg.RcYT16Func[0] = 0; 

	initial_cfg.Aux1Param1[0] = 1;
	initial_cfg.Aux2Param1[0] = 1;
	initial_cfg.Aux3Param1[0] = 1;
	initial_cfg.Aux4Param1[0] = 1;
	initial_cfg.Aux5Param1[0] = 1;
	initial_cfg.Aux6Param1[0] = 1;
	initial_cfg.Aux7Param1[0] = 1;
	initial_cfg.Aux8Param1[0] = 1;
/* 遥控通道1-16对应的数字云台参数1 */
	initial_cfg.RcYT1Param1[0] = 1;
	initial_cfg.RcYT2Param1[0] = 1;
	initial_cfg.RcYT3Param1[0] = 1;
	initial_cfg.RcYT4Param1[0] = 1;
	initial_cfg.RcYT5Param1[0] = 1;
	initial_cfg.RcYT6Param1[0] = 1;
	initial_cfg.RcYT7Param1[0] = 1;
	initial_cfg.RcYT8Param1[0] = 1;
	initial_cfg.RcYT9Param1[0] = 1;
	initial_cfg.RcYT10Param1[0] = 1;
	initial_cfg.RcYT11Param1[0] = 1;
	initial_cfg.RcYT12Param1[0] = 1;
	initial_cfg.RcYT13Param1[0] = 1;
	initial_cfg.RcYT14Param1[0] = 1;
	initial_cfg.RcYT15Param1[0] = 1;
	initial_cfg.RcYT16Param1[0] = 1;
	
	initial_cfg.Aux1Param2[0] = 0;
	initial_cfg.Aux2Param2[0] = 0;
	initial_cfg.Aux3Param2[0] = 0;
	initial_cfg.Aux4Param2[0] = 0;
	initial_cfg.Aux5Param2[0] = 0;
	initial_cfg.Aux6Param2[0] = 0;
	initial_cfg.Aux7Param2[0] = 0;
	initial_cfg.Aux8Param2[0] = 0;

/* 遥控通道1-16对应的数字云台参数2 */
	initial_cfg.RcYT1Param2[0] = 1;
	initial_cfg.RcYT2Param2[0] = 1;
	initial_cfg.RcYT3Param2[0] = 1;
	initial_cfg.RcYT4Param2[0] = 1;
	initial_cfg.RcYT5Param2[0] = 1;
	initial_cfg.RcYT6Param2[0] = 1;
	initial_cfg.RcYT7Param2[0] = 1;
	initial_cfg.RcYT8Param2[0] = 1;
	initial_cfg.RcYT9Param2[0] = 1;
	initial_cfg.RcYT10Param2[0] = 1;
	initial_cfg.RcYT11Param2[0] = 1;
	initial_cfg.RcYT12Param2[0] = 1;
	initial_cfg.RcYT13Param2[0] = 1;
	initial_cfg.RcYT14Param2[0] = 1;
	initial_cfg.RcYT15Param2[0] = 1;
	initial_cfg.RcYT16Param2[0] = 1;

  initial_cfg.Aux_YTSurveyPic[0] = 0;
	initial_cfg.Aux_CamOnPwm[0] = 2000;
	initial_cfg.Aux_CamOffPwm[0] = 1000;
	initial_cfg.Aux_CamShTime[0] = 0.1;
	initial_cfg.Aux_CamTrigEna[0] = 0;
	initial_cfg.Aux_BsYTPit0[0] = 1000;
	initial_cfg.Aux_BsYTPit90[0] = 2000;
	initial_cfg.Aux_StYTPit0[0] = 1250;
	initial_cfg.Aux_StYTPit90[0] = 1750;
	initial_cfg.Aux_StYT2Pit0[0] = 1250;
	initial_cfg.Aux_StYT2Pit90[0] = 1750;
	initial_cfg.Aux_StYTRolN45[0] = 1000;
	initial_cfg.Aux_StYTRolP45[0] = 2000;
	initial_cfg.Aux_YTPitMin[0] = -20;
	initial_cfg.Aux_YTPitMax[0] = 120;
	initial_cfg.Aux_YTRollMax[0] = 45;
	initial_cfg.Aux_Pump1Min[0] = 1000;
	initial_cfg.Aux_Pump1St[0] = 1100;
	initial_cfg.Aux_Pump1Max[0] = 1950;
	initial_cfg.Aux_Pump1Sp[0] = 0.15;
	MAV_PARAM_TYPE param_types[] = {
		MAV_PARAM_TYPE_UINT16 , // Aux1Func
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 , // Aux8Func
		MAV_PARAM_TYPE_UINT16 ,// RcYT1Func
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,// RcYT16Func	
		
		MAV_PARAM_TYPE_REAL32 ,// Aux1Param1
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,// Aux8Param1
		
		MAV_PARAM_TYPE_REAL32 ,// RcYT1Param1
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,// RcYT16Param1
		
		MAV_PARAM_TYPE_REAL32 ,// Aux1Param2
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,// Aux8Param2
		
		MAV_PARAM_TYPE_REAL32 ,// RcYT1Param2
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,// RcYT16Param2		
		
		MAV_PARAM_TYPE_UINT8,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
	};
	SName param_names[] = {
		"Aux_1Func" ,
		"Aux_2Func" ,
		"Aux_3Func" ,
		"Aux_4Func" ,
		"Aux_5Func" ,
		"Aux_6Func" ,
		"Aux_7Func" ,
		"Aux_8Func" ,
		"Aux_RcYT1Func" ,
		"Aux_RcYT2Func" ,
		"Aux_RcYT3Func" ,
		"Aux_RcYT4Func" ,
		"Aux_RcYT5Func" ,
		"Aux_RcYT6Func" ,
		"Aux_RcYT7Func" ,
		"Aux_RcYT8Func" ,
		"Aux_RcYT9Func" ,
		"Aux_RcYT10Func" ,
		"Aux_RcYT11Func" ,
		"Aux_RcYT12Func" ,
		"Aux_RcYT13Func" ,
		"Aux_RcYT14Func" ,
		"Aux_RcYT15Func" ,
		"Aux_RcYT16Func" ,		
		"Aux_1Param1" ,
		"Aux_2Param1" ,
		"Aux_3Param1" ,
		"Aux_4Param1" ,
		"Aux_5Param1" ,
		"Aux_6Param1" ,
		"Aux_7Param1" ,
		"Aux_8Param1" ,
		"Aux_RcYT1Param1" ,
		"Aux_RcYT2Param1" ,
		"Aux_RcYT3Param1" ,
		"Aux_RcYT4Param1" ,
		"Aux_RcYT5Param1" ,
		"Aux_RcYT6Param1" ,
		"Aux_RcYT7Param1" ,
		"Aux_RcYT8Param1" ,
		"Aux_RcYT9Param1" ,
		"Aux_RcYT10Param1" ,
		"Aux_RcYT11Param1" ,
		"Aux_RcYT12Param1" ,
		"Aux_RcYT13Param1" ,
		"Aux_RcYT14Param1" ,
		"Aux_RcYT15Param1" ,
		"Aux_RcYT16Param1" ,		
		"Aux_1Param2" ,
		"Aux_2Param2" ,
		"Aux_3Param2" ,
		"Aux_4Param2" ,
		"Aux_5Param2" ,
		"Aux_6Param2" ,
		"Aux_7Param2" ,
		"Aux_8Param2" ,
		"Aux_RcYT1Param2" ,
		"Aux_RcYT2Param2" ,
		"Aux_RcYT3Param2" ,
		"Aux_RcYT4Param2" ,
		"Aux_RcYT5Param2" ,
		"Aux_RcYT6Param2" ,
		"Aux_RcYT7Param2" ,
		"Aux_RcYT8Param2" ,
		"Aux_RcYT9Param2" ,
		"Aux_RcYT10Param2" ,
		"Aux_RcYT11Param2" ,
		"Aux_RcYT12Param2" ,
		"Aux_RcYT13Param2" ,
		"Aux_RcYT14Param2" ,
		"Aux_RcYT15Param2" ,
		"Aux_RcYT16Param2" ,	
		
		"Aux_YTSurveyPic" , //数字云台是否需在测绘任务过程中拍照
		"Aux_CamOnPwm" ,    //拍照开
		"Aux_CamOffPwm" ,   //拍照关
		"Aux_CamShTime" ,   //拍照持续时间
		"Aux_CamTrigEna" ,  //热靴触发使能
		"Aux_BsYTPit0" ,
		"Aux_BsYTPit90" ,
		"Aux_StYTPit0" ,
		"Aux_StYTPit90" ,
		"Aux_StYT2Pit0" ,
		"Aux_StYT2Pit90" ,
		"Aux_StYTRolN45" ,
		"Aux_StYTRolP45" ,
		"Aux_StPitMin" ,
		"Aux_StPitMax" ,
		"Aux_StRolMax" ,
		"Aux_Pump1Min" ,
		"Aux_Pump1St" ,
		"Aux_Pump1Max" ,
		"Aux_Pump1Sp" ,
	};
	ParamGroupRegister( "AuxCfg", 1, sizeof(initial_cfg)/8, param_types, param_names, (uint64_t*)&initial_cfg );
}