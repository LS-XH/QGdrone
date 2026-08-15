#include "drv_mosaxicX5.hpp"
#include "drv_RTK_DAO_Base.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "Parameters.hpp"
#include "Commulink.hpp"
#include "StorageSystem.hpp"
#include "drv_CRC.hpp"

#define INIT_LENGHT 80

struct DriverInfo
{
	uint32_t param;
	Port port;
};

struct GpsConfig
{
	uint8_t GNSS_Mode[8];
	float delay[2];
};

enum GPS_Scan_Operation
{
	//在指定波特率下发送初始化信息
	GPS_Scan_Baud9600 = 9600 ,
	GPS_Scan_Baud38400 = 38400 ,
	GPS_Scan_Baud460800 = 460800 ,
	GPS_Scan_Baud115200 = 115200 ,
	GPS_Scan_Baud57600 = 57600 ,
	
	//检查是否设置成功
	GPS_Check_Baud ,
	//在当前波特率下再次发送配置
	GPS_ResendConfig ,
	//GPS已检测存在
	GPS_Present ,
};

struct GPS_State_Machine
{
	uint32_t frame_datas_ind = 0;
	uint16_t frame_datas_length;
	uint8_t read_state = 0;	
	uint16_t block_num=0;
	uint16_t block_version=0;
	uint16_t crc=0;
};

static inline void ResetRxStateMachine( GPS_State_Machine* state_machine )
{
	state_machine->read_state=state_machine->frame_datas_ind=0;
}

static inline uint16_t GPS_ParseByte( GPS_State_Machine* state_machine, uint8_t* frame_datas, uint8_t r_data )
{
	frame_datas[ state_machine->frame_datas_ind++ ] = r_data;
	switch( state_machine->read_state )
	{
		case 0:	//找包头
		{				
			if( state_machine->frame_datas_ind == 1 )
			{
				if( r_data != '$' )
					state_machine->frame_datas_ind = 0;
			}
			else
			{
				if( r_data == '@' )	//header found
				{
					state_machine->read_state = 1;
				}		
				else
					state_machine->frame_datas_ind = 0;
			}	
			break;
		}
		case 1:	//The CRC field
		{	
			if( state_machine->frame_datas_ind == 4 )
			{			
				state_machine->crc = (*(unsigned short*)&frame_datas[2]);
				state_machine->read_state = 2;
			}
			break;
		}
		case 2:	//读Class ID和包长度
		{
			if( state_machine->frame_datas_ind == 8 )
			{
				state_machine->block_num = (*(unsigned short*)&frame_datas[4]) & 0x1FFF;
				state_machine->block_version = (*(unsigned short*)&frame_datas[4]) & 0xE000;
			
				state_machine->frame_datas_length = (*(unsigned short*)&frame_datas[6]);
				if( state_machine->frame_datas_length > 8 && state_machine->frame_datas_length < 1000 )
					state_machine->read_state = 3;						
				else
					ResetRxStateMachine(state_machine);
			}
			break;
		}
		case 3:	//读包内容,校验
		{	
			if( state_machine->frame_datas_ind == state_machine->frame_datas_length )
			{
 				uint32_t res =0;
				CRC_Calc( cg_CRC_CONFIG_CRC16_XMODEM, &frame_datas[4], state_machine->frame_datas_length-4, &res);
				ResetRxStateMachine(state_machine);
				if( res == state_machine->crc )
					return state_machine->frame_datas_length;	
				else
					return 0;
			}
			break;
		}
	}
	return 0;
}

static bool send_init_msg(DriverInfo& driver_info)
{
	int len=0;
	char mosaxicClr[100];
	//等待控制台准备好
	driver_info.port.reset_rx(0.1);
	len = sprintf( mosaxicClr, "\r\n ");	
	driver_info.port.write( (uint8_t*)&mosaxicClr[0], len, portMAX_DELAY, portMAX_DELAY );	
	driver_info.port.wait_sent(1);
	len = 0;
	const char CMDres[] = "COM";
	const uint8_t CMDres_length = sizeof(CMDres)-1;
	TIME chk_TIME = TIME::now();
	char COMport = -1;
	while( chk_TIME.get_pass_time() < 1.5 )
	{
		uint16_t rc_len = driver_info.port.read( (uint8_t*)&mosaxicClr[0], 1, 1, 0.5 );
		if( rc_len == 0 )
			return false;
		if( len < CMDres_length )
		{
			if( mosaxicClr[0] == CMDres[len] )
				++len;
			else if(len>0)
				len = 0;
		}
		else
		{	//匹配成功,接收端口号后退出
			if( len == 3 )
			{	//获取COM号
				if( mosaxicClr[0]<'0' || mosaxicClr[0]>'9' )
					return false;
				COMport = mosaxicClr[0] - '0';
			}
			if( len == 4 )
			{
				if( mosaxicClr[0] != '>' )
					return false;
			}
			if( ++len >= CMDres_length+2 )
				break;
		}
	}
	if( len != CMDres_length+2 )
		return false;
	
	for( uint8_t i=0; i < 12; i++ )
	{
		if(i==0)       // 卫星和频点
			len = sprintf( mosaxicClr,  "snu, all, all\r\n");	
		else if (i==1)// 设置PVTGeodetic+ExtEventPVTGeodetic+PostProcess
			len = sprintf( mosaxicClr,  "ssgp, Group1, PVTGeodetic+ExtEventPVTGeodetic+AuxAntPositions\r\n");
		else if (i==2)// 频率, 10HZ
			len = sprintf( mosaxicClr,  "sso, Stream1, COM%d, Group1, msec100\r\n", COMport );
		else if (i==3)// UAV模式
			len = sprintf( mosaxicClr,  "srd, Moderate, UAV\r\n");			
		else if (i==4)// 截止高度角
			len = sprintf( mosaxicClr,  "sem, Tracking+PVT, 10\r\n");	
		else if (i==5)// 设为移动站模式，限单点+DGNSS+RTK
			len = sprintf( mosaxicClr,  "spm, Rover, StandAlone+DGPS+RTK\r\n");			
		else if (i==6)// 跟踪 GPS、Glonass、Galileo、北斗所有卫星
			len = sprintf( mosaxicClr,  "sst, GPS+Glonass+Galileo+Beidou\r\n");
		else if (i==7)// 跟踪四星座的所有频点
			len = sprintf( mosaxicClr,  "snt, GPS+Glonass+Galileo+Beidou\r\n");		
		else if (i==8)// PVT 解算使用四星座所有卫星
			len = sprintf( mosaxicClr,  "ssu, GPS+Glonass+Galileo+Beidou\r\n");
		else if (i==9)// 开启窄带干扰抑制
			len = sprintf( mosaxicClr, "snf, all, auto\r\n");
		else if (i==10)// 开启宽带干扰抑制
			len = sprintf( mosaxicClr,  "swbi, on\r\n");
		else if (i==11)// 设置com1波特率为460800,8位数据位,无校验,一位停止位,无硬件流控制		
			len = sprintf( mosaxicClr,  "scs, COM%d, baud460800, bits8, No, bit1, none\r\n", COMport );
	
		driver_info.port.write( (uint8_t*)&mosaxicClr[0], len, 2, 2 );	
		driver_info.port.wait_sent(1);
		os_delay(0.55);
	}	
	return true;
}

static void RTK_Server(void* pvParameters)
{	
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	//GPS识别状态
	GPS_Scan_Operation current_GPS_Operation = GPS_Scan_Baud115200;
	//数据读取状态机
	Aligned_DMABuf uint8_t frame_datas[1200];
	//上次更新时间
	TIME last_update_time;
	
	//等待初始化完成
	while( getInitializationCompleted() == false )
		os_delay(0.1);	
	
	//注册Rtk端口
	RtkPort rtk_port;
	rtk_port.ena = false;
	rtk_port.write = driver_info.port.write;
	rtk_port.lock = driver_info.port.lock;
	rtk_port.unlock = driver_info.port.unlock;
	int8_t rtk_port_ind = RtkPortRegister(rtk_port);	
	
	bool rtc_updated = false;
	
	//读取是否需要记录PPK
	bool record_ppk = false;
	uint8_t log_ppk[8];
	if( ReadParam( "SDLog_PPK", 0, 0, (uint64_t*)log_ppk, 0 ) == PR_OK )
	{
		if( log_ppk[0] == 3 )
			record_ppk = true;
	}
	
GPS_CheckBaud:
//	//os_delay(15.0);
//	while(1)
//	{
//		//更改指定波特率
//		driver_info.port.SetBaudRate( current_GPS_Operation, 3, 0.1 );
//		//切换波特率
//		switch(current_GPS_Operation)
//		{
//			case GPS_Scan_Baud9600:
//				current_GPS_Operation = GPS_Scan_Baud57600;
//				break;
//			case GPS_Scan_Baud57600:
//				current_GPS_Operation = GPS_Scan_Baud38400;
//				break;
//			case GPS_Scan_Baud38400:
//				current_GPS_Operation = GPS_Scan_Baud460800;
//				break;
//			case GPS_Scan_Baud460800:
//				current_GPS_Operation = GPS_Scan_Baud115200;
//				break;
//			default:
//			case GPS_Scan_Baud115200:
//				current_GPS_Operation = GPS_Scan_Baud9600;
//				break;
//		}
//		//发送配置
//    if( send_init_msg(driver_info) == false )
//			continue;

//		debug_test[18] = 1;
		//更改波特率
		driver_info.port.SetBaudRate( 460800, 1, 0.1 );		
//		//清空接收缓冲区准备接收数据
//		driver_info.port.reset_rx(0.1);
//		GPS_State_Machine gps_state;
//		ResetRxStateMachine(&gps_state);
//		TIME RxChkStartTime = TIME::now();
//		debug_test[18] = 2;
//		while( RxChkStartTime.get_pass_time() < 5 )
//		{
//			uint8_t r_data;
//			if( driver_info.port.read( &r_data, 1, 0.5, 0.1 ) )
//			{
//				uint16_t pack_length = GPS_ParseByte( &gps_state, frame_datas, r_data );
//				if( pack_length )
//				{
//					if( gps_state.block_num == 4007 || gps_state.block_num == 4038 )
//					{	//已识别到PVT包
//						//跳转到gps接收程序
//						goto GPS_Present;
//					}
//				}
//			}
//		}		
//	}	
//	
//GPS_Present:
//	debug_test[18] = 3;
//	//重发配置
//	send_init_msg(driver_info);
//	debug_test[18] = 4;
		
	uint32_t sensor_key = 0;
	uint32_t dao_key = 0;
	GpsDAOConfig gps_cfg;
	if( ReadParamGroup( "GPSDAOCfg", (uint64_t*)&gps_cfg, 0 ) == PR_OK )
	{	
		//注册传感器
		sensor_key = PositionSensorRegister( default_rtk_sensor_index , \
																					"RTK_MosaxicX5" ,\
																					Position_Sensor_Type_GlobalPositioning , \
																					Position_Sensor_DataType_sv_xy , \
																					Position_Sensor_frame_ENU , \
																					gps_cfg.delay[0] , //延时
																					30 , //xy信任度
																					30 //z信任度
																				);	
		
		//注册侧向传感器
		dao_key = DAOSensorRegister( 0, "DRTK", vector3<double>(gps_cfg.DRTK_VecX[0],gps_cfg.DRTK_VecY[0],gps_cfg.DRTK_VecZ[0]), false, gps_cfg.delay[0] );
	}
	else
	{	//注册传感器
		sensor_key = PositionSensorRegister( default_rtk_sensor_index , \
																					"RTK_MosaxicX5" ,\
																					Position_Sensor_Type_GlobalPositioning , \
																					Position_Sensor_DataType_sv_xy , \
																					Position_Sensor_frame_ENU , \
																					0.1 , //延时
																					30 , //xy信任度
																					30 //z信任度
																				);
		gps_cfg.DRTK_VecX[0] = gps_cfg.DRTK_VecY[0] = gps_cfg.DRTK_VecZ[0] = 0;
	}
	//开启Rtk注入
	RtkPort_setEna( rtk_port_ind, true );
	
	//gps状态
	bool gps_available = false;
	bool z_available=false;
	double alt_offset = -1000000;
	TIME GPS_stable_start_time(false);
	double gps_alt;
	TIME gps_update_TIME;
	
	//清空接收缓冲区准备接收数据
	driver_info.port.reset_rx(0.1);
	GPS_State_Machine gps_state;
	ResetRxStateMachine(&gps_state);
	frame_datas[0] = frame_datas[1] = 0;
	last_update_time = TIME::now();
	
	//附加数据
	double addition_inf[8] = {0};
	
	while(1)
	{
		uint8_t r_data;
		if( driver_info.port.read( &r_data, 1, 2, 0.1 ) )
		{
			uint16_t pack_length = GPS_ParseByte( &gps_state, frame_datas, r_data );
			if( pack_length )
			{
				if( gps_state.block_num == 4007 )
				{ // PVTGeodetic
					//记录
					if(record_ppk)
						SDLog_Ubx( (const char*)frame_datas, 100 );
					last_update_time = TIME::now();
					struct UBX_NAV_PVT_Pack
					{	
						uint16_t crc;
						uint16_t ID;	
						uint16_t lenght;	
						uint32_t TOW;
						uint16_t WNC;
						uint8_t mode;
						uint8_t error;
						double lat;
						double lon;
						double height;
						float Undulation;
						float VelN;
						float VelE;
						float VelU;
						float cog;
						double RxClkBias;
						float RxClkDrift;
						uint8_t	TimeSystem;
						uint8_t Dataum;
						uint8_t numSV;
						uint8_t WACorrInfo;
						uint16_t ReferenceID;
						uint16_t MeanCorrAge;
						uint32_t SignalInfo;
						uint8_t AlertFlag;
						uint8_t NrBases;
						uint16_t PPPInfo;
						uint16_t Latency; 
						uint16_t hAcc;
						uint16_t vAcc;
						uint8_t Misc;
					  uint8_t Padding;
					}__attribute__((packed));
          UBX_NAV_PVT_Pack* pack = (UBX_NAV_PVT_Pack*)&frame_datas[2];				
					
					uint8_t gps_fix = 0;
					uint8_t mode = pack->mode & 0x0f;					
					if( mode == 0 && pack->error!=0)
						gps_fix = 1;
					else if( mode==1 || mode==2 )
					{
						bool fix2D = pack->mode & (1<<7);
						if( fix2D )
							gps_fix = 2;
						else
							gps_fix = 3;
					}
					else if( mode == 4 )//RTK with fixed ambiguities
						gps_fix = 6;
					else if( mode == 5 )//RTK with float ambiguities
						gps_fix = 5;	
					else
						gps_fix= mode;					

					pack->hAcc*=0.7;
					pack->vAcc*=0.7;
					if( ( mode!=0 ) && (pack->error==0) && (pack->numSV >= 5) )
					{
						if( gps_available == false )
						{
							if( pack->hAcc<250 && pack->lat>-10000 && pack->lon>-10000 )
							{
								if( GPS_stable_start_time.is_valid() == false )
									GPS_stable_start_time = TIME::now();
								else if( GPS_stable_start_time.get_pass_time() > 3.0f )
								{
									gps_available = true;
									GPS_stable_start_time.set_invalid();
								}
							}
							else
								GPS_stable_start_time.set_invalid();
						}
						else
						{
							if( pack->hAcc>350 || pack->lat<-10000 || pack->lon<-10000 )
								gps_available = z_available = false;
						}
					}
					else
					{
						gps_available = z_available = false;
						GPS_stable_start_time.set_invalid();
					}	
					addition_inf[0] = pack->numSV;
					addition_inf[1] = gps_fix;
					addition_inf[4] = pack->hAcc;
					addition_inf[5] = pack->vAcc;
					addition_inf[6] = 0;
					if( z_available )
					{
						if( pack->vAcc>450 || pack->height<-10000 )
							z_available = false;
					}
					else
					{
						if( pack->vAcc<250 && pack->height>-10000 )
						{
							gps_alt = pack->height * 1e2;
							z_available = true;
						}
					}
					double t = gps_update_TIME.get_pass_time_st();
					if( t > 1 )
						t = 1;	

					vector3<double> velocity;
					velocity.y = pack->VelN * 100;	//North
					velocity.x = pack->VelE * 100;	//East
					velocity.z = pack->VelU * 100;	//Up
					gps_alt += velocity.z * t;
					if( pack->vAcc < 600 )
					{	//高精度高度位置结果
						//使用绝对高度
						double r_height = pack->height*100;
						if( alt_offset <= -100000 )
							alt_offset = r_height - gps_alt;
						gps_alt += 0.5*t * ( r_height - gps_alt - alt_offset );
					}
					else
						alt_offset = -1000000;
					
					vector3<double> position_Global;
					position_Global.x = rad2degree( pack->lat );
					position_Global.y = rad2degree( pack->lon) ;
					position_Global.z = gps_alt;	

					if( z_available )
						PositionSensorChangeDataType( default_rtk_sensor_index,sensor_key, Position_Sensor_DataType_sv_xyz );
					else
						PositionSensorChangeDataType( default_rtk_sensor_index,sensor_key, Position_Sensor_DataType_sv_xy );
					
					//信任度
					double xy_trustD = pack->hAcc;
					double z_trustD = pack->vAcc;
					if( pack->lat>-10 && pack->lat<10 && pack->lat>-5 && pack->lat<5 && pack->VelE>-1e+10 && pack->VelN>-1e+10 && pack->VelU>-1e+10 )
					{
						PositionSensorUpdatePositionGlobalVel( default_rtk_sensor_index,sensor_key, position_Global, velocity, 
							gps_available, //available
							-1, //delay
							xy_trustD, 
							z_trustD, 
							addition_inf,
							xy_trustD,	//xy LTtrust
							z_trustD	//z LTtrust
						);
					}
				}
				else if( gps_state.block_num == 5942 )
				{ // AuxAntPositions
					last_update_time = TIME::now();
					struct AuxAntPosition_Pack
					{	
						uint8_t NrSV;
						uint8_t Error;
						uint8_t AmbiguityType;
						uint8_t AuxAntID;
						double DeltaEast;
						double DeltaNorth;
						double DeltaUp;
						double EastVel;
						double NorthVel;
						double UpVel;
					}__attribute__((packed));
					struct AuxAntPositions_Pack
					{	
						uint16_t crc;
						uint16_t ID;	
						uint16_t lenght;	
						uint32_t TOW;
						uint16_t WNC;
						uint8_t N;
						uint8_t SBLength;
						
						AuxAntPosition_Pack pos;
					}__attribute__((packed));
					
          AuxAntPositions_Pack* pack = (AuxAntPositions_Pack*)&frame_datas[2];
					
					bool available = false;
					if( pack->pos.Error==0 && pack->pos.AmbiguityType==0 )
						available = true;
					vector3<double> relPos( pack->pos.DeltaEast*100, pack->pos.DeltaNorth*100, pack->pos.DeltaUp*100 );
					if( pack->pos.DeltaEast>-1e+10 && pack->pos.DeltaNorth>-1e+10 && pack->pos.DeltaUp>-1e+10 )
						DAOSensorUpdate( 0, dao_key, relPos, available );
				}
			}
//			if( last_update_time.get_pass_time() > 2 )
//			{	//接收不到数据
//				PositionSensorUnRegister( default_rtk_sensor_index,sensor_key );
//				DAOSensorUnRegister(0);
//				//关闭Rtk注入
//				RtkPort_setEna( rtk_port_ind, false );
//				goto GPS_CheckBaud;
//			}	
		}
		else
		{	//接收不到数据
//			debug_test[18] =100;
//			PositionSensorUnRegister( default_rtk_sensor_index,sensor_key );
//			DAOSensorUnRegister(0);
//			//关闭Rtk注入
//			RtkPort_setEna( rtk_port_ind, false );
//			goto GPS_CheckBaud;
		}	
	}		
}

static bool RTK_DriverInit( Port port, uint32_t param )
{
	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	xTaskCreate( RTK_Server, "RTK2", 3200, driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_RTK_Mosaxic()
{
	PortFunc_Register( 20, RTK_DriverInit );
}
