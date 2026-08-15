#include "drv_UM982.hpp"
#include "drv_RTK_DAO_Base.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "Parameters.hpp"
#include "Commulink.hpp"
#include "StorageSystem.hpp"
#include "drv_CRC.hpp"
#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include "ControlSystem.hpp"

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
	GPS_Scan_Baud230400 = 230400 ,
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

enum FrameName
{
	FrameNone=0,
	FrameKSXT,
	FrameGGA,
	FrameGSA,
	FrameGST,
	FrameGSV,
	FrameHDT,
	FrameHDT2,
	FrameGNRMC,
	FrameGPRMC,
	FrameGPVTG,
	FrameGPGLL,
	FrameGPZDA,
	FrameEVENTMARK
};

struct GPS_State_Machine
{
	uint16_t frame_id;
	uint8_t frameType;
	uint8_t header_length;
	FrameName frame_name = FrameNone;
	uint32_t frame_datas_ind = 0;
	uint16_t frame_datas_length;
	uint8_t read_state = 0;	
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
				if( r_data != 0xAA )
					state_machine->frame_datas_ind = 0;	
			}
			else if(state_machine->frame_datas_ind == 2)
			{
				if( r_data != 0x44 )
					state_machine->frame_datas_ind = 0;	
			}
			else
			{
				if( r_data == 0xB5 || r_data == 0x12)	//header found
				{
					if(r_data==0xB5)
						state_machine->header_length=24;
					else if(r_data==0x12)
						state_machine->header_length=28;
					state_machine->frameType=r_data;
					state_machine->read_state = 1;
//					state_machine->frame_datas_ind = 0;
					state_machine->crc = 0;	//reset checksum
				}		
				else
					state_machine->frame_datas_ind = 0;
			}	
			break;
		}
		case 1:	//读Class ID和包长度
		{
			if( state_machine->frameType == 0xB5 && state_machine->frame_datas_ind == 8 )
			{
				state_machine->frame_id = *(unsigned short*)&frame_datas[4];
				state_machine->frame_datas_length = (*(unsigned short*)&frame_datas[6]) + 24 + 4;
				if( state_machine->frame_datas_length > 28 && state_machine->frame_datas_length < 2000 )
					state_machine->read_state = 2;						
				else
					ResetRxStateMachine(state_machine);
			}
			else if( state_machine->frameType == 0x12 && state_machine->frame_datas_ind == 10 )
			{
				state_machine->frame_id = *(unsigned short*)&frame_datas[4];
				state_machine->frame_datas_length = (*(unsigned short*)&frame_datas[8]) + 28 + 4;
				if( state_machine->frame_datas_length > 32 && state_machine->frame_datas_length < 2000 )
					state_machine->read_state = 2;						
				else
					ResetRxStateMachine(state_machine);
			}
			break;
		}
		case 2:	//读包内容
		{
			if(state_machine->frameType == 0xB5)
			{
				if( state_machine->frame_datas_ind == state_machine->frame_datas_length - 12 )
				{
					//payload read completed
					state_machine->read_state = 3;
				}
			}
			else if(state_machine->frameType == 0x12)
			{
				if( state_machine->frame_datas_ind == state_machine->frame_datas_length - 14 )
				{
					//payload read completed
					state_machine->read_state = 3;
				}
			}
			break;
		}
		case 3://校验
		{
			if( state_machine->frame_datas_ind == state_machine->frame_datas_length)
			{
				uint32_t res =0;
				CRC_Calc( cg_CRC_CONFIG_CRC32_MAVLINK, (uint8_t*)&frame_datas[0], state_machine->frame_datas_length-4, &res);
				ResetRxStateMachine(state_machine);
				if( res == *(uint32_t*)&frame_datas[state_machine->frame_datas_length-4] )
					return state_machine->frame_datas_length;		
				else
					return 0;
			}
		}
	}
	return 0;
}

static bool send_init_msg(DriverInfo& driver_info)
{
  int len=0;
	char UM982Clr[100];
	len = sprintf( UM982Clr, "UNLOG COM2\r\n");	
	driver_info.port.write( (uint8_t*)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY );	
	driver_info.port.wait_sent(1);
	driver_info.port.reset_rx(0.1);
	len = sprintf( UM982Clr, "CONFIG\r\n");	
	driver_info.port.write( (uint8_t*)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY );	
	driver_info.port.wait_sent(1);
	os_delay(0.15);
	const char CmdResult[] = "$command,CONFIG,response: OK*54";
	const uint8_t CmdResultLen = strlen(CmdResult);
	len=0;
	TIME waitCorrectResponseTime = TIME::now();
	while( waitCorrectResponseTime.get_pass_time() < 3 )
	{
		if(driver_info.port.read( (uint8_t*)&UM982Clr[0], 1, 0.5, 1 )==0)
			return false;
		if( len < CmdResultLen )
		{
			if( UM982Clr[0] == CmdResult[len] )
				++len;
			else
				len = 0;
		}
		else
				break;
	}
	
	if( len != CmdResultLen )
		return false;

	// 波特率匹配完成
	for( uint8_t i=0; i < 22; i++ )
	{
		if(i==0)      // 停止所有信息输出
			len = sprintf( UM982Clr,  "UNLOG COM2\r\n");	
			
		else if(i==1) // 流动站工作模式
			len = sprintf( UM982Clr,  "MODE ROVER\r\n");	
		
		else if (i==2)// 设置 RTK 模块指令，数据最大龄期，秒为单位
			len = sprintf( UM982Clr,  "CONFIG RTK TIMEOUT 600\r\n");
		
		else if (i==3)// 设置接收的 DGPS差分数据的最大龄期，秒为单位，接收到的滞后于指定龄期的 DGPS 差分数据被忽略，也用于禁止 DGPS 定位计算
			len = sprintf( UM982Clr,  "CONFIG DGPS TIMEOUT 600\r\n");
		
		else if (i==4)// 设置双天线接收机的主天线（ANT1）与从天线（ANT2）之间距离保持固定
			len = sprintf( UM982Clr,  "CONFIG HEADING FIXLENGTH\r\n");			
		
		else if (i==5)// 设置 EVENT 功能，下降沿有效、两个有效脉冲之间的最短时间要求, 单位ms。若小于 TGuard，则第二个 Event 被忽视。默认值: 4，最小： 2；最大：3,599,999
			len = sprintf( UM982Clr,  "CONFIG EVENT ENABLE NEGATIVE 2\r\n");	
		
		else if (i==6)// 设置关闭单频定位状态的判定条件，基线长度超过 5km 时，精度满足需求依然可以使用固定解
			len = sprintf( UM982Clr,  "CONFIG SFRTK Enable\r\n");			
		
		else if (i==7)// 将 BDS 系统卫星的 B1C&B2a 信号编入 RTCM 协议
			len = sprintf( UM982Clr,  "CONFIG RTCMB1CB2a Enable\r\n");
		
		else if (i==8)// 使能接收机跟踪 GPS 卫星系
			len = sprintf( UM982Clr,  "UNMASK GPS\r\n");		
		
		else if (i==9)// 使能接收机跟踪 BDS 卫星系统
			len = sprintf( UM982Clr,  "UNMASK BDS\r\n");
		
		else if (i==10)// 使能接收机跟踪 GLO 卫星系统
			len = sprintf( UM982Clr, "UNMASK GLO\r\n");
		
		else if (i==11)// 使能接收机跟踪 GAL 卫星系统
			len = sprintf( UM982Clr,  "UNMASK GAL\r\n");

		else if (i==12)// 使能接收机跟踪 QZSS 卫星系统
			len = sprintf( UM982Clr,  "UNMASK QZSS\r\n");
		
		else if (i==13)// 输出 EVENT 发生时刻的精确绝对时间及相对时间
			len = sprintf( UM982Clr,  "LOG EVENTMARKB ONCHANGED\r\n");	

	  else if (i==14)// 设置接收机跟踪卫星的卫星截止角度
			len = sprintf( UM982Clr,  "MASK 10\r\n");	
		
		else if (i==15)// 接收机默设置为动态模式
			len = sprintf( UM982Clr,  "RTKDYNAMICS DYNAMIC\r\n");	
		
		else if (i==16)// 接收机运动的航向
			len = sprintf( UM982Clr,  "UNIHEADINGB 0.1\r\n");

		else if (i==17)// UTC 时间
			len = sprintf( UM982Clr,  "LOG TIMEB ONTIME 0.1\r\n");

		else if (i==18)// 精度因子信息
			len = sprintf( UM982Clr,  "STADOPB 1\r\n");
		
//		else if (i==19)// 原始观测数据信息
//			len = sprintf( UM982Clr,  "LOG COM1 RANGEB ONTIME 0.1\r\n");		
//		
//		else if (i==20)// GLONASS 导航电文
//			len = sprintf( UM982Clr,  "LOG COM1 GLORAWSTRINGB ONCHANGED\r\n");				
	
//		else if (i==21)// GPS 导航电文
//			len = sprintf( UM982Clr,  "LOG COM1 RAWGPSSUBFRAMEB ONCHANGED\r\n");		
//		
//		else if (i==22)// BDS 导航电文
//			len = sprintf( UM982Clr,  "LOG COM1 BDSRAWNAVSUBFRAMEB ONCHANGED\r\n");	

//		else if (i==23)// QZSS 导航电文
//			len = sprintf( UM982Clr,  "LOG COM1 QZSSRAWSUBFRAMEB ONCHANGED\r\n");
		
		else if (i==19)// 主天线计算出的最佳可用位置和速度Bestnav
			len = sprintf( UM982Clr,  "BESTNAVB 0.05\r\n");

//		else if (i==22)// 20Hz 的 GNGGA 信息
//			len = sprintf( UM982Clr,  "GPGGA 0.05\r\n");
				
//		else if (i==20)// HEADING2
//			len = sprintf( UM982Clr,  "LOG HEADING2B ONCHANGED\r\n");
//		
//		else if (i==21)// HEADING2
//			len = sprintf( UM982Clr,  "HEADINGMODE VARIABLELENGTH\r\n");
		
		else if (i==20)// 设置com2波特率为460800,8位数据位,无校验,一位停止位
			len = sprintf( UM982Clr,  "CONFIG COM1 460800 8 n 1\r\n");
		else if (i==21)// 设置com2波特率为460800,8位数据位,无校验,一位停止位
			len = sprintf( UM982Clr,  "CONFIG COM2 460800 8 n 1\r\n");
	
		driver_info.port.write( (uint8_t*)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY );	
		driver_info.port.wait_sent(1);
		os_delay(0.1);
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
	__attribute__((aligned(4))) uint8_t frame_datas[2048];
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
		if( log_ppk[0] == 2 )
			record_ppk = true;
	}

GPS_CheckBaud:
	while(1)
	{
		//更改指定波特率
		driver_info.port.SetBaudRate( current_GPS_Operation, 3, 0.1 );
		//切换波特率
		switch(current_GPS_Operation)
		{
			case GPS_Scan_Baud9600:
				current_GPS_Operation = GPS_Scan_Baud57600;
				break;
			case GPS_Scan_Baud57600:
				current_GPS_Operation = GPS_Scan_Baud38400;
				break;
			case GPS_Scan_Baud38400:
				current_GPS_Operation = GPS_Scan_Baud230400;
				break;
			case GPS_Scan_Baud230400:
				current_GPS_Operation = GPS_Scan_Baud460800;
				break;
			case GPS_Scan_Baud460800:
				current_GPS_Operation = GPS_Scan_Baud115200;
				break;
			default:
			case GPS_Scan_Baud115200:
				current_GPS_Operation = GPS_Scan_Baud9600;
				break;
		}
		//发送配置
    if( send_init_msg(driver_info) == false )
			continue;

		//更改波特率
		driver_info.port.SetBaudRate( 460800, 1, 0.1 );		
		//清空接收缓冲区准备接收数据
		driver_info.port.reset_rx(0.1);
		GPS_State_Machine gps_state;
		ResetRxStateMachine(&gps_state);
		TIME RxChkStartTime = TIME::now();
		
		while( RxChkStartTime.get_pass_time()<3 )
		{
			uint8_t r_data;
			if( driver_info.port.read( &r_data, 1, 0.5, 0.1 ) )
			{
				bool res = GPS_ParseByte( &gps_state, frame_datas, r_data );
				if( res )
				{
					if( gps_state.frame_id == 2118 )
					{	//已识别,跳转到gps接收程序
						goto GPS_Present;
					}
				}
			}
		}		
	}	
	
GPS_Present:
	//重发配置
	//send_init_msg(driver_info);
		
	uint32_t sensor_key = 0;
	uint32_t dao_key = 0;
	GpsDAOConfig gps_cfg;
	if( ReadParamGroup( "GPSDAOCfg", (uint64_t*)&gps_cfg, 0 ) == PR_OK )
	{	
		//注册传感器
		sensor_key = PositionSensorRegister( default_rtk_sensor_index , \
																					"RTK_UM982" ,\
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
																					"RTK_UM982" ,\
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
	bool zHighPrec = false;
	TIME GPS_stable_start_time(false);
	double gps_alt;
	TIME gps_update_TIME;
	
	//速度积分
	double last_velcocity_z = 0;
	
	//清空接收缓冲区准备接收数据
	driver_info.port.reset_rx(0.1);
	GPS_State_Machine gps_state,gps_state_nmea;
	ResetRxStateMachine(&gps_state);
	frame_datas[0] = frame_datas[1] = 0;
	last_update_time = TIME::now();
	
	struct Dop_Pack
	{	
		uint32_t itow;
		float gdop;//几何精度因子
		float pdop;//位置精度因子
		float tdop;//时间精度因子
		float vdop;//垂直精度因子
		float hdop;//水平精度因子
		float ndop;//北向精度因子
		float edop;//东向精度因子
		float Cutoff;//截至高度角  
		float rsv2;
		uint16_t svNum;
		uint16_t prn;
	}__attribute__((packed));
	Dop_Pack dop_pack = {0};		
	
	//附加数据
	double addition_inf[8] = {0};
	
	while(1)
	{
		uint8_t r_data;
		if( driver_info.port.read( &r_data, 1, 2, 0.1 ) )
		{
			if( GPS_ParseByte( &gps_state, frame_datas, r_data ) )
			{
				if( gps_state.frame_id == 2118 )
				{// BESTNAV 最佳位置和速度
					//记录
					if(record_ppk)
						SDLog_Ubx( (const char*)frame_datas, gps_state.frame_datas_length );		
					
					last_update_time = TIME::now();
					struct Best_Nav_Pack
					{	
						uint32_t PosStatus;//位置解状态
						uint32_t PosType;//位置类型
						double lat;//纬度，deg
						double lon;//经度，deg
						double height;//海拔高，m
						float undulation;//大地水准面差距- 大地水准面和WGS84 椭球面之间的距,米
						uint32_t datum_id;//坐标系 ID 号,当前仅支持 WGS84
						float LatAcc;//纬度标准差，m 
						float LonAcc;//经度标准差，m 
						float HeightAcc;//高度标准差，m 
						uint8_t stn_id[4];//基站 ID
						float diff_age;//差分龄期，s  
						float sol_age;//解的龄期，s  
						uint8_t SVTrack;//跟踪的卫星数  
						uint8_t SVUse;//在解中使用的卫星数
						uint8_t rsv2[3];
						uint8_t ext_sol_stat;//扩展解的状态
						uint8_t mask1;//Galileo 使用的信号掩码
						uint8_t mask2;//GPS, GLONASS 和 BDS 使用的信号掩码
						uint32_t VelStatus;//速度解状态
						uint32_t VelType;//速度类型
						float latency;//据速度时标计算的延迟值，以秒为单位
						float age;//差分龄期，s  
						double horSpeed;//对地水平速度，m/s 
						double horSpeedAngle;//相对于真北的实际对地运动方向（相对地面轨迹），deg
						double VerticalSpeed;//垂直速度，m/s
						float VerspdStd;//高程速度标准差，单位 m/s
						float HorspdStd;//水平速度标准差，单位 m/s  
					}__attribute__((packed));
					Best_Nav_Pack* pack = (Best_Nav_Pack*)&frame_datas[gps_state.header_length];
				
					uint8_t gps_fix = 0;
					if(pack->PosType==16||(pack->PosType==17)||(pack->PosType==18))// 单点定位
						gps_fix = 3; 
					else if(pack->PosType==32||pack->PosType==33||pack->PosType==34)//RTK 浮点解
						gps_fix = 5;	
					else if(pack->PosType==48||pack->PosType==49||pack->PosType==50)// RTK 固定解
						gps_fix = 6;
					
					double hAcc = sqrt(pack->LatAcc*pack->LatAcc+pack->LonAcc*pack->LonAcc);
					double vAcc = pack->HeightAcc;
					
					if( gps_fix!=0  && pack->SVTrack>= 7 )
					{
						if( gps_available == false )
						{
							if( hAcc < 6 )
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
							bool inFlight;
							get_is_inFlight(&inFlight);
							double qAcc = 8;
							if( inFlight )
								qAcc = 20;
							if( hAcc > qAcc )
								gps_available = z_available = false;
						}
					}
					else
					{
						gps_available = z_available = false;
						GPS_stable_start_time.set_invalid();
					}	
					
					addition_inf[0] = pack->SVTrack;
					addition_inf[1] = gps_fix;
					addition_inf[4] = hAcc*100;
					addition_inf[5] = vAcc*100;
					addition_inf[6] = 0;
					
					if( z_available )
					{
						if( vAcc > 16 )
							z_available = false;
					}
					else
					{
						if( vAcc < 8 )
						{
							gps_alt = pack->height * 100;
							z_available = true;
						}
					}
					double t = gps_update_TIME.get_pass_time_st();
					if( t > 1 )
						t = 1;	

					vector3<double> velocity;
					velocity.y = pack->horSpeed*cos(degree2rad(pack->horSpeedAngle))*100;	//North
					velocity.x = pack->horSpeed*sin(degree2rad(pack->horSpeedAngle))*100;	//East
					velocity.z = pack->VerticalSpeed*100;	//Up
					
					gps_alt += 0.5*(velocity.z + last_velcocity_z) * t;
					last_velcocity_z = velocity.z;
					
					//高度切换高低精度模式
					bool zSwitch = false;
					if( zHighPrec == false )
					{
						if( vAcc < 0.5 )
						{
							double r_height = pack->height * 100;
							gps_alt = r_height;
							
							zSwitch = true;
							zHighPrec = true;
						}
					}
					else
					{
						if( vAcc > 1 )
						{
							zHighPrec = false;
						}
					}
					//高度源
					if( zHighPrec )
					{	//高精度高度位置结果
						//使用绝对高度
						double r_height = pack->height * 100;
						gps_alt += 0.5*t * ( r_height - gps_alt );
					}
					else
					{
					}
					
					vector3<double> position_Global;
					position_Global.x = pack->lat;
					position_Global.y = pack->lon;
					position_Global.z = gps_alt;

					if( z_available && !zSwitch )
						PositionSensorChangeDataType( default_rtk_sensor_index,sensor_key, Position_Sensor_DataType_sv_xyz );
					else
						PositionSensorChangeDataType( default_rtk_sensor_index,sensor_key, Position_Sensor_DataType_sv_xy );
					
					//信任度
					double xy_trustD = hAcc*10;
					double z_trustD = vAcc*10;
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
				else if( gps_state.frame_id == 972 )
				{// UNIHEADING 航向信息
					last_update_time = TIME::now();
					struct Heading_Pack
					{	
						uint32_t solStatus;//位置解状态
						uint32_t posType;//位置类型
						float length;	//基线长 (0 到 3000 m)  
						float heading;//航向 (0 到 360.0 deg)
						float pitch;	//俯仰(±90 deg)  
						float rsv2;
						float hdgstddev;//航向标准偏差
						float ptchstddev;//俯仰标准偏差
						uint8_t stn_id[4];//基站 ID
						uint8_t SVTrack;//跟踪的卫星数  
						uint8_t SVUse;//在解中使用的卫星数
						uint8_t obs;//截止高度角以上的卫星数
						uint8_t multi;//截止高度角以上有L2观测的卫星数
						uint8_t rsv3;
						uint8_t ext_sol_stat;//扩展解的状态
						uint8_t mask1;//Galileo 使用的信号掩码
						uint8_t mask2;//GPS, GLONASS 和 BDS 使用的信号掩码						
					}__attribute__((packed));
					Heading_Pack* pack = (Heading_Pack*)&frame_datas[gps_state.header_length];			
					double len_pos = sq(gps_cfg.DRTK_VecX[0]) + sq(gps_cfg.DRTK_VecY[0]) + sq(gps_cfg.DRTK_VecZ[0]);
					bool available = false;
					if( len_pos > 7*7 )
					{
						double heading = 90.0f - pack->heading;
						double sinY, cosY;
						double sinPit, cosPit;
						fast_sin_cos( degree2rad(heading), &sinY, &cosY );
						fast_sin_cos( degree2rad(pack->pitch), &sinPit, &cosPit );
						
						vector3<double> relPos( 
							pack->length*cosPit*100 * cosY,
							pack->length*cosPit*100 * sinY,  
							pack->length*sinPit*100 );
						if( pack->solStatus==0 && (pack->posType==50 || pack->posType==48) )
							available=true;
						DAOSensorUpdate( 0, dao_key, relPos, available );
					}
				}
				else if(gps_state.frame_id == 309)
				{// EVENT触发
					last_update_time = TIME::now();
					struct Event_Pack
					{	
						uint32_t itow;
						uint8_t eventID;//事件编码（Event 1 或Event 2）
						uint8_t status; //事件状态（待定）  
						uint8_t reserved0;
						uint8_t reserved1;
						uint32_t week;  //周
						uint32_t reserved2;
						uint32_t offset_second;   //按当前 GGA 输出频率，EVENT 时刻与最接近的GGA 输出的绝对时间之间的偏移值(second)
						uint32_t offset_SubSecond;//按当前 GGA 输出频率，EVENT 时刻与最接近的GGA 输出的绝对时间之间的偏移值(nanosecond)
						uint16_t prn;
					}__attribute__((packed));	
					Event_Pack* pack = (Event_Pack*)&frame_datas[gps_state.header_length];	
					if(record_ppk)
						SDLog_Ubx( (const char*)frame_datas, gps_state.frame_datas_length );						
				}
				else if(gps_state.frame_id == 954)
				{// STADOP DOP信息
					last_update_time = TIME::now();
					dop_pack = *(Dop_Pack*)&frame_datas[gps_state.header_length];	
				}
				else if(gps_state.frame_id == 101)
				{// RECTIME 信息
					last_update_time = TIME::now();
					struct Time_Pack
					{	
						uint32_t itow;
						uint32_t clock_status;//时钟模型状态
						double offset;//相对于 GPS 时的接收机钟差
						double Offset_std;//GPS 时间到 UTC 时间的偏
						uint32_t year;//UTC 年  
						uint8_t month;//UTC 月 (0-12)
						uint8_t day;	//UTC 天 (0-31)
						uint8_t hour; //UTC 小时 (0-23)  
						uint8_t min;	//UTC 分钟 (0-59)  
						uint32_t ms;	//UTC 毫秒 (0-60999)
						uint32_t utc_status;//UTC 状态：0 = INVALID，无效； 1 =VALID，有效； 2 = WARNING 11 
					}__attribute__((packed));					
					Time_Pack* pack = (Time_Pack*)&frame_datas[gps_state.header_length];	
					
          if( (pack->utc_status & (uint8_t)0x01) == (uint8_t)0x01 )
					{
						addition_inf[2] = *(uint16_t*)&frame_datas[14];
						addition_inf[3] = *(uint32_t*)&frame_datas[16];
					}					
					if(record_ppk)
						SDLog_Ubx( (const char*)frame_datas, gps_state.frame_datas_length );							
				}				
				else if(gps_state.frame_id == 43)
				{// 原始观测数据信息
					if(record_ppk)
						SDLog_Ubx( (const char*)frame_datas, gps_state.frame_datas_length );	
					//debug_test[23] =	gps_state.frame_datas_length;
				}
				else if(gps_state.frame_id == 1695)
				{// BDS 导航电文
					if(record_ppk)
						SDLog_Ubx( (const char*)frame_datas, gps_state.frame_datas_length );						
				}
				else if(gps_state.frame_id == 722)
				{// GLONASS 导航电文
					if(record_ppk)
						SDLog_Ubx( (const char*)frame_datas, gps_state.frame_datas_length );						
				}
				else if(gps_state.frame_id == 1330)
				{// QZSS 导航电文
					if(record_ppk)
						SDLog_Ubx( (const char*)frame_datas, gps_state.frame_datas_length );						
				}
				else if(gps_state.frame_id == 25)
				{// GPS 导航电文
					if(record_ppk)
						SDLog_Ubx( (const char*)frame_datas, gps_state.frame_datas_length );						
				}
			}
			
			if( last_update_time.get_pass_time() > 2 )
			{	//接收不到数据
				PositionSensorUnRegister( default_rtk_sensor_index,sensor_key );
				DAOSensorUnRegister(0,dao_key);
				//关闭Rtk注入
				RtkPort_setEna( rtk_port_ind, false );
				goto GPS_CheckBaud;
			}	
		}
		else
		{	//接收不到数据
			PositionSensorUnRegister( default_rtk_sensor_index,sensor_key );
			DAOSensorUnRegister(0,dao_key);
			//关闭Rtk注入
			RtkPort_setEna( rtk_port_ind, false );
			goto GPS_CheckBaud;
		}	
	}		
}

static bool RTK_DriverInit( Port port, uint32_t param )
{
	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	xTaskCreate( RTK_Server, "RTK982", 2500, driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_RTK_UM982()
{
	PortFunc_Register( 17, RTK_DriverInit );
}
