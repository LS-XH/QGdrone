#include "drv_GPS.hpp"
#include "drv_GPS1_UbloxM10.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "Parameters.hpp"
#include "Commulink.hpp"
#include "StorageSystem.hpp"
#include "ControlSystem.hpp"

struct DriverInfo
{
	uint32_t param;
	Port port;
};

struct GpsConfig
{
	/*搜星GNSS设置
		0:不变
		63:GPS+SBAS+Galileo+BeiDou+IMES+QZSS
		119:GPS+SBAS+Galileo+IMES+QZSS+GLONASS
	*/
	uint8_t GNSS_Mode[8];
	
	//延时时间
	float delay[2];
};

enum GPS_Scan_Operation
{
	//在指定波特率下发送初始化信息
	GPS_Scan_Baud9600 = 9600 ,
	GPS_Scan_Baud38400 = 38400 ,
	GPS_Scan_Baud460800 = 460800 ,
	GPS_Scan_Baud115200 = 115200 ,
	
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
	uint8_t read_state = 0;	//0=
	uint8_t CK_A , CK_B;	//checksum
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
				if( r_data != 0xac )
					state_machine->frame_datas_ind = 0;
			}
			else
			{
				if( r_data == 0x62 )	//header found
				{
					state_machine->read_state = 1;
//					state_machine->frame_datas_ind = 0;
					state_machine->CK_A = state_machine->CK_B = 0;	//reset checksum
				}		
				else
					state_machine->frame_datas_ind = 0;
			}	
			break;
		}
		case 1:	//?Class ID和包长度
		{
			state_machine->CK_A += r_data;
			state_machine->CK_B += state_machine->CK_A;
			if( state_machine->frame_datas_ind == 6 )
			{
				state_machine->frame_datas_length = (*(unsigned short*)&frame_datas[4]) + 6;
				if( state_machine->frame_datas_length > 6 && state_machine->frame_datas_length < 1500 )
					state_machine->read_state = 2;						
				else
					ResetRxStateMachine(state_machine);
			}
			break;
		}
		case 2:	//读包内容
		{
			state_machine->CK_A += r_data;
			state_machine->CK_B += state_machine->CK_A;
			
			if( state_machine->frame_datas_ind == state_machine->frame_datas_length )
			{
				//payload read completed
				state_machine->read_state = 3;
			}
			break;
		}
		case 3://校验
		{
			if( state_machine->frame_datas_ind == state_machine->frame_datas_length + 1 )
			{
				if( r_data != state_machine->CK_A )
					ResetRxStateMachine(state_machine);
			}
			else
			{
				ResetRxStateMachine(state_machine);
				if( r_data == state_machine->CK_B )
					return state_machine->frame_datas_length;				
			}
		}
	}
	return 0;
}

static void GPSM10_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	//GPS识别状态
	uint32_t current_GPS_Operation = GPS_Scan_Baud460800;
	//数据读取状态机
	__attribute__((aligned(4))) uint8_t frame_datas[2048];
	//上次更新时间
	TIME last_update_time;
	
	//注册RTK串口
	RtkPort rtk_port;
	rtk_port.ena = false;
	rtk_port.write = driver_info.port.write;
	rtk_port.lock = driver_info.port.lock;
	rtk_port.unlock = driver_info.port.unlock;
	int8_t rtk_port_ind = RtkPortRegister(rtk_port);

	bool rtc_updated = false;
	
	//等待初始化完成
	while( getInitializationCompleted() == false )
		os_delay(0.1);
	
	os_delay(3.0);
	
	//读取是否需要记录PPK
	bool record_ppk = false;
	uint8_t log_ppk[8];
	if( ReadParam( "SDLog_PPK", 0, 0, (uint64_t*)log_ppk, 0 ) == PR_OK )
	{
		if( log_ppk[0] == 1 )
			record_ppk = true;
	}
	
GPS_CheckBaud:
	while(1)
	{
		//更改指定波特率
		
		//更改波特率
		driver_info.port.SetBaudRate( 460800, 3, 0.1 );
		//清空接收缓冲区准备接收数据
		driver_info.port.reset_rx(0.1);
		GPS_State_Machine gps_state;
		ResetRxStateMachine(&gps_state);
		TIME RxChkStartTime = TIME::now();
		while( RxChkStartTime.get_pass_time() < 2 )
		{
			uint8_t r_data;
			if( driver_info.port.read( &r_data, 1, 0.5, 0.1 ) )
			{
				uint16_t pack_length = GPS_ParseByte( &gps_state, frame_datas, r_data );
				if( pack_length )
				{
					if( frame_datas[2]==0x01 && frame_datas[3]==0x07 )
					{	//已识别到PVT包
						//跳转到GPS接收程序
						goto GPS_Present;
					}
				}
			}
		}
	}
	
GPS_Present:

	uint16_t sd_length = 0;
	os_delay(0.11);
	
	uint32_t sensor_key = 0;
	uint32_t mag_key = 0;
	//??GNSS??
	GpsConfig gps_cfg;
	if( ReadParamGroup( "GPS1Cfg", (uint64_t*)&gps_cfg, 0 ) == PR_OK )
	{
		//注册传感器
		sensor_key = PositionSensorRegister( default_gps_sensor_index , \
																					"GPS_Ubx" ,\
																					Position_Sensor_Type_GlobalPositioning , \
																					Position_Sensor_DataType_sv_xy , \
																					Position_Sensor_frame_ENU , \
																					gps_cfg.delay[0] , //延时
																					30 , //xy信任度
																					30 //z信任度
																				);
	}
	else
	{	
		//注册传感器
		sensor_key = PositionSensorRegister( default_gps_sensor_index , \
																					"GPS_Ubx" ,\
																					Position_Sensor_Type_GlobalPositioning , \
																					Position_Sensor_DataType_sv_xy , \
																					Position_Sensor_frame_ENU , \
																					0.1 , //延时
																					30 , //xy信任度
																					30 //z信任度
																				);
	}
	
	//开启RTK注入
	RtkPort_setEna( rtk_port_ind, true );
	
	//gps状态
	bool gps_available = false;
	bool z_available = false;
	TIME GPS_stable_start_time(false);
	double gps_alt;
	bool zHighPrec = false;
	TIME gps_update_TIME;
	
	//速度积分
	double last_velcocity_z = 0;
	
	//清除接收缓冲区准备接收数据
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
				if( frame_datas[2]==0x01 && frame_datas[3]==0x07 )
				{
					if( record_ppk )
						SDLog_Ubx( (const char*)frame_datas, 100 );
					last_update_time = TIME::now();
					struct UBX_NAV_PVT_Pack
					{
						uint8_t CLASS;
						uint8_t ID;
						uint16_t length;
						
						uint32_t iTOW;
						uint16_t year;
						uint8_t month;
						uint8_t day;
						uint8_t hour;
						uint8_t min;
						uint8_t sec;
						uint8_t valid;
						uint32_t t_Acc;
						int32_t nano;
						uint8_t fix_type;
						uint8_t flags;
						uint8_t flags2;
						uint8_t numSV;
						int32_t lon;
						int32_t lat;
						int32_t height;
						int32_t hMSL;
						uint32_t hAcc;
						uint32_t vAcc;
						int32_t velN;
						int32_t velE;
						int32_t velD;
						int32_t gSpeed;
						int32_t headMot;
						uint32_t sAcc;
						uint32_t headAcc;
						uint16_t pDOP;
						uint8_t res1[6];
						int32_t headVeh;
						uint8_t res2[4];
					}__attribute__((packed));
					UBX_NAV_PVT_Pack* pack = (UBX_NAV_PVT_Pack*)&frame_datas[2];
					
					uint8_t gps_fix = 0;
					if( pack->fix_type == 0 )
						gps_fix= 1;
					else
					{
						if( (pack->flags & 3) == 3 )
						{
							switch( pack->flags >> 6 )
							{
								case 1:
									gps_fix= 5;
									break;
								case 2:
									gps_fix= 6;
									break;
								default:
									gps_fix = pack->fix_type;
							}
						}
						else
							gps_fix = pack->fix_type;
					}
					if( ((pack->flags & 1) == 1) && (pack->fix_type == 0x03) && (pack->numSV >= 5) )
					{
						if( gps_available == false )
						{
							if( pack->hAcc < 2500 )
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
							double qAcc = 3500;
							if( inFlight )
								qAcc = 15000;
							if( pack->hAcc > qAcc )
								gps_available = z_available = false;
						}
						
					}
					else
					{
						gps_available = z_available = false;
						GPS_stable_start_time.set_invalid();
					}
					
					/*更新RTC时间(本地时间)*/	
						if( rtc_updated==false && gps_available )	{
							if( Lock_RTC() )
							{
								if( get_RTC_Updated() == false )
								{
									extern int8_t TimeZone;
									RTC_TimeStruct rtc;
									if((pack->flags & 1) == 1 && pack->fix_type == 0x03)
										TimeZone = GetTimeZone(pack->lat*1e-7,pack->lon*1e-7);
									UTC2LocalTime(&rtc, pack->year, pack->month, pack->day, pack->hour, pack->min, pack->sec, TimeZone, 0);
									Set_RTC_Time(&rtc);	
								}
								rtc_updated = true;
								UnLock_RTC();
							}
						}
          /*?更新RTC时间（本地时间）*/
					
					addition_inf[0] = pack->numSV;
					addition_inf[1] = gps_fix;
					addition_inf[4] = pack->hAcc*0.1;
					addition_inf[5] = pack->vAcc*0.1;
					addition_inf[6] = pack->sAcc*0.1;
					
					if( z_available )
					{
						if( pack->vAcc > 15000 )
							z_available = false;
					}
					else
					{
						if( pack->vAcc < 2500 )
						{
							gps_alt = pack->hMSL * 1e-1;
							z_available = true;
						}
					}
					double t = gps_update_TIME.get_pass_time_st();
					if( t > 1 )
						t = 1;						
					
					vector3<double> velocity;
					velocity.y = pack->velN * 0.1;	//North
					velocity.x = pack->velE * 0.1;	//East
					velocity.z = -pack->velD * 0.1;	//Up
					
					gps_alt += 0.5*(velocity.z + last_velcocity_z) * t;
					last_velcocity_z = velocity.z;
					
					//高度切换高低精度模式
					bool zSwitch = false;
					if( zHighPrec == false )
					{
						if( pack->vAcc < 250 )
						{
							double r_height = pack->hMSL*0.1;
							gps_alt = r_height;
							
							zSwitch = true;
							zHighPrec = true;
						}
					}
					else
					{
						if( pack->vAcc > 350 )
						{
							zHighPrec = false;
						}
					}
					//高度源
					if( zHighPrec )
					{	//高精度高度位置结果
						//使用绝对高度
						double r_height = pack->hMSL*0.1;
						gps_alt += 0.5*t * ( r_height - gps_alt );
					}
					else
					{
					}
					
					vector3<double> position_Global;
					position_Global.x = pack->lat * 1e-7;
					position_Global.y = pack->lon * 1e-7;
					position_Global.z = gps_alt;
	
					if( z_available && !zSwitch )
						PositionSensorChangeDataType( default_gps_sensor_index,sensor_key, Position_Sensor_DataType_sv_xyz );
					else
						PositionSensorChangeDataType( default_gps_sensor_index,sensor_key, Position_Sensor_DataType_sv_xy );
					
					//信任度
					double xy_trustD = pack->hAcc * 0.1;
					double z_trustD = pack->vAcc * 0.1;
					if( xy_trustD < 150 )
						xy_trustD = 150;
					if( z_trustD>20 && z_trustD<300 )
						z_trustD = 150;
					PositionSensorUpdatePositionGlobalVel( default_gps_sensor_index,sensor_key, position_Global, velocity, 
						gps_available, //available
						-1, //delay
						xy_trustD, 
						z_trustD, 
						addition_inf,
						xy_trustD,	//xy LTtrust
						z_trustD	//z LTtrust
					);
				}
			
				else if( frame_datas[2]==0x02 && frame_datas[3]==0x15 )
				{//UBX-RXM-RAWX	
					if( record_ppk )
						SDLog_Ubx( (const char*)frame_datas, 24 + 32 * frame_datas[11+6] );					
				}
				else if( frame_datas[2]==0x02 && frame_datas[3]==0x13 )
				{//UBX-RXM-SFRBX	
					if( record_ppk )
						SDLog_Ubx( (const char*)frame_datas, 16 + 4 * frame_datas[4+6] );					
				}		
				else if( frame_datas[2]==0x0D && frame_datas[3]==0x03 )
				{//UBX-TIM-TM2					
					static bool waitRasingEdge = false;
					static uint16_t camTrigCount = 0;
					if( !waitRasingEdge && (( frame_datas[7] & 0x04 ) == 0x04 && (frame_datas[7] & 0x80 ) == 0x80) )
					{
						if( record_ppk)
						{  
							*(uint16_t*)&frame_datas[8]=++camTrigCount; 
							uint8_t checksum_a = 0,checksum_b = 0;     
							for(uint8_t i = 2; i < 34; i++)
							{//重新计算校验
								checksum_a += frame_datas[i];
								checksum_b += checksum_a;
							}
							frame_datas[34] = checksum_a;
							frame_datas[35] = checksum_b;        
							SDLog_Ubx( (const char*)frame_datas, 36 );     
						}        
						waitRasingEdge = false;    
					}
					else if( !waitRasingEdge && ((frame_datas[7] & 0x04 )) == 0x04 )
					{
						if( record_ppk)
						{ 
							*(uint16_t*)&frame_datas[8]=camTrigCount;
							uint8_t checksum_a = 0,checksum_b = 0;     
							for(uint8_t i = 2; i < 34; i++)
							{//重新计算校验
								checksum_a += frame_datas[i];
								checksum_b += checksum_a;
							}
							frame_datas[34] = checksum_a;
							frame_datas[35] = checksum_b;          
							SDLog_Ubx( (const char*)frame_datas, 36 ); 
						}  
						waitRasingEdge=true;
					}
					else if(waitRasingEdge)
					{
						if( record_ppk && (frame_datas[7] & 0x80 ) == 0x80)
						{
							if( (frame_datas[7] & 0x04 ) == 0x04 )
								frame_datas[7] &= 0xFB;  
							*(uint16_t*)&frame_datas[8]=++camTrigCount;
							uint8_t checksum_a = 0,checksum_b = 0;     
							for(uint8_t i = 2; i < 34; i++)
							{//重新计算校验
								checksum_a += frame_datas[i];
								checksum_b += checksum_a;
							}
							frame_datas[34] = checksum_a;
							frame_datas[35] = checksum_b;          
							SDLog_Ubx( (const char*)frame_datas, 36 ); 
						}
						waitRasingEdge = false;
					}
				}					
				else if( frame_datas[2]==0x01 && frame_datas[3]==0x20 )
				{//UBX-NAV-TIMEGPS
					struct UBX_NAV_TIMEGPS_Pack
					{
						uint8_t CLASS;
						uint8_t ID;
						uint16_t length;
						
						uint32_t iTOW;
						int32_t fTOW;
						uint16_t week;
						uint8_t leapS;
						uint8_t valid;
						uint32_t tAcc;
					}__attribute__((packed));
					UBX_NAV_TIMEGPS_Pack* pack = (UBX_NAV_TIMEGPS_Pack*)&frame_datas[2];
					
          if( (pack->valid & (uint8_t)0x03) == (uint8_t)0x03 )
					{
						addition_inf[2] = pack->week;
						addition_inf[3] = pack->iTOW*1e-3 + pack->fTOW*1e-9;
					}
          if( record_ppk )					
						SDLog_Ubx( (const char*)frame_datas, 24 );												
				}else if(frame_datas[2]==0x01 && frame_datas[3]==0x50)
				{
						last_update_time = TIME::now();
						 typedef struct
							{
									uint8_t magId;
									uint8_t magX_L;
									uint8_t magX_H;
									uint8_t magY_L;
									uint8_t magY_H;
									uint8_t magZ_L;
									uint8_t magZ_H;
							} __attribute__((packed)) ExternalMagData_t;
							ExternalMagData_t *mag_data = (ExternalMagData_t *)&frame_datas[6];

							if (mag_data->magId != 0x10)
							{
									if (mag_key)
									{
											IMUMagnetometerUnRegister(External_Magnetometer_Index, mag_key);
											mag_key = 0;
									}
							}
							else
							{
									if (mag_key == 0)
											mag_key = IMUMagnetometerRegister(External_Magnetometer_Index, SName("e") + SName("M10Mag"), 0.003);
									if (mag_key)
									{
											vector3<int32_t> data;
											data.x = -(int16_t)((mag_data->magX_H << 8) | (mag_data->magX_L));
											data.y = -(int16_t)((mag_data->magY_H << 8) | (mag_data->magY_L));
											data.z = -(int16_t)((mag_data->magZ_H << 8) | (mag_data->magZ_L));
											IMUMagnetometerUpdate(External_Magnetometer_Index, mag_key, data, false);
									}
							}
				}								
			}			
			if( last_update_time.get_pass_time() > 2 )
				{	//搜不到数据
				PositionSensorUnRegister( default_gps_sensor_index,sensor_key );
				IMUMagnetometerUnRegister(External_Magnetometer_Index, mag_key);
				//关闭RTK注入
				//RtkPort_setEna( rtk_port_ind, false );
				goto GPS_CheckBaud;
			}
		}
		else
			{	//搜不到数据
			PositionSensorUnRegister( default_gps_sensor_index,sensor_key );
			IMUMagnetometerUnRegister(External_Magnetometer_Index, mag_key);	
			//关闭RTK注入
			//RtkPort_setEna( rtk_port_ind, false );
			goto GPS_CheckBaud;
		}
	}
}

static bool GPSM10_DriverInit( Port port, uint32_t param )
{
	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	xTaskCreate( GPSM10_Server, "GPS", 3000, driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_GPSM10(void)
{
	PortFunc_Register( 10, GPSM10_DriverInit ); //M10GPS
}