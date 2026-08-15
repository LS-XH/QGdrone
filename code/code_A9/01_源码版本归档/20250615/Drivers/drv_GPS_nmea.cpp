#include "drv_GPS_nmea.hpp"
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

struct NMEA_GPS_State_Machine
{
	FrameName frame_name = FrameNone;
	uint32_t frame_datas_ind = 0;
	uint16_t frame_datas_length;
	uint8_t read_state = 0;	
	uint16_t crc=0;
	
	uint16_t dataP[120];
	uint16_t dataPInd;
	uint16_t dataPCount;
};

static uint8_t AsciiToByte(uint8_t b)
{
	uint8_t ret = 0;
	if(b >= '0' && b <= '9')
		ret = b - '0';
	else if(b >= 'A' && b <= 'F')
		ret = b - 'A' + 10;
	else if(b >= 'a' && b <= 'f')
		ret = b - 'a' + 10;
	else
		ret = 0;
	return ret;
}
static uint8_t ByteToAscii(uint8_t b)
{
	if(b < 10)
		return b + '0';
	else
		return b - 10 + 'A';
}

static uint8_t NMEA_Split_Pos(uint8_t *str,uint8_t pos)
{	 		    
	uint8_t offset = 0;
	while(pos)
	{		 
		if(*str=='*'||*str<' '||*str>'z')
			return 0xff;
		if(*str==',')
			pos--;
		str++;
		offset++;
	}
	return offset;	 
}

static double NMEA_Pow(double m,uint8_t n)
{
	double result=1;	 
	while(n--)
		result*=m;    
	return result;
}

static bool NMEA_Str2num(uint8_t *str, double& result)
{
	uint8_t *p=str;
	uint8_t ilen=0,flen=0,i;
	uint8_t mask=0;
	int64_t integerNum=0;
	double floatNum=0;
	int readCnt=0;
	while(1)
	{// 获取整数和小数的位数		
		readCnt++;
		if(readCnt>100)
			return false;
		
		if(*p=='-'){
			mask|=0x02;
			p++;
		}
		
		if(*p==','||(*p=='*'))
			break;		
		
	  if(*p=='.'){
			mask|=0X01;
			p++;
		}
		else if(*p>'9'||(*p<'0'))
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01)
			flen++;
		else 
			ilen++;
		p++;
	}
	
	// 跳过负号
	if(mask&0X02)
		str++;
	
	if(ilen>16)
		return false;

	if(flen>16)
		return false;
	
	// 计算整数大小
	for(i=0;i<ilen;i++)	 
		integerNum+=NMEA_Pow(10,ilen-1-i)*(str[i]-'0');

	// 计算小数大小	
	for(i=0;i<flen;i++) 
		floatNum+=NMEA_Pow(0.1,i+1)*(str[ilen+1+i]-'0');
	
	double res = integerNum+floatNum;
	
	// 是否是负数
	if(mask&0X02)
		res=-res;
	
	result = res;
	return true;
}	  
static double convert_from_ddmm(double val)
{
	int degrees = (int)val / 100;
	double minutes = val - degrees * 100.0;
	return degrees + minutes / 60.0;
}


static inline void NMEA_ResetRxStateMachine( NMEA_GPS_State_Machine* state_machine )
{
	state_machine->dataPInd=state_machine->dataPCount=0;
	state_machine->read_state=state_machine->frame_datas_ind=0;
}

static inline bool NEMA_Parse( NMEA_GPS_State_Machine* state_machine, uint8_t* frame_datas, uint8_t r_data )
{
	if( state_machine->frame_datas_ind > 1000 )
		NMEA_ResetRxStateMachine(state_machine);
	
	frame_datas[ state_machine->frame_datas_ind++ ] = r_data;
	switch( state_machine->read_state )
	{
		case 0:	//找包头
		{
			if( state_machine->frame_datas_ind == 1 )
			{
				if( r_data != '$' )
					state_machine->frame_datas_ind = 0;
				else
					state_machine->read_state = 1;
			}
			break;
		}
		case 1:
		{	
			if( state_machine->frame_datas_ind == 6 )
			{			
				if(strncmp((char*)&frame_datas[3],"GGA",3)==0){
					state_machine->frame_name = FrameGGA;
					state_machine->read_state = 2;
				}
				else if(strncmp((char*)&frame_datas[3],"RMC",3)==0){
					state_machine->frame_name = FrameGPRMC;
					state_machine->read_state = 2;
				}
				else if(strncmp((char*)&frame_datas[3],"GSV",3)==0){
					state_machine->frame_name = FrameGSV;
					state_machine->read_state = 2;
				}
				else
					NMEA_ResetRxStateMachine(state_machine);	
			}
			break;
		}
		case 2:	//*
		{
			if( r_data == ',')
			{
				state_machine->dataP[state_machine->dataPInd++] = state_machine->frame_datas_ind;
				if( state_machine->dataPInd > 118 )
					NMEA_ResetRxStateMachine(state_machine);
			}
			else if( r_data == '*' )
			{
				state_machine->read_state = 3;
				state_machine->crc=0;	
			}				
			break;
		}
	  case 3:	//校验第一个字节
		{
			state_machine->crc = AsciiToByte(r_data)*16;
			state_machine->read_state = 4;
			break;
		}
		case 4:	//校验第二个字节
		{
			state_machine->crc += AsciiToByte(r_data);
			state_machine->read_state = 5;	
			break;
		}
		case 5:	//0x0D
		{
			if( r_data == 0x0D ){
				state_machine->read_state = 6;	
			}else
				NMEA_ResetRxStateMachine(state_machine);
			break;
		}
		case 6:	//0x0A
		{	
			uint16_t data_len = state_machine->frame_datas_ind;
			uint16_t dataPCount = state_machine->dataPInd;
			NMEA_ResetRxStateMachine(state_machine);
			if( r_data == 0x0A )
			{
 				uint8_t result = frame_datas[1];
				for(int i = 2; frame_datas[i]!='*';i++ )
					result ^= frame_datas[i];
				if( result == state_machine->crc )
				{
					state_machine->frame_datas_length = data_len;
					state_machine->dataPCount = dataPCount;
					return true;	
				}
				else
					return false;
			}
			break;
		}
	}
	return false;
}

static void GPS_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	//GPS识别状态
	GPS_Scan_Operation current_GPS_Operation = GPS_Scan_Baud9600;
	//数据读取状态机
	__attribute__((aligned(4))) uint8_t NMEA_frame_datas[1024];
	NMEA_GPS_State_Machine nmea_state;
	//上次更新时间
	TIME last_update_time;
	
	//注册Rtk端口
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
	
GPS_CheckBaud:
	while(1)
	{
		//更改指定波特率
		driver_info.port.SetBaudRate( current_GPS_Operation, 3, 0.1 );
		switch(current_GPS_Operation)
		{
			case GPS_Scan_Baud9600:
				current_GPS_Operation = GPS_Scan_Baud38400;
				break;
			case GPS_Scan_Baud38400:
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
		
		//清空接收缓冲区准备接收数据
		driver_info.port.reset_rx(0.1);
		NMEA_ResetRxStateMachine(&nmea_state);
		TIME RxChkStartTime = TIME::now();
		while( RxChkStartTime.get_pass_time() < 2 )
		{
			uint8_t r_data;
			if( driver_info.port.read( &r_data, 1, 0.5, 0.1 ) )
			{
				bool nmeaParsed = NEMA_Parse( &nmea_state, NMEA_frame_datas, r_data );
				if( nmeaParsed && nmea_state.frame_name==FrameGPRMC )
					goto GPS_Present;
			}
		}
		uint8_t r_data;
		
	}
	
GPS_Present:
	uint32_t sensor_key = 0;
	//发送GNSS配置
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
	{	//注册传感器
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
	
	//开启Rtk注入
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
	
	//清空接收缓冲区准备接收数据
	driver_info.port.reset_rx(0.1);
	NMEA_ResetRxStateMachine(&nmea_state);
	last_update_time = TIME::now();
	
	//卫星
	uint32_t gsvRcd = 0;
	uint8_t gsvRcCount = 0;
	uint8_t gsvCount = 0;
	uint16_t satsCount;
	
	//附加数据
	double addition_inf[8] = {0};
	
	while(1)
	{
		uint8_t r_data;
		if( driver_info.port.read( &r_data, 1, 2, 0.1 ) )
		{
			bool nmeaParsed = NEMA_Parse( &nmea_state, NMEA_frame_datas, r_data );
			if( nmeaParsed )
			{
				switch(nmea_state.frame_name)
				{
					case FrameGPRMC:
					{
						bool available = false;
						
						char nmeaAV = NMEA_frame_datas[nmea_state.dataP[1]];
						if( nmeaAV == 'A' )
							available = true;
						
						double lat, lon;
						if( NMEA_Str2num( &NMEA_frame_datas[nmea_state.dataP[2]], lat ) == false )
						{
							available = false;
						}
						if( NMEA_Str2num( &NMEA_frame_datas[nmea_state.dataP[4]], lon ) == false )
						{
							available = false;
						}
						
						char nmeaNS = NMEA_frame_datas[nmea_state.dataP[3]];
						if( nmeaNS!='N' && nmeaNS!='S' )
						{
							available = false;
						}
						char nmeaEW = NMEA_frame_datas[nmea_state.dataP[5]];
						if( nmeaEW!='E' && nmeaEW!='W' )
						{
							available = false;
						}
						
						double nmeaVel, nmeaVelDir;
						if( NMEA_Str2num( &NMEA_frame_datas[nmea_state.dataP[6]], nmeaVel ) == false )
						{
							available = false;
						}
						if( NMEA_Str2num( &NMEA_frame_datas[nmea_state.dataP[7]], nmeaVelDir ) == false )
						{
							available = false;
						}
						nmeaVel *= 51.44444;
						
						
						lat = convert_from_ddmm(lat);
						if( nmeaNS == 'S' )
							lat = -lat;
						
						lon = convert_from_ddmm(lon);
						if( nmeaEW == 'W' )
							lon = -lon;
						
						vector3<double> position_Global;
						position_Global.x = lat;
						position_Global.y = lon;
						position_Global.z = 0;
						
						vector3<double> velocity;
						velocity.y = nmeaVel*cos(degree2rad(nmeaVelDir));	//North
						velocity.x = nmeaVel*sin(degree2rad(nmeaVelDir));	//East
						//velocity.z = -pack->velD * 0.1;	//Up
						
						PositionSensorChangeDataType( default_gps_sensor_index,sensor_key, Position_Sensor_DataType_sv_xy );
						if(available)
						{
							PositionSensorUpdatePositionGlobalVel( default_gps_sensor_index,sensor_key, position_Global, velocity, 
								available, //available
								-1, //delay
								100, 
								100, 
								addition_inf,
								100,	//xy LTtrust
								100	//z LTtrust
							);
						}
						else
							PositionSensorSetInavailable( default_gps_sensor_index,sensor_key );
						
						break;
					}
					
					case FrameGGA:
					{
						double fix;
						if( NMEA_Str2num( &NMEA_frame_datas[nmea_state.dataP[5]], fix ) == false )
						{
							switch((int)fix)
							{
								case 0:
									addition_inf[1] = 0;
									break;
								case 1:
									addition_inf[1] = 3;
									break;
								case 2:
									addition_inf[1] = 6;
									break;
								default:
									addition_inf[1] = 0;
									break;
							}
						}
						else
							addition_inf[1] = 0;
						
						if( NMEA_Str2num( &NMEA_frame_datas[nmea_state.dataP[7]], addition_inf[5] ) )
						{
							addition_inf[5] *= 100;
							addition_inf[4] = addition_inf[5];
						}
						break;
					}
					
					case FrameGSV:
					{
						double count, ind;
						if( NMEA_Str2num( &NMEA_frame_datas[nmea_state.dataP[0]], count ) == false )
							break;
						if( NMEA_Str2num( &NMEA_frame_datas[nmea_state.dataP[1]], ind ) == false )
							break;
						double sats;
						if( NMEA_Str2num( &NMEA_frame_datas[nmea_state.dataP[2]], sats ) == false )
							break;
						
						if( ( gsvCount != (uint8_t)count ) || ( gsvRcd & (1<<(uint8_t)(ind-1)) ) )
						{
							gsvCount = (uint8_t)count;
							gsvRcCount = 0;
							gsvRcd = 0;
							satsCount = 0;
						}
						
						++gsvRcCount;
						gsvRcd |= (uint8_t)(ind-1);
						satsCount += sats;
						
						if( gsvRcCount == gsvCount )
						{
							//addition_inf[0] = satsCount;
							
							gsvCount = count;
							gsvRcCount = 0;
							gsvRcd = 0;
							satsCount = 0;
						}
						addition_inf[0] = sats;
						
						
						break;
					}
					
					default:
						break;
				}
			}
		}
		else
		{	//接收不到数据
			PositionSensorUnRegister( default_gps_sensor_index,sensor_key );
			//关闭Rtk注入
			RtkPort_setEna( rtk_port_ind, false );
			goto GPS_CheckBaud;
		}
	}
}

static bool GPS_DriverInit( Port port, uint32_t param )
{
	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	xTaskCreate( GPS_Server, "GPS", 3000, driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_GPS_nmea()
{
	PortFunc_Register( 11, GPS_DriverInit );
}