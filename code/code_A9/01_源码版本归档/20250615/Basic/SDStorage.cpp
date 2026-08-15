#include "StorageSystem.hpp"
#include "Basic.hpp"
#include "drv_SDMMC.hpp"
#include "Parameters.hpp"
#include "fatfs.h"
#include <stdlib.h>

#include "Sensors.hpp"
#include "MeasurementSystem.hpp"
#include "Sensors.hpp"
#include "semphr.h"
#include "stream_buffer.h"
#include "message_buffer.h"
#include "ControlSystem.hpp"
#include "Commulink.hpp"
#include "usb_composite.h"
#include "Receiver.hpp"
#include "Modes.hpp"
#include "drv_ADC.hpp"

//SD卡是否初始化完成
	static bool SD_Init_Complete = false;
	static bool SD_Init_Success = false;
	extern "C" bool Get_SD_Init_Complete(void)
	{
	  return SD_Init_Complete;
	}
	static inline void Set_SD_Init_Complete(void)
	{
		SD_Init_Complete = true;
	}
	static inline void Clear_SD_Init_Complete(void)
	{
		SD_Init_Success = false;
		SD_Init_Complete = false;
	}
	
	
	extern "C" bool Get_SD_Init_Success(void)
	{
	  return SD_Init_Success;
	}
	static inline void Set_SD_Init_Success(void)
	{
		SD_Init_Success = true;
	}
	static inline void Clear_SD_Init_Success(void)
	{
		SD_Init_Success = false;
	}

	static bool mscWritten = false;
	extern "C" void mscDiskWritten(void)
	{
	  mscWritten = true;
	}
/*SD卡信息*/
	static bool SdAvailable = false;
	static uint64_t tot_size = 0;
	static uint64_t fre_size = 0;
	static float totGB = 0;
	static float freGB = 0;
	static uint8_t sdSaveFailCounter = 0;
	static inline void reset_SdInfo() 
	{
		SdAvailable = false;
		tot_size = fre_size = 0;
		totGB = freGB = 0;
		sdSaveFailCounter = 0;
	}
	static inline void update_SdSize( uint64_t tSize, uint64_t writtenSize )
	{
		if( writtenSize < fre_size )
			fre_size -= writtenSize;
		else
			fre_size = 0;
		freGB = (fre_size>>20) / 1024.0f;
		
		if( tSize != writtenSize )
		{
			if( sdSaveFailCounter < 250 )
				++sdSaveFailCounter;
		}
		else if( sdSaveFailCounter > 0 )
			--sdSaveFailCounter;
	}
	float getSdTotalSizeGB() { return totGB; }
	float getSdFreeSizeGB() { return freGB; }
/*SD卡信息*/
	
/*SD Log*/
	//Log缓冲区
	#define BufSize 20480
	Static_DTCMBuf uint8_t LogStreamBufferStorage[ BufSize+1 ];
	Static_DTCMBuf StaticMessageBuffer_t LogStreamBufferStruct;
	Static_DTCMBuf MessageBufferHandle_t LogStreamBuffer = xMessageBufferCreateStatic( BufSize,
                                                 LogStreamBufferStorage,
                                                 &LogStreamBufferStruct );;
	static SemaphoreHandle_t LogSemphr = xSemaphoreCreateMutex();
	
	static bool Lock_SDLog( double TIMEOUT = -1 )
	{
		if(get_is_usb_connected())
			return false;
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake( LogSemphr , TIMEOUT_Ticks ) )
			return true;
		return false;
	}
	static void UnLock_SDLog()
	{
		xSemaphoreGive(LogSemphr);
	}
	
	/*Log数据*/
		bool SDLog_Msg_SysState( double SyncTIMEOUT )
		{
			//获取接收机
			Receiver rc;	SName rcName;
			getReceiver(&rc,&rcName);
			//获取控制器状态
			bool att_ctrl_ena, alt_ctrl_ena, pos_ctrl_ena;
			is_Attitude_Control_Enabled(&att_ctrl_ena);
			is_Altitude_Control_Enabled(&alt_ctrl_ena);
			is_Position_Control_Enabled(&pos_ctrl_ena);
			//获取电量
			BatteryInfo batInfo;
			int8_t batInd = -1;
			getCurrentBatteryInfo(&batInfo, &batInd);
			
			struct Msg
			{
				uint8_t msg_type;
				uint8_t length;
				/*flags
					bit0-姿态控制器开启
					bit1-高度控制器开启
					bit2-位置控制器开启
				*/
				uint16_t flags;
				uint32_t Time;
				uint8_t cpuLoad;
				int8_t currentRc;
				uint8_t MainBatRMPercent;
				int8_t batId;
				float MainBatVoltage;
				float MainBatCurrent;
				float RcChannels[8];
				int8_t cpuTemperature;
				uint8_t flyMode;	//bit7:是否为安全模式
				uint8_t rsv2[6];
			}__attribute__((__packed__));
			Msg msg;
			
			msg.msg_type = LogMsg_SysState;
			msg.length = sizeof(msg);
			//flags
			uint16_t flags = 0;
			if( att_ctrl_ena ) flags |= (1<<0);
			if( alt_ctrl_ena ) flags |= (1<<1);
			if( pos_ctrl_ena ) flags |= (1<<2);
			msg.flags = flags;
			
			msg.Time = TIME::get_System_Run_Time()*1e+4;
			//CPU load
			msg.cpuLoad = getCPULoad();
			//接收机状态
			if( rc.available == false )
				msg.currentRc = -1;
			else 
			{
				if( rcName == "Sbus" )
					msg.currentRc = 0;
				else if( rcName == "PPM" )
					msg.currentRc = 1;
				else if( rcName == "JoyStickMv" )
					msg.currentRc = 10;
				memcpy( msg.RcChannels, rc.data, 8*sizeof(float) );
			}
			//电池状态
			msg.batId = batInd;
			msg.MainBatVoltage = batInfo.totalVoltRaw;
			msg.MainBatCurrent = batInfo.totalCurrent;
			msg.MainBatRMPercent = batInfo.totalPercent;
			msg.cpuTemperature = adcGet_CPUTemperature();
			msg.flyMode = (is_MSafeCtrl()?(1<<7):0) | getCurrentFlyMode();
			if( Lock_SDLog(SyncTIMEOUT) )
			{
				xMessageBufferSend( LogStreamBuffer, &msg, sizeof(msg), 0 );				
				UnLock_SDLog();
				return true;
			}
			return false;
		}
	
		bool SDLog_Msg_IMUSensor( uint8_t ind,IMUType imu_type, IMU_Sensor sensor, double SyncTIMEOUT )
		{
			if( imu_type!=IMUType_Accelerometer &&
					imu_type!=IMUType_Gyroscope &&
					imu_type!=IMUType_Magnetometer )
				return false;
			
			struct Msg
			{
				uint8_t msg_type;
				uint8_t length;
				//类别 1-加速度 2-陀螺仪 3-磁罗盘 4-双天线
				uint8_t type;
				//序号
				uint8_t ind;
				uint32_t Time;
				//原始值(双天线传感器为双天线向量)
				float rawValue[3];
				//校准值(双天线传感器为基准向量)
				float value[3];
				//更新周期
				float updateT;
				//温度(双天线传感器为延时时间)
				float temperature;
			}__attribute__((__packed__));
			Msg msg;
			
			msg.msg_type = LogMsg_IMUSensor;
			msg.length = sizeof(msg);
			msg.type = imu_type;
			msg.ind = ind;
			
			msg.Time = TIME::get_System_Run_Time()*1e+4;
			
			msg.rawValue[0] = sensor.data_raw.x;
			msg.rawValue[1] = sensor.data_raw.y;
			msg.rawValue[2] = sensor.data_raw.z;
			
			msg.value[0] = sensor.data.x;
			msg.value[1] = sensor.data.y;
			msg.value[2] = sensor.data.z;
			
			msg.updateT = sensor.sample_time;
			msg.temperature = sensor.temperature;
			
			if( Lock_SDLog(SyncTIMEOUT) )
			{
				xMessageBufferSend( LogStreamBuffer, &msg, sizeof(msg), 0 );				
				UnLock_SDLog();
				return true;
			}
			return false;
		}
		bool SDLog_Msg_DFSensor( uint8_t ind, DAO_Sensor sensor, double SyncTIMEOUT )
		{
			struct Msg
			{
				uint8_t msg_type;
				uint8_t length;
				//类别 1-加速度 2-陀螺仪 3-磁罗盘 4-双天线
				uint8_t type;
				//序号
				uint8_t ind;
				uint32_t Time;
				//原始值(双天线传感器为双天线向量)
				float rawValue[3];
				//校准值(双天线传感器为基准向量)
				float value[3];
				//更新周期
				float updateT;
				//温度(双天线传感器为延时时间)
				float temperature;
			}__attribute__((__packed__));
			Msg msg;
			
			msg.msg_type = LogMsg_IMUSensor;
			msg.length = sizeof(msg);
			msg.type = IMUType_DFAntenna;
			msg.ind = ind;
			
			msg.Time = TIME::get_System_Run_Time()*1e+4;
			
			msg.rawValue[0] = sensor.relPos.x;
			msg.rawValue[1] = sensor.relPos.y;
			msg.rawValue[2] = sensor.relPos.z;
			
			msg.value[0] = sensor.st_relPos.x;
			msg.value[1] = sensor.st_relPos.y;
			msg.value[2] = sensor.st_relPos.z;
			
			msg.updateT = sensor.sample_time;
			msg.temperature = sensor.delay;
			
			if( Lock_SDLog(SyncTIMEOUT) )
			{
				xMessageBufferSend( LogStreamBuffer, &msg, sizeof(msg), 0 );				
				UnLock_SDLog();
				return true;
			}
			return false;
		}
		
		bool SDLog_Msg_PosSensor( uint8_t ind, Position_Sensor sensor, double SyncTIMEOUT )
		{
			struct MsgS
			{
				uint8_t msg_type;
				uint8_t length;
				uint8_t SensorType;	//为Position_Sensor_Type
				uint8_t DataType;	//Position_Sensor_DataType
				uint32_t Time;
				uint8_t sensor;	//传感器编号（bit7为是否可用）
				uint8_t frame;	//Position_Sensor_frame
				uint8_t rsv1[2];
				uint16_t eph;	//水平精度cm
				uint16_t epv;	//垂直精度cm
				double posx;
				double posy;
				double posz;
			}__attribute__((__packed__));
			struct MsgV
			{
				uint8_t msg_type;
				uint8_t length;
				uint8_t SensorType;	//为Position_Sensor_Type
				uint8_t DataType;	//Position_Sensor_DataType
				uint32_t Time;	
				uint8_t sensor;	//传感器编号（bit7为是否可用）
				uint8_t frame;	//Position_Sensor_frame
				uint8_t rsv1[2];	
				uint16_t eph;	//水平精度cm
				uint16_t epv;	//垂直精度cm
				double velx;
				double vely;
				double velz;
			}__attribute__((__packed__));
			struct MsgSV
			{
				uint8_t msg_type;
				uint8_t length;
				uint8_t SensorType;	//为Position_Sensor_Type
				uint8_t DataType;	//Position_Sensor_DataType
				uint32_t Time;
				uint8_t sensor;	//传感器编号（bit7为是否可用）
				uint8_t frame;	//Position_Sensor_frame
				uint8_t rsv1[2];
				uint16_t eph;	//水平精度cm
				uint16_t epv;	//垂直精度cm
				double posx;
				double posy;
				double posz;
				double velx;
				double vely;
				double velz;
			}__attribute__((__packed__));
			struct MsgGS
			{
				uint8_t msg_type;
				uint8_t length;
				uint8_t SensorType;	//为Position_Sensor_Type
				uint8_t DataType;	//Position_Sensor_DataType
				uint32_t Time;
				uint8_t sensor;	//传感器编号（bit7为是否可用）
				uint8_t frame;	//Position_Sensor_frame
				uint8_t numSV;	//星数
				uint8_t fix;	//fix_type
				uint16_t eph;	//水平精度cm
				uint16_t epv;	//垂直精度cm
				double OriginLat;
				double OriginLon;
				double Lat;
				double Lon;
				double posx;
				double posy;
				double posz;
			}__attribute__((__packed__));
			struct MsgGSV
			{
				uint8_t msg_type;
				uint8_t length;
				uint8_t SensorType;	//为Position_Sensor_Type
				uint8_t DataType;	//Position_Sensor_DataType
				uint32_t Time;
				uint8_t sensor;	//传感器编号（bit7为是否可用）
				uint8_t frame;	//Position_Sensor_frame
				uint8_t numSV;	//星数
				uint8_t fix;	//fix_type
				uint16_t eph;	//水平精度cm
				uint16_t epv;	//垂直精度cm
				double OriginLat;
				double OriginLon;
				double Lat;
				double Lon;
				double posx;
				double posy;
				double posz;
				double velx;
				double vely;
				double velz;
			}__attribute__((__packed__));
			
			uint8_t dataType = sensor.data.sensor_DataType;
			if( dataType > 100 )
				dataType -= 100;
			if( dataType < 8 )
			{	//s传感器
				if( sensor.data.sensor_type == Position_Sensor_Type_GlobalPositioning )
				{	//GS
					MsgGS msg;
					msg.msg_type = LogMsg_PosSensor;
					msg.length = sizeof(msg);
					msg.SensorType = sensor.data.sensor_type;
					msg.DataType = sensor.data.sensor_DataType;
					msg.Time = TIME::get_System_Run_Time()*1e+4;
					msg.sensor = sensor.data.available ? (1<<7)|ind : ind;
					msg.frame = sensor.data.data_frame;
					msg.OriginLat = rad2degree(sensor.data.mp.lat0_rad);
					msg.OriginLon = rad2degree(sensor.data.mp.lon0_rad);
					msg.Lat = sensor.data.position_Global.x;
					msg.Lon = sensor.data.position_Global.y;
					msg.posx = sensor.data.position.y*0.01;
					msg.posy = sensor.data.position.x*0.01;
					msg.posz = -sensor.data.position.z*0.01;
					msg.numSV = sensor.inf.addition_inf[0];
					msg.fix = (sensor.inf.addition_inf[1]>1)?sensor.inf.addition_inf[1]:1;
					float eph = sensor.inf.addition_inf[4];
					float epv = sensor.inf.addition_inf[5];
					msg.eph = eph<65535 ? eph : 65535;
					msg.epv = epv<65535 ? epv : 65535;
					if( Lock_SDLog(SyncTIMEOUT) )
					{
						xMessageBufferSend( LogStreamBuffer, &msg, sizeof(msg), 0 );
						
						UnLock_SDLog();
						return true;
					}
				}
				else
				{	//S
					MsgS msg;
					msg.msg_type = LogMsg_PosSensor;
					msg.length = sizeof(msg);
					msg.SensorType = sensor.data.sensor_type;
					msg.DataType = sensor.data.sensor_DataType;
					msg.Time = TIME::get_System_Run_Time()*1e+4;
					msg.sensor = sensor.data.available ? (1<<7)|ind : ind;
					msg.frame = sensor.data.data_frame;
					msg.posx = sensor.data.position.y*0.01;
					msg.posy = sensor.data.position.x*0.01;
					msg.posz = -sensor.data.position.z*0.01;
					msg.eph = sensor.data.xy_trustD<65535 ? sensor.data.xy_trustD : 65535;
					msg.epv = sensor.data.z_trustD<65535 ? sensor.data.z_trustD : 65535;
					if( Lock_SDLog(SyncTIMEOUT) )
					{
						xMessageBufferSend( LogStreamBuffer, &msg, sizeof(msg), 0 );
						
						UnLock_SDLog();
						return true;
					}
				}
			}
			else if( dataType < 16 )
			{	//v传感器
				MsgV msg;
				msg.msg_type = LogMsg_PosSensor;
				msg.length = sizeof(msg);
				msg.SensorType = sensor.data.sensor_type;
				msg.DataType = sensor.data.sensor_DataType;
				msg.Time = TIME::get_System_Run_Time()*1e+4;
				msg.sensor = sensor.data.available ? (1<<7)|ind : ind;
				msg.frame = sensor.data.data_frame;
				msg.velx = sensor.data.velocity.y*0.01;
				msg.vely = sensor.data.velocity.x*0.01;
				msg.velz = -sensor.data.velocity.z*0.01;
				msg.eph = sensor.data.xy_trustD<65535 ? sensor.data.xy_trustD : 65535;
				msg.epv = sensor.data.z_trustD<65535 ? sensor.data.z_trustD : 65535;
				if( Lock_SDLog(SyncTIMEOUT) )
				{
					xMessageBufferSend( LogStreamBuffer, &msg, sizeof(msg), 0 );
					
					UnLock_SDLog();
					return true;
				}
			}
			else if( dataType < 24 )
			{	//sv传感器
				if( sensor.data.sensor_type == Position_Sensor_Type_GlobalPositioning )
				{	//GSV
					MsgGSV msg;
					msg.msg_type = LogMsg_PosSensor;
					msg.length = sizeof(msg);
					msg.SensorType = sensor.data.sensor_type;
					msg.DataType = sensor.data.sensor_DataType;
					msg.Time = TIME::get_System_Run_Time()*1e+4;
					msg.sensor = sensor.data.available ? (1<<7)|ind : ind;
					msg.frame = sensor.data.data_frame;
					msg.OriginLat = rad2degree(sensor.data.mp.lat0_rad);
					msg.OriginLon = rad2degree(sensor.data.mp.lon0_rad);
					msg.Lat = sensor.data.position_Global.x;
					msg.Lon = sensor.data.position_Global.y;
					msg.posx = sensor.data.position.y*0.01;
					msg.posy = sensor.data.position.x*0.01;
					msg.posz = -sensor.data.position.z*0.01;
					msg.velx = sensor.data.velocity.y*0.01;
					msg.vely = sensor.data.velocity.x*0.01;
					msg.velz = -sensor.data.velocity.z*0.01;
					msg.numSV = sensor.inf.addition_inf[0];
					msg.fix = (sensor.inf.addition_inf[1]>1)?sensor.inf.addition_inf[1]:1;
					float eph = sensor.inf.addition_inf[4];
					float epv = sensor.inf.addition_inf[5];
					msg.eph = eph<65535 ? eph : 65535;
					msg.epv = epv<65535 ? epv : 65535;
					if( Lock_SDLog(SyncTIMEOUT) )
					{
						xMessageBufferSend( LogStreamBuffer, &msg, sizeof(msg), 0 );
						
						UnLock_SDLog();
						return true;
					}
				}
				else
				{	//S
					MsgSV msg;
					msg.msg_type = LogMsg_PosSensor;
					msg.length = sizeof(msg);
					msg.SensorType = sensor.data.sensor_type;
					msg.DataType = sensor.data.sensor_DataType;
					msg.Time = TIME::get_System_Run_Time()*1e+4;
					msg.sensor = sensor.data.available ? (1<<7)|ind : ind;
					msg.frame = sensor.data.data_frame;
					msg.posx = sensor.data.position.y*0.01;
					msg.posy = sensor.data.position.x*0.01;
					msg.posz = -sensor.data.position.z*0.01;
					msg.velx = sensor.data.velocity.y*0.01;
					msg.vely = sensor.data.velocity.x*0.01;
					msg.velz = -sensor.data.velocity.z*0.01;
					msg.eph = sensor.data.xy_trustD<65535 ? sensor.data.xy_trustD : 65535;
					msg.epv = sensor.data.z_trustD<65535 ? sensor.data.z_trustD : 65535;
					if( Lock_SDLog(SyncTIMEOUT) )
					{
						xMessageBufferSend( LogStreamBuffer, &msg, sizeof(msg), 0 );
						
						UnLock_SDLog();
						return true;
					}
				}
			}
			
			return false;
		}
		bool SDLog_Msg_Attitude( double SyncTIMEOUT )
		{
			Quaternion airframe_quat;
			if( get_AirframeY_quat(&airframe_quat,SyncTIMEOUT) == false )
				return false;
			vector3<double> angular_rate;
			if( get_AngularRate_Ctrl(&angular_rate,SyncTIMEOUT) == false )
				return false;
			struct Msg
			{
				uint8_t msg_type;
				uint8_t length;
				uint16_t rsv;
				uint32_t Time;			
				double roll;
				double pitch;
				double yaw;
				double rollspeed;
				double pitchspeed;
				double yawspeed;
			}__attribute__((__packed__));
			Msg msg;
			
			msg.msg_type = LogMsg_Attitude;
			msg.length = sizeof(msg);
			msg.Time = TIME::get_System_Run_Time()*1e+4;			
			msg.roll = airframe_quat.getRoll();
			msg.pitch = airframe_quat.getPitch();
			msg.yaw = airframe_quat.getYaw();
			msg.rollspeed = angular_rate.x;
			msg.pitchspeed = angular_rate.y;
			msg.yawspeed = angular_rate.z;
			if( Lock_SDLog(SyncTIMEOUT) )
			{
				xMessageBufferSend( LogStreamBuffer, &msg, sizeof(msg), 0 );
				
				UnLock_SDLog();
				return true;
			}
			return false;
		}
		bool SDLog_Msg_AttitudeQuaternion( double SyncTIMEOUT )
		{
			Quaternion airframe_quat;
			if( get_AirframeY_quat(&airframe_quat,SyncTIMEOUT) == false )
				return false;
			vector3<double> angular_rate;
			if( get_AngularRate_Ctrl(&angular_rate,SyncTIMEOUT) == false )
				return false;
			struct Msg
			{
				uint8_t msg_type;
				uint8_t length;
				uint16_t rsv;
				uint32_t Time;
				double q0;
				double q1;
				double q2;
				double q3;
				double rollspeed;
				double pitchspeed;
				double yawspeed;
			}__attribute__((__packed__));
			Msg msg;
			
			msg.msg_type = LogMsg_AttitudeQuaternion;
			msg.length = sizeof(msg);
			msg.Time = TIME::get_System_Run_Time()*1e+4;			
			msg.q0 = airframe_quat.get_qw();
			msg.q1 = airframe_quat.get_qx();
			msg.q2 = airframe_quat.get_qy();
			msg.q3 = airframe_quat.get_qz();
			msg.rollspeed = angular_rate.x;
			msg.pitchspeed = angular_rate.y;
			msg.yawspeed = angular_rate.z;
			if( Lock_SDLog(SyncTIMEOUT) )
			{
				xMessageBufferSend( LogStreamBuffer, &msg, sizeof(msg), 0 );
				
				UnLock_SDLog();
				return true;
			}
			return false;
		}
		bool SDLog_Msg_LocalPositionNed( double SyncTIMEOUT )
		{
			vector3<double> pos;
			vector3<double> vel;
			vector3<double> acc;
			if( get_Position_Ctrl( &pos, SyncTIMEOUT ) == false )
				return false;
			if( get_VelocityENU_Ctrl( &vel, SyncTIMEOUT ) == false )
				return false;
			if( get_AccelerationENU_Ctrl( &acc, SyncTIMEOUT ) == false )
				return false;
			struct Msg
			{
				uint8_t msg_type;
				uint8_t length;
				int8_t XYSensor;
				int8_t ZSensor;
				uint32_t Time;			
				double posx;
				double posy;
				double posz;
				double xspeed;
				double yspeed;
				double zspeed;
				double accx;
				double accy;
				double accz;
			}__attribute__((__packed__));
			Msg msg;
			
			msg.msg_type = LogMsg_LocalPositionNed;
			msg.length = sizeof(msg);
			msg.XYSensor = get_Current_XYSensor();
			msg.ZSensor = get_Current_ZSensor();
			msg.Time = TIME::get_System_Run_Time()*1e+4;		
			msg.posx = pos.y * 0.01;
			msg.posy = pos.x * 0.01;
			msg.posz = -pos.z * 0.01;
			msg.xspeed = vel.y * 0.01;
			msg.yspeed = vel.x * 0.01;
			msg.zspeed = -vel.z * 0.01;
			msg.accx = acc.y * 0.01;
			msg.accy = acc.x * 0.01;
			msg.accz = -acc.z * 0.01;
			if( Lock_SDLog(SyncTIMEOUT) )
			{
				xMessageBufferSend( LogStreamBuffer, &msg, sizeof(msg), 0 );
				
				UnLock_SDLog();
				return true;
			}
			return false;
		}
		bool SDLog_Msg_DebugVect( const char* name, double vect[], uint8_t length, double SyncTIMEOUT )
		{
			if( length > 20 )
				return false;
			struct Msg
			{
				uint8_t msg_type;
				uint8_t length;
				uint16_t rsv;
				uint32_t Time;
				char name[10];
			}__attribute__((__packed__));

			uint8_t* buf = new uint8_t[sizeof(Msg)+length*8];
			Msg* msg = (Msg*)buf;
			msg->msg_type = LogMsg_DebugVect;
			msg->length = sizeof(Msg)+length*8;
			msg->Time = TIME::get_System_Run_Time()*1e+4;
			for(uint8_t i = 0; i < 10 ; ++i )
			{
				msg->name[i] = name[i];
				if( name[i] == 0 )
					break;
			}
			memcpy( &buf[sizeof(Msg)], vect, length*8 );
			
			if( Lock_SDLog(SyncTIMEOUT) )
			{
				xMessageBufferSend( LogStreamBuffer, buf, msg->length, 0 );
								
				UnLock_SDLog();
				delete[] buf;
				return true;
			}
			delete[] buf;
			return false;
		}
	/*Log数据*/
/*SD Log*/

/*Txt1 Log*/
	//Log缓冲区
	#define Txt1BufSize 4096
	Static_DTCMBuf uint8_t Txt1LogStreamBufferStorage[ Txt1BufSize+1 ];
	Static_DTCMBuf StaticMessageBuffer_t Txt1LogStreamBufferStruct;
	Static_DTCMBuf MessageBufferHandle_t Txt1LogStreamBuffer = xMessageBufferCreateStatic( Txt1BufSize,
                                                 Txt1LogStreamBufferStorage,
                                                 &Txt1LogStreamBufferStruct );;
	static SemaphoreHandle_t Txt1LogSemphr = xSemaphoreCreateMutex();
	static bool Lock_Txt1Log( double TIMEOUT = -1 )
	{
		if(get_is_usb_connected())
			return false;
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake( Txt1LogSemphr , TIMEOUT_Ticks ) )
			return true;
		return false;
	}
	static void UnLock_Txt1Log()
	{
		xSemaphoreGive(Txt1LogSemphr);
	}
	
	bool SDLog_Txt1( const char* txt, uint16_t length, double SyncTIMEOUT )
	{
		if( Lock_Txt1Log(SyncTIMEOUT) )
		{
			xMessageBufferSend( Txt1LogStreamBuffer, txt, length, 0 );
							
			UnLock_Txt1Log();
			return true;
		}
		return false;
	}
/*Txt1 Log*/


/*ubx文件记录 Log*/
	//ubx文件记录缓冲区
	#define UbxBufSize 84*1024
	__attribute__ ((aligned (4))) Static_SRAM2Buf uint8_t UbxLogStreamBufferStorage[ UbxBufSize+1 ];
	Static_DTCMBuf StaticMessageBuffer_t UbxLogStreamBufferStruct;
	Static_DTCMBuf MessageBufferHandle_t UbxLogStreamBuffer = xMessageBufferCreateStatic( UbxBufSize,
                                                 UbxLogStreamBufferStorage,
                                                 &UbxLogStreamBufferStruct );;
	static SemaphoreHandle_t UbxLogSemphr = xSemaphoreCreateMutex();
	static bool Lock_UbxLog( double TIMEOUT = -1 )
	{
		if(get_is_usb_connected())
			return false;
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake( UbxLogSemphr , TIMEOUT_Ticks ) )
			return true;
		return false;
	}
	static void UnLock_UbxLog()
	{
		xSemaphoreGive(UbxLogSemphr);
	}
	
	bool SDLog_Ubx( const char* txt, uint16_t length, double SyncTIMEOUT )
	{
		if( Lock_UbxLog(SyncTIMEOUT) )
		{
			xMessageBufferSend( UbxLogStreamBuffer, txt, length, 0 );
							
			UnLock_UbxLog();
			return true;
		}
		return false;
	}
/*ubx文件记录 Log*/

	
/*sd卡清理*/
	static inline bool calcDays( const char* date1, const RTC_TimeStruct& date2, int32_t* days )
	{
		uint32_t dateStrLen = strlen(date1);
		if( dateStrLen != 8 )
			return false;
		for( uint8_t i=0; i<dateStrLen; ++i )
		{
			if( !is_number(date1[i]) )
				return false;
		}
		int32_t date1Year = Ascii2num(date1[0])*1e3 + Ascii2num(date1[1])*1e2 + Ascii2num(date1[2])*1e1 + Ascii2num(date1[3]);
		int32_t date1Month = Ascii2num(date1[4])*1e1 + Ascii2num(date1[5]);
		if( date1Month<1 || date1Month>12 )
			return false;
		int32_t date1Day = Ascii2num(date1[6])*1e1 + Ascii2num(date1[7]);
		if( date1Day<1 || date1Day>32 )
			return false;
		
		*days = ((int32_t)date2.Year - date1Year)*365 +
						((int32_t)date2.Month - date1Month)*30 +
						((int32_t)date2.Date - date1Day);
		return true;
	}
	static inline bool deleteLogDir( char* path, uint16_t* dirsDeleted=0, uint32_t* filesDeleted=0, bool* dirChanged=0 )
	{
		FRESULT res;
		DIR dir;

		res = f_opendir(&dir, path);
		uint32_t pathLen = strlen(path);
		bool fDeleted = false;
		if (res == FR_OK) 
		{	//打开文件夹
			for (;;) 
			{	//遍历文件夹内容
				FILINFO fno;
				res = f_readdir(&dir, &fno);
				if (res != FR_OK || fno.fname[0] == 0)
					break;  /* Break on error or end of dir */
				if (fno.fattrib & AM_DIR) 
				{	//文件夹
					
				} 
				else
				{	//文件
					sprintf(&path[pathLen], "/%s", fno.fname);
					res = f_unlink(path);
					if( res==FR_OK )
					{
						fDeleted = true;
						if(filesDeleted)
							++(*filesDeleted);
					}
				}
			}
			f_closedir(&dir);
			
			//删除当前文件夹
			path[pathLen] = 0;
			res = f_unlink(path);
			bool dirC = res==FR_OK || fDeleted;
			if( dirsDeleted && dirC )
				++(*dirsDeleted);
			if( dirChanged )
				(*dirChanged) = dirC;
			return true;
		}
		return false;
	}
	static inline void cleanupOutdatedLogs( int16_t daysBefore )
	{
		//获取今天日期
		RTC_TimeStruct RTC_Time;
		RTC_Time = Get_RTC_Time();
		
		FRESULT res;
		DIR dir;

		//统计删除文件
		uint32_t filesDeleted = 0;
		uint16_t dirsDeleted = 0;
		
		char path[60];
		path[0] = 0;
		strcat( path, SDPath );
		strcat( path, "ACFly/Log" );
		uint32_t pathLen = strlen(path);
		res = f_opendir(&dir, path);
		if (res == FR_OK) 
		{	//打开文件夹
			
			//打开log文件
			bool logFilOpened = false;
			FIL logFil;
			char filename[30];
			char logTxt[100];
			filename[0] = 0;
			strcat( filename, SDPath );
			strcat( filename, "ACFly/sysLog.txt" );
			res = f_open( &logFil, filename, FA_OPEN_APPEND | FA_WRITE );
			if( res == FR_OK )
			{
				logFilOpened = true;
				
				sprintf( logTxt, "\r\nSD Cleaner\r\n%d/%02d/%02d-%02d:%02d:%02d\r\n", 
					RTC_Time.Year, RTC_Time.Month, RTC_Time.Date,
					RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds );
				f_puts( logTxt, &logFil );
				
				sprintf( logTxt, "Total Space:%.1fGB\r\nFree Space:%.1fGB\r\n", 
					totGB, 
					freGB );
				f_puts( logTxt, &logFil );
			}
			
			for (;;) 
			{	//遍历文件夹内容
				FILINFO fno;
				res = f_readdir(&dir, &fno);
				if (res != FR_OK || fno.fname[0] == 0)
					break;  /* Break on error or end of dir */
				if (fno.fattrib & AM_DIR) 
				{	//文件夹
					int32_t days;
					if( calcDays( fno.fname, RTC_Time, &days ) )
					{	//获取记录天数
						if( daysBefore<0 || days>daysBefore )
						{	//日期较早
							//删除此记录
							sprintf(&path[pathLen], "/%s", fno.fname);
							bool dirChanged = false;
							deleteLogDir(path, &dirsDeleted, &filesDeleted, &dirChanged);
							
							if( logFilOpened && dirChanged )
							{	//记录删除的文件夹
								sprintf( logTxt, "\tDelete dir: %s\r\n", path );
								f_puts( logTxt, &logFil );
							}
						}
					}
				} 
				else
				{	//文件
					
				}
			}
			f_closedir(&dir);
			
			//计算容量
			FATFS *fs;
			DWORD fre_clust;
			/* Get volume information and free clusters of drive 1 */
			res = f_getfree((TCHAR const*)SDPath, &fre_clust, &fs);
			/* Get total sectors and free sectors */
			fre_size = (uint64_t)fre_clust * (uint64_t)fs->csize * (uint64_t)512;
			freGB = (fre_size>>20) / 1024.0f;
			
			if( logFilOpened )
			{	//记录删除的文件夹
				sprintf( logTxt, "%d Files and %d Directories deleted.\r\nTotal Space:%.1fGB\r\nFree Space:%.1fGB\r\nSD Cleanup Completed.\r\n", 
					filesDeleted,
					dirsDeleted,
					totGB, 
					freGB );
				f_puts( logTxt, &logFil );
				f_close(&logFil);
			}
		}
	}
	
	int16_t sdCeanupCmd = -2;
	void cleanupSD( int16_t monthBefore )
	{
		if( monthBefore < 0 )
			monthBefore = -1;
		sdCeanupCmd = monthBefore;
	}
/*sd卡清理*/
	
/*SD卡更新参数*/
	//最大行字节数
	#define maxLineLen 100
	//最大参数名字节数
	#define maxParamNameLen 16
	
	//ASCALL码转char
	static inline char ConvertHexChar(char ch)
	{
		if((ch >= '0') && (ch <= '9'))
			return (ch-'0');
		else if((ch >= 'A') && (ch <= 'F'))
			return ((ch-'A')+10);
		else if((ch >= 'a') && (ch <= 'f'))
			return ((ch-'a')+10);
		else return (-1);
	}
	static inline bool runParamCheck( FIL& fil, bool doUpdateParam, bool& logFilOpened, FIL& logFil, char* logTxt, uint16_t* updatedParamsCount=0 )
	{
		uint16_t lineNumber = 0;
		while(1)
		{	
			//读取一行字符
			char txt[maxLineLen];
			txt[maxLineLen-1] = 0;
			char* readRes = f_gets( txt, maxLineLen, &fil );
			if( readRes == txt )
			{	//读取成功
				
				++lineNumber;
				//获取行长度
				size_t lineSize = strlen(txt);
				//行结束符必须为\n
				if( txt[lineSize-1]!='\n' && !f_eof(&fil) )
				{
					if( logFilOpened )
					{
						sprintf( logTxt, "\tParam file data error. No CRLF detected or line too long.\tLine: %d\r\n", lineNumber );
						f_puts( logTxt, &logFil );
					}
					return false;
				}
				//注释符直接开始读下一行
				if( txt[0] == '#' )
					continue;
				//校验和
				uint8_t checkSum = 0;
				
				//跳过空格
				uint8_t paramName_start = 0;
				for( uint8_t i=0; i<lineSize; ++i )
				{
					if( txt[i]==' ' || txt[i]=='\t' )
					{	//空格
						//跳过
					}
					else if( txt[i]=='\r' || txt[i]=='\n' )
						//换行符读下一行
						goto CONTINE_READLINE;
					else
					{
						paramName_start = i;
						break;
					}
				}
				
				//获取参数名称
				uint8_t paramName_len = 0;
				for( uint8_t i=paramName_start; i<lineSize; ++i )
				{
					if( txt[i]==',' || txt[i]==' ' || txt[i]=='\t' )
					{	//参数分隔符
						txt[i] = 0;
						if( paramName_len==0 || paramName_len>maxParamNameLen )
						{	//参数名称不正确
							if( logFilOpened )
							{
								sprintf( logTxt, "\tParam name error. Param name empty or too long.\tLine: %d\r\n", lineNumber );
								f_puts( logTxt, &logFil );
							}
							return false;
						}
						break;
					}
					else if( txt[i]>=33 && txt[i]<=126 )
					{	//参数字符
						checkSum += txt[i];
						++paramName_len;
					}
					else
					{	//参数名含有不合法字符
						if( logFilOpened )
						{
							sprintf( logTxt, "\tParam name error. Param name contains illegal charactor.\tLine: %d\r\n", lineNumber );
							f_puts( logTxt, &logFil );
						}
						return false;
					}
				}
				
				//获取参数数值
				uint8_t paramValue_start = paramName_start+paramName_len+1;
				uint8_t paramValue_len = 0;
				float paramValue_f = std::nanf("");
				for( uint8_t i=paramValue_start; i<lineSize; ++i )
				{
					if( txt[i]==',' || txt[i]==' ' || txt[i]=='\t' )
					{	//参数分隔符
						txt[i] = 0;
						break;
					}
					else if( is_number(txt[i]) || txt[i]=='.' || txt[i]=='-' || txt[i]=='+' )
					{	//参数数值字符
						checkSum += txt[i];
						++paramValue_len;
					}
					else
					{	//参数数值含有不合法字符
						if( logFilOpened )
						{
							sprintf( logTxt, "\tParam value error. Param value contains illegal charactor.\tLine: %d\r\n", lineNumber );
							f_puts( logTxt, &logFil );
						}
						return false;
					}
				}
				paramValue_f = atof(&txt[paramValue_start]);
				if( !isvalid(paramValue_f) )
				{
					if( logFilOpened )
					{
						sprintf( logTxt, "\tParam value error. Param value data illegal.\tLine: %d\r\n", lineNumber );
						f_puts( logTxt, &logFil );
					}
					//参数数值错误
					return false;
				}
				
				//获取参数类型
				uint8_t paramType_start = paramValue_start+paramValue_len+1;
				uint8_t paramType_len = 0;
				uint8_t paramType = 0;
				for( uint8_t i=paramType_start; i<lineSize; ++i )
				{
					if( txt[i]==',' || txt[i]==' ' || txt[i]=='\t' )
					{	//参数分隔符
						txt[i] = 0;
						break;
					}
					else if( is_number(txt[i]) )
					{	//参数类别字符
						checkSum += txt[i];
						++paramType_len;
					}
					else
					{	//参数类别含有不合法字符
						if( logFilOpened )
						{
							sprintf( logTxt, "\tParam type error. Param type contains illegal charactor.\tLine: %d\r\n", lineNumber );
							f_puts( logTxt, &logFil );
						}
						return false;
					}
				}
				if( paramType_len==0 || paramType_len>2 )
				{
					if( logFilOpened )
					{
						sprintf( logTxt, "\tParam type error. Param type data illegal.\tLine: %d\r\n", lineNumber );
						f_puts( logTxt, &logFil );
					}
					//参数类别错误
					return false;
				}
				paramType = atoi(&txt[paramType_start]);
				if( paramType==0 || paramType>10 )
				{
					if( logFilOpened )
					{
						sprintf( logTxt, "\tParam type error. Param type data illegal.\tLine: %d\r\n", lineNumber );
						f_puts( logTxt, &logFil );
					}
					//参数类别错误
					return false;
				}
				
				//获取校验
				uint8_t paramChecksum_start = paramType_start+paramType_len+1;
				uint8_t paramChecksum_len = 0;
				uint8_t paramChecksum = 0;
				for( uint8_t i=paramChecksum_start; i<lineSize; ++i )
				{
					if( txt[i]==',' || txt[i]==' ' || txt[i]=='\t' || txt[i]=='\r' )
					{	//参数分隔符
						txt[i] = 0;
						break;
					}
					else if( is_number(txt[i]) || is_lowercase_letter(txt[i]) || is_capital_letter(txt[i]) )
					{	//参数校验字符
						++paramChecksum_len;
					}
					else
					{	//参数校验含有不合法字符
						if( logFilOpened )
						{
							sprintf( logTxt, "\tParam checksum error. Param checksum contains illegal charactor.\tLine: %d\r\n", lineNumber );
							f_puts( logTxt, &logFil );
						}
						return false;
					}
				}
				if( paramChecksum_len==0 || paramChecksum_len>2 )
				{
					if( logFilOpened )
					{
						sprintf( logTxt, "\tParam checksum error. Param checksum data illegal.\tLine: %d\r\n", lineNumber );
						f_puts( logTxt, &logFil );
					}
					//参数校验错误
					return false;
				}
				uint8_t rdChecksum = 0;
				for( uint8_t i=paramChecksum_start; i<paramChecksum_start+paramChecksum_len; ++i )
				{	//求校验和
					rdChecksum = (rdChecksum<<4) | ConvertHexChar(txt[i]);
				}
				if( checkSum != rdChecksum )
				{
					if( logFilOpened )
					{
						sprintf( logTxt, "\tParam checksum error. Param checksum error.\tLine: %d\r\n", lineNumber );
						f_puts( logTxt, &logFil );
					}
					//参数校验错误
					return false;
				}
				else
				{	//校验成功
					SName param_name(&txt[paramName_start]);
					MAV_PARAM_TYPE param_type;
					__attribute__ ((aligned(8))) uint64_t paramValue_o;
					if( ReadParam( param_name, 0, &param_type, &paramValue_o, 0 )==PR_OK )
					{	//读取参数成功
						if( param_type != paramType )
						{
							if( logFilOpened )
							{
								sprintf( logTxt, "\tParam type error. Param type doesn't match.\tLine: %d\tParam name:%s\r\n", lineNumber, &txt[paramName_start] );
								f_puts( logTxt, &logFil );
							}
							//参数类型不符合
							return false;
						}
						if( doUpdateParam )
						{	//需要更新参数
							uint64_t paramValue;
							float paramValue_of = std::nanf("");
							switch( param_type )
							{
								case MAV_PARAM_TYPE_INT8:
									paramValue = (int8_t)paramValue_f;
									paramValue_of = *(int8_t*)&paramValue_o;
									break;
								case MAV_PARAM_TYPE_UINT8:
									paramValue = (uint8_t)paramValue_f;
									paramValue_of = *(uint8_t*)&paramValue_o;;
									break;
								case MAV_PARAM_TYPE_INT16:
									paramValue = (int16_t)paramValue_f;
									paramValue_of = *(int16_t*)&paramValue_o;
									break;
								case MAV_PARAM_TYPE_UINT16:
									paramValue = (uint16_t)paramValue_f;
									paramValue_of = *(uint16_t*)&paramValue_o;
									break;
								case MAV_PARAM_TYPE_INT32:
									paramValue = (int32_t)paramValue_f;
									paramValue_of = *(int32_t*)&paramValue_o;
									break;
								case MAV_PARAM_TYPE_UINT32:
									paramValue = (uint32_t)paramValue_f;
									paramValue_of = *(uint32_t*)&paramValue_o;
									break;
								case MAV_PARAM_TYPE_REAL32:
									paramValue = *(uint32_t*)&paramValue_f;
									paramValue_of = *(float*)&paramValue_o;
									break;
								default:
									continue;
									break;
							}
							if( isvalid(paramValue_of) )
							{
								char valueStr1[30];
								char valueStr2[30];
								sprintf( valueStr1, "%.3f", paramValue_f );
								sprintf( valueStr2, "%.3f", paramValue_of );
								if( strcmp( valueStr1, valueStr2 ) != 0 )
								{	//参数不一致需要保存
									if(strcmp(&txt[paramName_start],"Init_Boot_Count") && strcmp(&txt[paramName_start],"Init_Firmware_V"))
									{
										UpdateParam( param_name, paramValue );
										if( updatedParamsCount )
											++(*updatedParamsCount);
									}
								}
							}
						}
					}
					else
					{
						if( logFilOpened )
						{
							sprintf( logTxt, "\tParam name error. No parameter named \"%s\" in the controller.\tLine: %d\r\n", &txt[paramName_start], lineNumber );
							f_puts( logTxt, &logFil );
						}
						//无此参数
						return false;
					}
				}
			}
			else
				//读取结束
				break;
			
CONTINE_READLINE:
			continue;
		}
		return true;
	}
	/*用sd卡参数文件更新参数
	*/
	enum UDPARAM_RS
	{
		UDPARAM_RS_NOFILE = 0,
		UDPARAM_RS_SUCCESS = 1,
		UDPARAM_RS_FAIL = -1,
	};
	static inline UDPARAM_RS updateParams()
	{
		//执行结果
		UDPARAM_RS res = UDPARAM_RS_NOFILE;
		//文件指针
		DIR dir;
		FIL fil;
		char filename[50];
		//文件操作状态
		FRESULT fres;
		//文件（夹）状态
		FILINFO finfo;
		//是否有ACFly目录
		filename[0] = 0;
		strcat( filename, SDPath );
		strcat( filename,  "ACFly" );
		fres = f_stat( filename, &finfo );
		if( fres == FR_OK )
		{	//打开ACFly目录
			fres = f_opendir(&dir,filename);
			if( fres == FR_OK )
			{	//查找acParams后缀文件			
				fres = f_findfirst(&dir, &finfo, filename, "*.acParams"); 				
				if(fres == FR_OK && finfo.fname[0])
				{//acParams后缀文件存在					
					strcat( filename, "/" );
					strcat( filename, finfo.fname );
					f_chmod( filename, 0, AM_RDO );
					if(!f_open( &fil, filename, FA_OPEN_EXISTING | FA_READ ))
					{	//文件打开成功
						res = UDPARAM_RS_FAIL;
						
						//打开log文件
						bool logFilOpened = false;
						FIL logFil;
						char log_filename[30];
						char logTxt[150];
						log_filename[0] = 0;
						strcat( log_filename, SDPath );
						strcat( log_filename, "ACFly/sysLog.txt" );
						fres = f_open( &logFil, log_filename, FA_OPEN_APPEND | FA_WRITE );
						if( fres == FR_OK )
						{	//log打开成功
							logFilOpened = true;
							
							RTC_TimeStruct RTC_Time;
							RTC_Time = Get_RTC_Time();
							sprintf( logTxt, "\r\nParam Updater\r\n%d/%02d/%02d-%02d:%02d:%02d\r\n", 
								RTC_Time.Year, RTC_Time.Month, RTC_Time.Date,
								RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds );
							f_puts( logTxt, &logFil );
							
							sprintf( logTxt, "\tParam file detected: %s\r\n", filename );
							f_puts( logTxt, &logFil );
						}
						
						uint16_t updatedParamsCount = 0;
						//跳过文件头BOM
						uint8_t BOMlen = 0;
						while(1)
						{
							char txt;
							UINT bytes_read = 0;
							fres = f_read( &fil, &txt, 1, &bytes_read );
							if( fres!=FR_OK || bytes_read!=1 )
							{
								if( logFilOpened )
								{
									sprintf( logTxt, "\tParam file empty or open failed!\r\n" );
									f_puts( logTxt, &logFil );
								}
								//读取文件错误
								goto READFIN;
							}
							
							if( txt > 0 )
							{	//读取到字符
								break;
							}
							else
							{	//读取到非字符
								++BOMlen;
								if( BOMlen > 5 )
								{
									if( logFilOpened )
									{
										sprintf( logTxt, "\tParam file header data check failed!\r\n" );
										f_puts( logTxt, &logFil );
									}
									//文件开头格式错误
									goto READFIN;
								}
							}
						}
						
						//校验参数
						f_lseek( &fil, BOMlen );
						if( runParamCheck(fil, false, logFilOpened, logFil, logTxt) == false )
						{
							if( logFilOpened )
							{
								sprintf( logTxt, "Param update failed.\r\n" );
								f_puts( logTxt, &logFil );
							}
							//参数文件校验失败
							goto READFIN;
						}
						
						//更新参数
						f_lseek( &fil, BOMlen );
						if( runParamCheck(fil, true, logFilOpened, logFil, logTxt, &updatedParamsCount) )
						{
							if( logFilOpened )
							{
								sprintf( logTxt, "%d parameters have been updated.\r\nParam update success.\r\n", updatedParamsCount );
								f_puts( logTxt, &logFil );
							}
							res = UDPARAM_RS_SUCCESS;
						}
						else
						{
							if( logFilOpened )
							{
								sprintf( logTxt, "Param update failed during the writing process.\r\n%d parameters have been updated.\r\n", updatedParamsCount );
								f_puts( logTxt, &logFil );
								sprintf( logTxt, "Please check your SD card and re-update parameters!\r\n" );
								f_puts( logTxt, &logFil );
							}
						}
						
READFIN:
						//关闭文件
						f_close(&fil);
						//删除文件
						f_unlink(filename);
						//log
						if( logFilOpened )
							f_close(&logFil);
					}
				}
				//关闭文件夹
				f_closedir(&dir);
			}
		}
		return res;
	}
	
/*SD卡更新参数*/	
	
static void SDS_Task(void* pvParameters)
{
reload_SD:
	//等待SD卡插入卡槽
	mscWritten = false;
	Clear_SD_Init_Complete();
	do
	{
		os_delay(2.0);
	}while( BSP_SD_IsDetected() == false );
	
	if(Lock_SD(-1))
	{		
		if( BSP_SD_Init() != MSD_OK )
		{
			UnLock_SD();
			goto reload_SD;
		}
		SD_Driver.disk_initialize(0);	
		UnLock_SD();
	}
	
	if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 1) == FR_OK)
	{		
		//文件指针
		static FIL SDFile;
		static FIL Txt1File;	bool Txt1FileCreated = false;
		static FIL UbxFile;	bool UbxFileCreated = false;
		//文件操作状态
		FRESULT fres;
		//文件（夹）状态
		FILINFO finfo;
		//文件（夹）名
		char filename[60];
		uint8_t filename_len;
		//写文件临时变量
		__attribute__ ((aligned (4))) Static_AXIDMABuf char buf[BufSize];
		UINT length; 
		
		//计算容量
		FATFS *fs;
		DWORD fre_clust;
		/* Get volume information and free clusters of drive 1 */
    fres = f_getfree((TCHAR const*)SDPath, &fre_clust, &fs);
    /* Get total sectors and free sectors */
    tot_size = (uint64_t)((fs->n_fatent - 2) * fs->csize) * (uint64_t)512;
    fre_size = (uint64_t)fre_clust * (uint64_t)fs->csize * (uint64_t)512;
		totGB = (tot_size>>20) / 1024.0f;
		freGB = (fre_size>>20) / 1024.0f;
		
		if( freGB < 1 )
		{	//自动清理SD卡
			int16_t daysBefore = 12*30;
			do
			{
				cleanupOutdatedLogs(daysBefore);
				
				/* Get volume information and free clusters of drive 1 */
				fres = f_getfree((TCHAR const*)SDPath, &fre_clust, &fs);
				/* Get total sectors and free sectors */
				tot_size = (uint64_t)((fs->n_fatent - 2) * fs->csize) * (uint64_t)512;
				fre_size = (uint64_t)fre_clust * (uint64_t)fs->csize * (uint64_t)512;
				totGB = (tot_size>>20) / 1024.0f;
				freGB = (fre_size>>20) / 1024.0f;
				
				daysBefore -= 2*30;
				if( daysBefore < 3 )
				{
					daysBefore = 3;
					break;
				}
			}while(freGB<1);
			
			if( freGB < 1 )
			{	//最后尝试清理三天前所有log
				cleanupOutdatedLogs(3);
				
				/* Get volume information and free clusters of drive 1 */
				fres = f_getfree((TCHAR const*)SDPath, &fre_clust, &fs);
				/* Get total sectors and free sectors */
				tot_size = (uint64_t)((fs->n_fatent - 2) * fs->csize) * (uint64_t)512;
				fre_size = (uint64_t)fre_clust * (uint64_t)fs->csize * (uint64_t)512;
				totGB = (tot_size>>20) / 1024.0f;
				freGB = (fre_size>>20) / 1024.0f;
			}
		}
		
		//更新参数
		bool attEna;
		is_Attitude_Control_Enabled(&attEna);
		if(!attEna)
		{
			//等待初始化完成
			while( getInitializationCompleted() == false )
				os_delay(0.01);
			//更新参数
			UDPARAM_RS res = updateParams();
			if( res == UDPARAM_RS_SUCCESS )
				sendLedSignal(LEDSignal_Success1);
			else if( res == UDPARAM_RS_FAIL )
				sendLedSignal(LEDSignal_Err1);
		}
		
		//创建ACFly目录
		filename[0] = 0;
		strcat( filename, SDPath );
		strcat( filename, "ACFly" );
		fres = f_stat( filename, &finfo );
		if( fres!=FR_NO_FILE && (finfo.fattrib&AM_DIR)==0 )
		{	//删除同名文件
			if( (finfo.fattrib&AM_RDO)!=0 )
				f_chmod( filename, 0, AM_RDO );
			f_unlink(filename);		
		}
		fres = f_mkdir(filename);
		if( fres!=FR_OK && fres!=FR_EXIST )
			goto sdErr;
		
		//创建BootLoaderUpdate目录
		filename[0] = 0;
		strcat( filename, SDPath );
		strcat( filename, "ACFly/BootLoaderUpdate" );
		fres = f_stat( filename, &finfo );
		if( fres!=FR_NO_FILE && (finfo.fattrib&AM_DIR)==0 )
		{	//删除同名文件
			if( (finfo.fattrib&AM_RDO)!=0 )
				f_chmod( filename, 0, AM_RDO );
			f_unlink(filename);		
		}
		fres = f_mkdir(filename);
		if( fres!=FR_OK && fres!=FR_EXIST )
			goto sdErr;
		
		//创建log目录
		filename[0] = 0;
		strcat( filename, SDPath );
		strcat( filename, "ACFly/Log" );
		fres = f_stat( filename, &finfo );
		if( fres!=FR_NO_FILE && (finfo.fattrib&AM_DIR)==0 )
		{	//删除同名文件
			if( (finfo.fattrib&AM_RDO)!=0 )
				f_chmod( filename, 0, AM_RDO );
			f_unlink(filename);		
		}
		fres = f_mkdir(filename);
		if( fres!=FR_OK && fres!=FR_EXIST )
			goto sdErr;
		
		//创建log日期目录
		RTC_TimeStruct RTC_Time;
		RTC_Time = Get_RTC_Time();
		filename_len = strlen(filename);
		sprintf( &filename[filename_len], "/%d%02d%02d" , RTC_Time.Year, RTC_Time.Month, RTC_Time.Date );
		fres = f_stat( filename, &finfo );
		if( fres!=FR_NO_FILE && (finfo.fattrib&AM_DIR)==0 )
		{	//删除同名文件
			if( (finfo.fattrib&AM_RDO)!=0 )
				f_chmod( filename, 0, AM_RDO );
			f_unlink(filename);
		}
		fres = f_mkdir(filename);
		if( fres!=FR_OK && fres!=FR_EXIST )
			goto sdErr;
		
		//创建Log文件
		filename_len = strlen(filename);
		sprintf( &filename[filename_len], "/%d%02d%02d%02d%02d.aclog" , RTC_Time.Year, RTC_Time.Month, RTC_Time.Date , RTC_Time.Hours, RTC_Time.Minutes );
		fres = f_stat( filename, &finfo );
		uint32_t ind = 1;
		while( fres!=FR_NO_FILE )
		{	//删除同名文件
			if( fres != FR_OK )
				goto sdErr;
			sprintf( &filename[filename_len], "/%d%02d%02d%02d%02d_%d.aclog" , RTC_Time.Year, RTC_Time.Month, RTC_Time.Date , RTC_Time.Hours, RTC_Time.Minutes, ind );
			fres = f_stat( filename, &finfo );
			++ind;
		}
		fres = f_open(&SDFile, filename, FA_CREATE_ALWAYS | FA_WRITE);
		//写入数据头和版本信息
		buf[0] = 'A';	buf[1] = 'C'; buf[2] = 0; buf[3] = (1<<1) | 1;
		f_write( &SDFile, buf, 4, &length );
		//写描述字符
		sprintf( buf, "ACFlyA9 %d%02d%02d%02d%02d", RTC_Time.Year, RTC_Time.Month, RTC_Time.Date, RTC_Time.Hours, RTC_Time.Minutes );
		f_write( &SDFile, buf, 24, &length );
		f_close(&SDFile);
		char LogFilename[45];
		strcpy( LogFilename, filename );
		fres = f_open(&SDFile, LogFilename, FA_OPEN_APPEND | FA_WRITE);
		
		//Txt1文件名
		char Txt1Filename[45];
		strcpy( Txt1Filename, filename );
		filename_len = strlen(filename);
		sprintf( &Txt1Filename[filename_len-6], "_1.txt" );
		
		//Ubx文件名
		char UbxFilename[45];
		strcpy( UbxFilename, filename );
		filename_len = strlen(filename);
		sprintf( &UbxFilename[filename_len-6], ".ubx" );		
		
		//完成sd初始化
		Set_SD_Init_Complete();
		Set_SD_Init_Success();
		
		bool last_usb_connected = false;
		bool last_inFlight;
		get_is_inFlight(&last_inFlight);
		TickType_t xLastWakeTime = xTaskGetTickCount();
		TIME last_flush_TIME;
		bool SDLog_wait_to_sync = false;
		bool Txt1_wait_to_sync = false;
		bool Ubx_wait_to_sync = false;
		uint8_t sync_ind = 0;
		while(1)
		{
			bool usb_connected = get_is_usb_connected();
			
			if( usb_connected == false )
			{
				//电脑修改过sd卡
				//重新挂载
				if( mscWritten )
					goto reload_SD;
			}
			
			if( sdCeanupCmd != -2 )
			{	//清理SD卡
				if( usb_connected == false )
					cleanupOutdatedLogs(sdCeanupCmd);
				sdCeanupCmd = -2;
			}
			
			if( usb_connected == false )
			{
				/*写入Log*/
					//从缓冲区获取要写入SD卡的数据
					uint32_t t_length = 0;
					while( BufSize > t_length + 2 )
					{
						buf[t_length+0] = 'A';	buf[t_length+1] = 'C';
						length = xMessageBufferReceive( LogStreamBuffer, &buf[t_length+2], BufSize - t_length - 2, 0 );
						if( length > 0 )
						{
							t_length += 2 + length;
						}
						else
							break;
					}

					//数据写入文件
					if( t_length > 0 ){
						f_write( &SDFile, buf, t_length, &length );
						update_SdSize(t_length, length);
						SDLog_wait_to_sync = true;
					}
				/*写入Log*/
					
				/*写入Txt1*/
					//从缓冲区获取要写入SD卡的数据
					t_length = 0;
					while( BufSize > t_length )
					{
						length = xMessageBufferReceive( Txt1LogStreamBuffer, &buf[t_length], BufSize - t_length, 0 );
						if( length > 0 )
						{
							t_length += length;
						}
						else
							break;
					}
					
					//数据写入文件
					if( t_length > 0 )
					{
						if( Txt1FileCreated == false )
						{	//创建txt1文件
							fres = f_open(&Txt1File, Txt1Filename, FA_CREATE_ALWAYS | FA_WRITE);
							Txt1FileCreated = true;
						}
						f_write( &Txt1File, buf, t_length, &length );
						update_SdSize(t_length, length);
						Txt1_wait_to_sync = true;
					}
				/*写入Txt1*/

				/*写入Ubx*/
					//从缓冲区获取要写入SD卡的数据
					t_length = 0;
					while( BufSize > t_length )
					{
						length = xMessageBufferReceive( UbxLogStreamBuffer, &buf[t_length], BufSize - t_length, 0 );
						if( length > 0 )
						{
							t_length += length;
						}
						else
							break;
					}
					
					//数据写入文件
					if( t_length > 0 )
					{
						if( UbxFileCreated == false )
						{	//创建Ubx文件
							fres = f_open(&UbxFile, UbxFilename, FA_CREATE_ALWAYS | FA_WRITE);
							f_sync(&UbxFile);	
							UbxFileCreated = true;
						}
						fres = f_write( &UbxFile, buf, t_length, &length );
						update_SdSize(t_length, length);
						Ubx_wait_to_sync = true;
					}
				/*写入Ubx*/
					
				//每隔一段时间刷新数据	
				if( last_flush_TIME.get_pass_time()>1 )
				{
					switch(sync_ind)
					{
						case 0:
						{
							if(SDLog_wait_to_sync)
							{
								f_sync(&SDFile);
								SDLog_wait_to_sync = false;
							}
							break;
						}
						
						case 1:
						{
							if(Txt1FileCreated && Txt1_wait_to_sync)
							{				
								f_sync(&Txt1File);
								Txt1_wait_to_sync = false;
							}
							break;
						}
						
						default:
						{
							if(UbxFileCreated && Ubx_wait_to_sync)
							{				
								f_sync(&UbxFile);
								Ubx_wait_to_sync = false;
							}
							break;
						}
					}

					if( ++sync_ind >= 3 )
						sync_ind = 0;				
					last_flush_TIME = TIME::now();
				}
			}
			else
			{	//usb已连接不写入数据
				//清空缓冲区
				if( last_usb_connected != usb_connected )
				{
					xMessageBufferReset(LogStreamBuffer);
					xMessageBufferReset(Txt1LogStreamBuffer);
					xMessageBufferReset(UbxLogStreamBuffer);
				}
			}
			last_usb_connected = usb_connected;
			vTaskDelayUntil( &xLastWakeTime, 1 );
		}
	}
	else
	{	//SD卡挂载失败
		//完成sd初始化
		Set_SD_Init_Complete();
		while(1)
		{
			bool usb_connected = get_is_usb_connected();
			if( usb_connected == false )
			{
				//电脑修改过sd卡
				//重新挂载
				if( mscWritten )
					goto reload_SD;
				else
					os_delay(0.01);
			}
			else
				os_delay(0.01);
		}
	}
	
sdErr:
	//sd卡内容初始化失败
	//完成sd初始化
	Set_SD_Init_Complete();
	while(1)
	{
		bool usb_connected = get_is_usb_connected();
		if( usb_connected == false )
		{
			//电脑修改过sd卡
			//重新挂载
			if( mscWritten )
				goto reload_SD;
			else
				os_delay(0.01);
		}
	}
}

#define SDS_StackSize 4500
Static_AXIDMABuf StackType_t SDS_Stack[SDS_StackSize];
Static_AXIDMABuf StaticTask_t SDS_TaskBuffer;
void init_SDStorage()
{
	//数据记录参数
	MAV_PARAM_TYPE param_types[] = {
		MAV_PARAM_TYPE_UINT32 ,	//系统状态记录分频
		MAV_PARAM_TYPE_UINT32 ,	//姿态记录分频
		MAV_PARAM_TYPE_UINT32 ,	//位置速度记录分频
		MAV_PARAM_TYPE_UINT32 ,	//原始IMU记录分频
		MAV_PARAM_TYPE_UINT32 ,	//姿态控制器记录分频
		MAV_PARAM_TYPE_UINT32 ,	//位置控制器记录分频
		MAV_PARAM_TYPE_UINT32 ,	//位置传感器记录
		MAV_PARAM_TYPE_UINT32 ,	//接收机信号记录
		MAV_PARAM_TYPE_UINT32 ,	//PPK记录 0-不记录 1-GPS 2-RTK
	};
	SName param_names[] = {
		"SDLog_SysState" , //系统状态记录分频
		"SDLog_Att" ,	//姿态记录分频
		"SDLog_LocalNed" ,	//位置速度记录分频
		"SDLog_RawIMU" ,	//原始IMU记录分频
		"SDLog_AttCtrl" ,	//姿态控制器记录分频
		"SDLog_PosCtrl" ,	//位置控制器记录分频
		"SDLog_PosSensor" ,	//位置传感器记录
		"SDLog_Receiver" ,	//接收机信号记录
		"SDLog_PPK" ,	//PPK记录 0-不记录 1-GPS 2-RTK
	};
	uint64_t initial_cfg[] = {
		20 ,	//系统状态记录分频
		5 ,	//姿态记录分频
		5 ,	//位置速度记录分频
		0 ,	//原始IMU记录分频
		10 ,	//姿态控制器记录分频
		10 ,	//位置控制器记录分频
		1 ,	//位置传感器记录
		1 ,	//接收机信号记录
		0 ,	//PPK记录 0-不记录 1-GPS 2-RTK
	};
	ParamGroupRegister( "SDLog", 2, sizeof(initial_cfg)/8, param_types, param_names, (uint64_t*)&initial_cfg );
	
	//xTaskCreate( SDS_Task , "SDS_Task" ,3600 , NULL , SysPriority_UserTask , NULL );
	xTaskCreateStatic( SDS_Task , "SDLog" ,SDS_StackSize, NULL, SysPriority_UserTask, SDS_Stack, &SDS_TaskBuffer);
}