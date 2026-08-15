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
#include "ControlSystem.hpp"

struct DriverInfo
{
	uint32_t param;
	Port port;
};

static inline void send_gps_attitude_toPod(DriverInfo* driver_info)
{
	struct _droneInfo {
		uint8_t header;
		uint8_t id;   
		uint8_t length;  
		int16_t roll;  // 左倾为负，右倾为正
		int16_t pitch; // 低头为负，抬头为正
		int16_t yaw;   // 顺时针为0~180  逆时针为0~-180
		int32_t lat;   // *1e7
		int32_t lon;   // *1e7
		int16_t asml;  // 飞机GPS（海拔）高度，单位分米
		int16_t relativeAlt;  // 飞机相对高度，单位分米
		uint8_t res[6];
		int16_t velE;
		int16_t velN;
		int16_t velU;
		uint8_t crc;
	}__attribute__((packed));	

	if( get_Attitude_MSStatus() != MS_Ready )
		return;
	Quaternion airframe_quat;
	get_AirframeY_quat(&airframe_quat);
	airframe_quat.Enu2Ned();
	
	if( get_Altitude_MSStatus() != MS_Ready )
		return;
	mavlink_message_t msg_sd;	
	Position_Sensor gps_sensor;

	if( GetPositionSensor( default_rtk_sensor_index, &gps_sensor ) )
	{	
		
	}
	else if( GetPositionSensor( default_gps_sensor_index, &gps_sensor ) )
	{
		
	}else{
		return;
	}
	
	static bool last_inFlight = false;
	static double homeZ = 0;
	bool inFlight = false;
	get_is_inFlight(&inFlight);
	if( inFlight != last_inFlight )
	{
		getHomeLocalZ( &homeZ, 0, 0.01 );
		last_inFlight = inFlight;
	}

	vector3<double> position;
	get_Position_Ctrl(&position);
	double heightAboveGround = position.z - homeZ;
	
	_droneInfo droneInfo;
	droneInfo.header = 0xEE; // 1
	droneInfo.id 		 = 0x81; // 2
	droneInfo.length = 0x22; // 3
	droneInfo.roll   = rad2degree(airframe_quat.getRoll())*10;  // 5
	droneInfo.pitch  = rad2degree(airframe_quat.getPitch())*10; // 7
	droneInfo.yaw    = rad2degree(airframe_quat.getYaw())*10;   // 9
	droneInfo.lat    = gps_sensor.data.position_Global.x*1e+7;	// 13
	droneInfo.lon    = gps_sensor.data.position_Global.y*1e+7;	// 17
	droneInfo.asml   = gps_sensor.data.position_Global.z*0.1;	  // 19
	droneInfo.relativeAlt  = heightAboveGround*0.1;	            // 21
	droneInfo.res[0] = 0;  // 22
	droneInfo.res[1] = 0;  // 23
	droneInfo.res[2] = 0;  // 24
	droneInfo.res[3] = 0;  // 25
	droneInfo.res[4] = 0;  // 26
	droneInfo.res[5] = 0;  // 27
	droneInfo.velE   = 0;  // 29
	droneInfo.velN   = 0;  // 31 
	droneInfo.velU   = 0;  // 33  
	droneInfo.crc    = 0;  // 34  
	
//	droneInfo.header = 0xEE; // 1
//	droneInfo.id 		 = 0x81; // 2
//	droneInfo.length = 0x22; // 3
//	droneInfo.roll   = 0*10;  // 5
//	droneInfo.pitch  = 0*10; // 7
//	droneInfo.yaw    = 0*10;   // 9
//	droneInfo.lat    = 21.11*1e+7;	// 13
//	droneInfo.lon    = 113.22*1e+7;	// 17
//	droneInfo.asml   = 1000*10;	  // 19
//	droneInfo.relativeAlt  = 1000*10;	            // 21
//	droneInfo.res[0] = 0;  // 22
//	droneInfo.res[1] = 0;  // 23
//	droneInfo.res[2] = 0;  // 24
//	droneInfo.res[3] = 0;  // 25
//	droneInfo.res[4] = 0;  // 26
//	droneInfo.res[5] = 0;  // 27
//	droneInfo.velE   = 0;  // 29
//	droneInfo.velN   = 0;  // 31 
//	droneInfo.velU   = 0;  // 33  
//	droneInfo.crc    = 0;  // 34  
	
	uint8_t* ptr = (uint8_t*)&droneInfo;
	for(int i=0;i<33;i++){
		droneInfo.crc += ptr[i];
	}
	
	if(driver_info){
		driver_info->port.write(ptr,34,0.01,0.01);
	}
}

static void YT_INYYO_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;

	while(1)
	{
		send_gps_attitude_toPod(&driver_info);
		os_delay(0.05);
	}
}

static bool YT_INYYO_DriverInit( Port port, uint32_t param )
{
	//波特率19200
	port.SetBaudRate( 115200, 2, 2 );

	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	xTaskCreate( YT_INYYO_Server, "YT_INYYO",  812, (void*)driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_YT_INYYO()
{
	PortFunc_Register( 102, YT_INYYO_DriverInit );
}