#include "drv_YT_Viewlink.hpp"
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

static uint8_t componentID = MAV_COMP_ID_CAMERA_VIEWLING;

struct DriverInfo
{
	uint32_t param;
	Port port;
};

uint8_t viewlink_protocol_checksum(uint8_t *viewlink_data_buf)
{
    uint8_t len = viewlink_data_buf[3];
    uint8_t checksum = len;
    for(uint8_t i =0;i < len-2;i++)
    {
        checksum = checksum ^ viewlink_data_buf[4+i];
    }
    return(checksum);
}

static inline void send_gps_attitude_toPod(DriverInfo* driver_info)
{
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
	
	uint8_t sdata[100];
	sdata[0]=0x55;
	sdata[1]=0xAA;
	sdata[2]=0xDC;
	sdata[3]=0x2D;
	sdata[4]=0xB1;
	sdata[5]=0x07;
	sdata[6]=0x00;
	sdata[7]=0x00;
	sdata[8]=0x00;
	sdata[9]=0x00;
	sdata[10]=0x00;
	sdata[11]=0x00;
	sdata[12]=0x00;
	
	int16_t roll_t = rad2degree(airframe_quat.getRoll())*360.0/65536.0;
	int16_t pitch_t = rad2degree(airframe_quat.getPitch())*360.0/65536.0;
	int16_t yaw_t = rad2degree(airframe_quat.getYaw())*360.0/65536.0;
	
	*(int16_t*)&sdata[13] = static_cast<int16_t>((roll_t&0x00ff)<<8|(roll_t>>8));
	*(int16_t*)&sdata[15] = static_cast<int16_t>((pitch_t&0x00ff)<<8|(roll_t>>8));
	*(int16_t*)&sdata[17] = static_cast<int16_t>((yaw_t&0x00ff)<<8|(roll_t>>8));
	
	// date
	sdata[19]=0x00;
	sdata[20]=0x00;
	
	//time
	sdata[21]=0x00;
	sdata[22]=0x00;
	sdata[23]=0x00;
	
	// GPS heading
	sdata[24]=0x00;
	sdata[25]=0x00;
	
	// position falg
	sdata[26]=0x00;
	
	uint32_t lat = static_cast<uint32_t>(gps_sensor.data.position_Global.x);
	uint32_t lon = static_cast<uint32_t>(gps_sensor.data.position_Global.y);
	uint32_t alt = heightAboveGround*10;
	*(int32_t*)&sdata[27] = static_cast<int32_t>( (lat&0x000000ff)<<24 | (lat&0x0000ff00)<<16 | (lat&0x00ff0000)>>8 | (lat>>24) );
	*(int32_t*)&sdata[31] = static_cast<int32_t>( (lon&0x000000ff)<<24 | (lon&0x0000ff00)<<16 | (lon&0x00ff0000)>>8 | (lon>>24) );
	*(int32_t*)&sdata[35] = static_cast<int32_t>( (alt&0x000000ff)<<24 | (alt&0x0000ff00)<<16 | (alt&0x00ff0000)>>8 | (alt>>24) );
	
	sdata[39]=0x00;
	sdata[40]=0x00;
	
	sdata[41]=0x00;
	sdata[42]=0x00;
	
	sdata[43]=0x00;
	sdata[44]=0x00;
	
	sdata[45]=0x00;
	sdata[46]=0x00;
	
	sdata[47] = viewlink_protocol_checksum(&sdata[0]);
	
	if(driver_info){
		driver_info->port.write(sdata,48,0.01,0.01);
	}
}

static void YT_Viewlink_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;

	while(1)
	{
		send_gps_attitude_toPod(&driver_info);
		os_delay(0.02);
	}
}

static bool YT_Viewlink_DriverInit( Port port, uint32_t param )
{
	//波特率19200
	port.SetBaudRate( 115200, 2, 2 );

	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	xTaskCreate( YT_Viewlink_Server, "YT_Viewlink",  812, (void*)driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_YT_Viewlink()
{
	PortFunc_Register( 130, YT_Viewlink_DriverInit );
}