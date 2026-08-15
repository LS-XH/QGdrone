#include "drv_US100.hpp"
#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"

struct DriverInfo
{
	uint32_t param;
	Port port;
	uint32_t sensor_key;
};

static void US100_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	uint8_t buf[4];
	driver_info.port.reset_rx(2);
	while(1)
	{
		//读取温度
		buf[0] = 0x50;
		driver_info.port.write( buf,1, 2,0.5 );
		if( driver_info.port.read( buf, 1, 2, 0.5 ) == false )
		{
			driver_info.port.reset_rx(2);
			continue;
		}
		float temperature = (float)buf[0] - 45;
		
		//读取距离
		buf[0] = 0x55;
		driver_info.port.write( buf,1, 2,0.5 );
		if( driver_info.port.read( buf, 2, 2, 0.5 ) == false )
		{
			driver_info.port.reset_rx(2);
			continue;
		}
		vector3<double> position;
		position.z = ( ( buf[0] << 8 ) | buf[1] ) * 0.1;
		//获取倾角
		Quaternion quat;
		get_Airframe_quat( &quat );
		double lean_cosin = quat.get_lean_angle_cosin();
		//更新
		position.z *= lean_cosin;
		PositionSensorUpdatePosition( default_ultrasonic_sensor_index,driver_info.sensor_key, position, true );
	}
}

static bool US100_DriverInit( Port port, uint32_t param )
{
	//波特率115200
	port.SetBaudRate( 9600, 2, 2 );
	//注册传感器
	uint32_t sensor_key = PositionSensorRegister( default_ultrasonic_sensor_index , \
																								"US100" ,\
																								Position_Sensor_Type_RangePositioning , \
																								Position_Sensor_DataType_s_z , \
																								Position_Sensor_frame_ENU , \
																								0.01 , //延时
																								0 ,	//xy信任度
																								0 //z信任度
																								);
	if(sensor_key==0)
		return false;
	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	driver_info->sensor_key = sensor_key;
	xTaskCreate( US100_Server, "US100", 1024, (void*)driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_US100()
{
	PortFunc_Register( 53, US100_DriverInit );
}