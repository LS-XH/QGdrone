#include "drv_ISTRA24.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"
#include "StorageSystem.hpp"
#include "ControlSystem.hpp"
#include "Commulink.hpp"

struct DriverInfo
{
	uint32_t param;
	Port port;
	uint32_t sensor_key;
};

typedef struct
{
	uint16_t Message_Id;//ID
	uint16_t Strength;//信号强度
	uint16_t Range;	//目标距离
	uint8_t Rsv[5];	//预留
}__PACKED _ISTRA24;
static const unsigned char packet_ID[2] = { 0xAA , 0xAA };
static void ISTRA24_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	/*状态机*/
		__attribute__ ((aligned (4))) _ISTRA24  SensorD;
		unsigned char rc_counter = 0;
		unsigned short sum = 0;
	/*状态机*/
	while(1)
	{
		uint8_t rdata=0;
		if( driver_info.port.read( &rdata, 1, 2, 0.5 ) )
		{
			if( rc_counter == 0 )
				sum = 0;
			if( rc_counter < 2 )
			{
				//接收包头
				if( rdata != packet_ID[ rc_counter ] )
					rc_counter = 0;
				else
				{
					++rc_counter;
				}
			}
			else if( rc_counter < 11 )
			{	//接收数据
				((unsigned char*)&SensorD)[rc_counter - 2] = rdata;
				if( rc_counter > 3 )//前7个字节之和
					sum += (unsigned char)rdata;
				++rc_counter;
			}
			else
			{	//校验
				if( uint8_t(sum&0x00ff) == rdata )
				{	//校验成功  					
					//大小端转换					
					__attribute__ ((aligned (4))) uint16_t Range = SensorD.Range;
					((uint16_t*)&Range)[0] = __REV16(Range);						
					if( SensorD.Strength > 5 && Range > 20 && Range < 9000 )
					{
						vector3<double> position;
						position.z = Range;								
						//获取倾角
						Quaternion quat;
						get_Airframe_quat( &quat ); 
						double lean_cosin = quat.get_lean_angle_cosin();
						//更新
						position.z *= lean_cosin;
						PositionSensorUpdatePosition( default_mmWaveRadarAH_sensor_index,driver_info.sensor_key, position, true );
					}
					else
					{
						SensorD.Range = 0;
						PositionSensorSetInavailable( default_mmWaveRadarAH_sensor_index,driver_info.sensor_key );
					}
				}
				
				rc_counter = 0;
			}
		}
	}
}

static bool ISTRA24_DriverInit( Port port, uint32_t param )
{
	//波特率115200
	port.SetBaudRate( 115200, 2, 2 );
	//注册传感器
	uint32_t sensor_key = PositionSensorRegister( default_mmWaveRadarAH_sensor_index , \
																								"ISTRA24" ,\
																								Position_Sensor_Type_RangePositioning , \
																								Position_Sensor_DataType_s_z , \
																								Position_Sensor_frame_ENU , \
																								0.05 , //延时
																								0 ,	//xy信任度
																								0 //z信任度
																								);
	if(sensor_key==0)
		return false;
	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	driver_info->sensor_key = sensor_key;
	xTaskCreate( ISTRA24_Server, "ISTRA24", 800, (void*)driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_ISTRA24()
{
	PortFunc_Register( 55, ISTRA24_DriverInit );
}



