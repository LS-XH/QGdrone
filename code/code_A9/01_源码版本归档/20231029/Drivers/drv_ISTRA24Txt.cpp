#include "drv_ISTRA24Txt.hpp"
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

static void ISTRA24Txt_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	/*状态机*/
		unsigned char rc_counter = 0;
		char txtBuf[20];
	/*状态机*/
	while(1)
	{
		uint8_t rdata=0;
		if( driver_info.port.read( &rdata, 1, 2, 0.5 ) )
		{
			if( rc_counter < 1 )
			{
				//接收包头
				if( rdata != 0x20 )
					rc_counter = 0;
				else
					++rc_counter;
			}
			else if( rc_counter < 100 )
			{	//接收数据
				if( rc_counter > 20 )
				{	//数据太多
					rc_counter = 0;
					continue;
				}
				if( is_number(rdata) || rdata==0x2e )
				{	//是数字
					txtBuf[rc_counter-1] = rdata;
					++rc_counter;
				}
				else
				{	//非数字
					if( rc_counter>2 && rdata==0x0d )
					{	//接收到换行符代表接收成功
						txtBuf[rc_counter-1] = 0;
						rc_counter = 100;
					}
					else	//失败
						rc_counter = 0;
				}
			}
			else
			{	//校验
				if( rdata == 0x0a )
				{	//换行符接收成功			
					double Range = atof(txtBuf);			
					if( isvalid(Range) && Range>=0 )
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
						PositionSensorSetInavailable( default_mmWaveRadarAH_sensor_index,driver_info.sensor_key );
					}
				}
				rc_counter = 0;
			}
		}
	}
}

static bool ISTRA24Txt_DriverInit( Port port, uint32_t param )
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
	xTaskCreate( ISTRA24Txt_Server, "ISTRA24Txt", 800, (void*)driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_ISTRA24Txt()
{
	PortFunc_Register( 56, ISTRA24Txt_DriverInit );
}



