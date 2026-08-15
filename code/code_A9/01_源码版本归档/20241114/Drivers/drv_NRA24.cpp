#include "drv_NRA24.hpp"
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
	uint16_t Message_Id;
	uint8_t payload[7];
	uint8_t checkSum;
}__PACKED _NRA24;
static const unsigned char packet_ID[2] = { 0xAA , 0xAA };
static void NRA24_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	/*状态机*/
		__attribute__ ((aligned (4))) _NRA24 SensorD;
		unsigned char rc_counter = 0;
		uint8_t sum = 0;
	
		uint8_t targetCount = 0;
		uint8_t targetInd = 0;
		double minDist = -1;
	/*状态机*/
	while(1)
	{
		uint8_t rdata=0;
		if( driver_info.port.read( &rdata, 1, 2, 0.5 ) )
		{
			if( rc_counter < 2 )
			{
				//复位状态机
				sum = 0;
				
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
				if( rc_counter >= 4 )//前7个字节之和
					sum += (unsigned char)rdata;
				++rc_counter;
			}
			else
			{	//校验
				if( sum == rdata )
				{	//校验成功
					switch(SensorD.Message_Id)
					{
						case 0x60A:
						{	//系统状态
							break;
						}
						
						case 0x70B:
						{	//目标输出状态
							typedef struct
							{
								uint8_t targetCount;
								uint8_t rollCount:2;
							}__PACKED _TARGET_STATUS;
							const _TARGET_STATUS* targetStatus = (_TARGET_STATUS*)&SensorD.payload;
							
							targetCount = targetStatus->targetCount;
							targetInd = 0;
							minDist = -1;
							
							if( targetCount == 0 )
							{	//无目标
								PositionSensorSetInavailable( default_mmWaveRadarAH_sensor_index,driver_info.sensor_key );
							}
							break;
						}
						
						case 0x70C:
						{	//目标输出信息
							typedef struct
							{
								uint8_t index;
								uint8_t rcs;
								uint16_t range;
								uint8_t rsv1;
								uint8_t velRelH:3;
								uint8_t rsv2:3;
								uint8_t rollCount:2;
								uint8_t velRelL;
							}__PACKED _TARGET_INFO;
							_TARGET_INFO* targetInfo = (_TARGET_INFO*)&SensorD.payload;
							
							if( targetInfo->index==targetInd+1 && targetInd<targetCount )
							{	//处理目标信息
								++targetInd;
								targetInfo->range = __REV16(targetInfo->range);
								uint16_t velRel = (targetInfo->velRelH<<8) | targetInfo->velRelL;
								
								//过滤距离过小目标
								if( targetInfo->range > driver_info.param )
								{
									double rDist = targetInfo->range - driver_info.param;;
									if( minDist<0 || rDist<minDist )
										minDist = rDist;
								}
								
								if( targetInd == targetCount )
								{	//接收完成更新
									if( minDist < 0 )
									{	//无目标
										PositionSensorSetInavailable( default_mmWaveRadarAH_sensor_index,driver_info.sensor_key );
									}
									else
									{
										vector3<double> position;
										position.z = minDist;								
										//获取倾角
										Quaternion quat;
										get_Airframe_quat( &quat ); 
										double lean_cosin = quat.get_lean_angle_cosin();
										//更新
										position.z *= lean_cosin;
										PositionSensorUpdatePosition( default_mmWaveRadarAH_sensor_index,driver_info.sensor_key, position, true );
									}
								}
							}
							else
							{	//接收目标出错重置状态机
								targetCount = 0;
								targetInd = 0;
							}
							break;
						}
					}
				}
				
				rc_counter = 0;
			}
		}
	}
}

static bool NRA24_DriverInit( Port port, uint32_t param )
{
	//波特率115200
	port.SetBaudRate( 115200, 2, 2 );
	//注册传感器
	uint32_t sensor_key = PositionSensorRegister( default_mmWaveRadarAH_sensor_index , \
																								"NRA24" ,\
																								Position_Sensor_Type_RangePositioning , \
																								Position_Sensor_DataType_s_z , \
																								Position_Sensor_frame_ENU , \
																								0.03 , //延时
																								0 ,	//xy信任度
																								0 //z信任度
																								);
	if(sensor_key==0)
		return false;
	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	driver_info->sensor_key = sensor_key;
	xTaskCreate( NRA24_Server, "NRA24", 1000, (void*)driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_NRA24()
{
	PortFunc_Register( 58, NRA24_DriverInit );
}



