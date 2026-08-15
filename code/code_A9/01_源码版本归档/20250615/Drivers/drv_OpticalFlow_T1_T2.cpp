#include "drv_OpticalFlow_T1_T2.hpp"
#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"
#include "ControlSystem.hpp"
#include "StorageSystem.hpp"

#define SensorInd 2

struct DriverInfo
{
	uint32_t param;
	Port port;
	uint32_t sensor_key_opf;
	uint32_t sensor_key_tof;
};

typedef struct
{
	int16_t flow_x_integral;	// X 像素点累计时间内的累加位移(radians*10000)
														// [除以 10000 乘以高度(mm)后为实际位移(mm)]
	int16_t flow_y_integral;	// Y 像素点累计时间内的累加位移(radians*10000)
														// [除以 10000 乘以高度(mm)后为实际位移(mm)]
	uint16_t integration_timespan;	// 上一次发送光流数据到本次发送光流数据的累计时间（us）
	uint16_t ground_distance; // 激光测距距离(mm)，比如低字节为0x12，高字节为0x08，则激光测距距离为0x0812=2066mm
	uint8_t valid;	// 状态值:0(0x00)为光流数据不可用 245为光流数据可用
	uint8_t tof_quality;	//激光测距置信度，比如0x64表示激光测距置信度为100%
}__PACKED _Flow;

static const unsigned char packet_ID[2] = { 0xfe , 0x0a };

static void OpticalFlow_T1_T2_Server(void* pvParameters)
{
	/*状态机*/
		_Flow  Flow;
		unsigned char rc_counter = 0;
		unsigned char sum = 0;
	/*状态机*/
	
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	double freqH = 0.03;
	while(1)
	{
		uint8_t rdata;
		int tof_i = 0;

		if( driver_info.port.read( &rdata, 1, 0.02, 0.02 ) )
		{
			if( rc_counter < 2 )
			{	//接收包头
				if( rdata != packet_ID[ rc_counter ] )
					rc_counter = 0;
				else
				{
					++rc_counter;
					sum = 0;
				}
			}
			else if( rc_counter < 12 )
			{	//接收数据
				( (unsigned char*)&Flow )[ rc_counter - 2 ] = rdata;
				sum ^= rdata;
				++rc_counter;
			}
			else if( rc_counter == 12 )
			{	//校验
				if( sum != rdata )
					rc_counter = 0;
				else
					++rc_counter;
			}
			else
			{	//接收包尾
				if( rdata == 0x55 )
				{	
					if( Flow.valid != 0 )
					{	//测距传感器可用
						double height = Flow.ground_distance / 10;
						vector3<double> position;
						position.z = height;
						
						//获取角速度
						vector3<double> AngularRate;
						get_AngularRate_Ctrl( &AngularRate );
						//补偿光流
						
						double rotation_compensation_x = -constrain( AngularRate.y * 10000 , 4500000000.0 );
						double rotation_compensation_y = constrain(  AngularRate.x * 10000 , 4500000000.0 );
						double integral_time = (Flow.integration_timespan * 1e-6f);
						double temp_flow_x, temp_flow_y;
						double flow_x, flow_y;
						temp_flow_x = Flow.flow_x_integral;
						temp_flow_y = -Flow.flow_y_integral;
						switch(driver_info.param)
						{
							case 0:
							default:
							{
								flow_x = temp_flow_x;
								flow_y = temp_flow_y;
								break;
							}
							case 1:
							{
								flow_x = temp_flow_y;
								flow_y = -temp_flow_x;
								break;
							}
							case 2:
							{
								flow_x = -temp_flow_x;
								flow_y = -temp_flow_y;
								break;
							}
							case 3:
							{
								flow_x = -temp_flow_y;
								flow_y = temp_flow_x;
								break;
							}
						}
						
						integral_time = 1.0 / integral_time;

						vector3<double> vel;
						vel.x = ( flow_x*integral_time - rotation_compensation_x ) * 1e-4f * ( 1 + height );
						vel.y = ( flow_y*integral_time - rotation_compensation_y ) * 1e-4f * ( 1 + height ) ;
						
						double logbuf[5];
						logbuf[0] = temp_flow_x;
						logbuf[1] = temp_flow_y;
						logbuf[2] = vel.x;
						logbuf[3] = vel.y;
						logbuf[4] = height;
						SDLog_Msg_DebugVect( "opticalFlow", logbuf, 5 );

						PositionSensorUpdateVel( default_optical_flow_index,driver_info.sensor_key_opf, vel , true, freqH);
						PositionSensorUpdatePosition( 15, driver_info.sensor_key_tof, position, true, freqH );
					}
					else
					{
						PositionSensorSetInavailable( default_optical_flow_index,driver_info.sensor_key_opf );
					}
					
					Flow.flow_x_integral = 0;
					Flow.flow_y_integral = 0;
					Flow.ground_distance = 0;
					Flow.integration_timespan = 0;
					Flow.tof_quality = 0;
					Flow.valid = 0;
				}
				rc_counter = 0;
			}
		}
	}
}

static bool OpticalFlow_T1_T2_DriverInit( Port port, uint32_t param )
{
	//波特率115200
	port.SetBaudRate( 115200, 1, 1 );
	//注册传感器
	uint32_t sensor_key_opf = PositionSensorRegister( default_optical_flow_index , \
																								"T1_T2_OpticalFlow" ,\
																								Position_Sensor_Type_RelativePositioning , \
																								Position_Sensor_DataType_v_xy_nAC , \
																								Position_Sensor_frame_BodyHeading , \
																								0.1, 100 );																		
	if( sensor_key_opf==0 )
		return false;
	
	//注册传感器
	uint32_t sensor_key_tof = PositionSensorRegister( 15 , \
													"T1_T2_TOF" ,\
													Position_Sensor_Type_RangePositioning , \
													Position_Sensor_DataType_s_z , \
													Position_Sensor_frame_ENU , \
													0.1 , //延时
													0 ,	//xy信任度
													0 //z信任度
													) ;

	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	driver_info->sensor_key_opf = sensor_key_opf;
	driver_info->sensor_key_tof = sensor_key_tof;
	xTaskCreate( OpticalFlow_T1_T2_Server, "T1_T2_OpticalFlow", 1024, (void*)driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_OpticalFlow_T1_T2()
{
	PortFunc_Register( 37, OpticalFlow_T1_T2_DriverInit );
}