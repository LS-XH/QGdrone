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
#define VL53L1x_SensorInd  15
uint32_t VL53L1x_sensorkey = 0;

typedef struct
{   
	int16_t flow_x_integral;	// X轴从最后一次数据更新到当前更新的位移
	int16_t flow_y_integral;	// Y轴从最后一次数据更新到当前更新的位移
	int32_t distance;	//测距
	uint8_t quality;	//图像质量0到100，30以下数据较差建议舍弃
}__PACKED _Flow;

static const unsigned char packet_ID[2] = { 0xfe , 0x04 };
static void OpticalFlow_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	/*状态机*/
		_Flow  Flow;
		unsigned char rc_counter = 0;
		signed char sum = 0;
	/*状态机*/
	
	while(1)
	{
		uint8_t rdata;
		if( driver_info.port.read( &rdata, 1, 2, 0.5 ) )
		{
			if( rc_counter < 2 )
			{
				//接收包头
				if( rdata != packet_ID[ rc_counter ] )
					rc_counter = 0;
				else
				{
					++rc_counter;
					sum = 0;
				}
			}
			else if( rc_counter < 11 )
			{	//接收数据
				( (unsigned char*)&Flow )[ rc_counter - 2 ] = rdata;
				if(rc_counter < 6)
					sum += (signed char)rdata;
				++rc_counter;
			}
			else
			{	//接收包尾
				if( rdata == 0xAA && Flow.quality > 30 )
				{
					sum = 0 ;

					//获取高度
					double height = (double)Flow.distance / 10;
					vector3<double> position;
					position.z = height;
					//获取角速度
					vector3<double> AngularRate;
					get_AngularRate_Ctrl( &AngularRate );
					//获取倾角
					Quaternion quat_flow;
					get_Airframe_quat( &quat_flow );
					double lean_cosin = quat_flow.get_lean_angle_cosin();
					//补偿光流
					vector3<double> vel;
					#define OKp 460
					double rotation_compensation_x = -constrain( AngularRate.y * OKp , 4500000000.0 );
					double rotation_compensation_y = constrain(  AngularRate.x * OKp , 4500000000.0 );						
					//求积分时间
					
					if(height>0 && height < 390){
						Position_Sensor_Data sensor;
						GetPositionSensorData( default_optical_flow_index, &sensor );
						double integral_time = sensor.last_update_time.get_pass_time();
						if( integral_time > 1e-3 )
						{
							double temp_flow_x, temp_flow_y;
							double flow_x, flow_y;
							temp_flow_x = -Flow.flow_x_integral;
							temp_flow_y = Flow.flow_y_integral;
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
							
							double freq = 1.0 / integral_time;
							
							extern float debug_test[30];
							debug_test[25] += ( flow_x - rotation_compensation_x ) * (1.0/OKp) * ( 1 + height );
							debug_test[26] += ( flow_y - rotation_compensation_y ) * (1.0/OKp) * ( 1 + height );
							debug_test[27] = position.z;
							
							
							vel.x = ( flow_x*freq - rotation_compensation_x ) * (1.0/OKp) * ( 1 + height );
							vel.y = ( flow_y*freq - rotation_compensation_y ) * (1.0/OKp) * ( 1 + height );
							PositionSensorUpdateVel( default_optical_flow_index,driver_info.sensor_key, vel , true );
							
							PositionSensorUpdatePosition( VL53L1x_SensorInd, VL53L1x_sensorkey, position, true ); 
						}
						else
							PositionSensorSetInavailable( default_optical_flow_index,driver_info.sensor_key );
					}
				}
				rc_counter = 0;
			}
			
		}
	}
}

static bool OpticalFlow_Laser_ACFLY_DriverInit( Port port, uint32_t param )
{
	//波特率115200
	port.SetBaudRate( 115200, 2, 2 );
	//注册传感器
	uint32_t sensor_key = PositionSensorRegister( default_optical_flow_index , \
																			"LaserACFLY" ,\
																			Position_Sensor_Type_RelativePositioning , \
																			Position_Sensor_DataType_v_xy_nAC , \
																			Position_Sensor_frame_BodyHeading , \
																			0.1, 100);
	
	//注册传感器
	VL53L1x_sensorkey = PositionSensorRegister( VL53L1x_SensorInd , \
													"VL53L1" ,\
													Position_Sensor_Type_RangePositioning , \
													Position_Sensor_DataType_s_z , \
													Position_Sensor_frame_ENU , \
													0.05 , //延时
													0 ,	//xy信任度
													0 //z信任度
													) ;
	
	if((sensor_key==0) || (VL53L1x_sensorkey==0))
		return false;
	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = 2;
	driver_info->port = port;
	driver_info->sensor_key = sensor_key;
	xTaskCreate( OpticalFlow_Server, "OptFlowLaserACFLY", 1024, (void*)driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_OpticalFlow_Laser_ACFLY()
{
	PortFunc_Register( 36, OpticalFlow_Laser_ACFLY_DriverInit );
}