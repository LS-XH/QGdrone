#include "Sensors.hpp"
#include "SensorsBackend.hpp"
#include "Basic.hpp"
#include "Parameters.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include "StorageSystem.hpp"
#include "ControlSystem.hpp"
#include "MeasurementSystem.hpp"
#include "AC_Math.hpp"

/*IMU*/
	const uint8_t Ins_Magnetometer_Index = 0;
	const uint8_t Slam_Magnetometer_Index = 0;
	const uint8_t External_Magnetometer_Index = 1;
	const uint8_t Internal_Magnetometer_Index = 2;

	static SemaphoreHandle_t AccelerometersMutex = xSemaphoreCreateMutex();
	static SemaphoreHandle_t GyroscopesMutex = xSemaphoreCreateMutex();
	static SemaphoreHandle_t MagnetometersMutex = xSemaphoreCreateMutex();
	static IMU_Sensor* Accelerometers[IMU_Sensors_Count] = {0};
	static IMU_Sensor* Gyroscopes[IMU_Sensors_Count] = {0};
	static IMU_Sensor* Magnetometers[IMU_Sensors_Count] = {0};
	
	/*IMU传感器注册函数*/
		uint32_t IMUAccelerometerRegister( uint8_t index, SName name, double sensitivity, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return 0;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(AccelerometersMutex,TIMEOUT_Ticks) )
			{
				if( Accelerometers[index] != 0 )
				{
					xSemaphoreGive(AccelerometersMutex);
					return 0;
				}
				Accelerometers[index] = new IMU_Sensor;
				IMU_Sensor* sensor = Accelerometers[index];
				if( sensor )
				{
					sensor->name = name;
					sensor->last_update_time.set_invalid();
					sensor->sensitivity = sensitivity;
					sensor->data.zero();
					sensor->data_raw.zero();
					sensor->sample_time = -1;
					sensor->slam = false;
					
					xSemaphoreGive(AccelerometersMutex);
					
					//注册参数
					MAV_PARAM_TYPE types[] = { MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64 };
					IMUConfig initial_cfg;
					initial_cfg.offset[0] = initial_cfg.offset[1] = initial_cfg.offset[2] = 0;
					initial_cfg.scale[0] = initial_cfg.scale[1] = initial_cfg.scale[2] = 1;
					initial_cfg.STTemperature = 0;
					initial_cfg.TemperatureCoefficient[0] = initial_cfg.TemperatureCoefficient[1] = initial_cfg.TemperatureCoefficient[2] = 0;
					ParamGroupRegister( name+"_Acc", 3, IMUConfigLength, types, 0, (uint64_t*)&initial_cfg );
				}
				return (uint32_t)sensor;
			}
			return 0;
		}
		bool IMUAccelerometerUnRegister( uint8_t index,uint32_t key, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(AccelerometersMutex,TIMEOUT_Ticks) )
			{
				if( Accelerometers[index]==0 || ( (uint32_t)Accelerometers[index]!=key ) )
				{
					xSemaphoreGive(AccelerometersMutex);
					return false;
				}
				delete Accelerometers[index];
				Accelerometers[index] = 0;
				
				xSemaphoreGive(AccelerometersMutex);
				return true;
			}
			return false;
		}
		
		uint32_t IMUGyroscopeRegister( uint8_t index, SName name, double sensitivity, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return 0;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(GyroscopesMutex,TIMEOUT_Ticks) )
			{
				if( Gyroscopes[index] != 0 )
				{
					xSemaphoreGive(GyroscopesMutex);
					return 0;
				}
				Gyroscopes[index] = new IMU_Sensor;
				IMU_Sensor* sensor = Gyroscopes[index];
				sensor->name = name;
				sensor->last_update_time.set_invalid();
				sensor->sensitivity = sensitivity;
				sensor->data.zero();
				sensor->data_raw.zero();
				sensor->sample_time = -1;
				sensor->slam = false;
				
				xSemaphoreGive(GyroscopesMutex);
				
				//注册参数
				MAV_PARAM_TYPE types[] = { MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64 };
				IMUConfig initial_cfg;
				initial_cfg.offset[0] = initial_cfg.offset[1] = initial_cfg.offset[2] = 0;
				initial_cfg.scale[0] = initial_cfg.scale[1] = initial_cfg.scale[2] = 1;
				initial_cfg.STTemperature = 0;
				initial_cfg.TemperatureCoefficient[0] = initial_cfg.TemperatureCoefficient[1] = initial_cfg.TemperatureCoefficient[2] = 0;
				ParamGroupRegister( name+"_Gyro", 3, IMUConfigLength, types, 0, (uint64_t*)&initial_cfg );
				
				return (uint32_t)sensor;
			}
			return 0;
		}
		bool IMUGyroscopeUnRegister( uint8_t index,uint32_t key, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(GyroscopesMutex,TIMEOUT_Ticks) )
			{
				if( Gyroscopes[index]==0 || ( (uint32_t)Gyroscopes[index]!=key ) )
				{
					xSemaphoreGive(GyroscopesMutex);
					return false;
				}
				delete Gyroscopes[index];
				Gyroscopes[index] = 0;
				
				xSemaphoreGive(GyroscopesMutex);
				return true;
			}
			return false;
		}
		
		uint32_t IMUMagnetometerRegister( uint8_t index, SName name, double sensitivity, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return 0;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(MagnetometersMutex,TIMEOUT_Ticks) )
			{
				if( Magnetometers[index] != 0 )
				{
					xSemaphoreGive(MagnetometersMutex);
					return 0;
				}
				Magnetometers[index] = new IMU_Sensor;
				IMU_Sensor* sensor = Magnetometers[index];
				if( sensor )
				{
					sensor->name = name;
					sensor->last_update_time.set_invalid();
					sensor->sensitivity = sensitivity;
					sensor->data.zero();
					sensor->data_raw.zero();
					sensor->sample_time = -1;
					sensor->slam = false;
					
					xSemaphoreGive(MagnetometersMutex);
					
					//注册参数
					MAV_PARAM_TYPE types[] = { MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64 };
					IMUConfig initial_cfg;
					initial_cfg.offset[0] = initial_cfg.offset[1] = initial_cfg.offset[2] = 0;
					initial_cfg.scale[0] = initial_cfg.scale[1] = initial_cfg.scale[2] = 1;
					initial_cfg.STTemperature = 0;
					initial_cfg.TemperatureCoefficient[0] = initial_cfg.TemperatureCoefficient[1] = initial_cfg.TemperatureCoefficient[2] = 0;
					ParamGroupRegister( name+"_Mag", 3, IMUConfigLength, types, 0, (uint64_t*)&initial_cfg );
				}
				return (uint32_t)sensor;
			}
			return 0;
		}
		uint32_t IMUMagnetometerSlamRegister( uint8_t index, SName name, double sensitivity, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return 0;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(MagnetometersMutex,TIMEOUT_Ticks) )
			{
				if( Magnetometers[index] != 0 )
				{
					xSemaphoreGive(MagnetometersMutex);
					return 0;
				}
				Magnetometers[index] = new IMU_Sensor;
				IMU_Sensor* sensor = Magnetometers[index];
				if( sensor )
				{
					sensor->name = name;
					sensor->last_update_time.set_invalid();
					sensor->sensitivity = sensitivity;
					sensor->data.zero();
					sensor->data_raw.zero();
					sensor->sample_time = -1;
					sensor->slam = true;
					
					xSemaphoreGive(MagnetometersMutex);
					
					//注册参数
					MAV_PARAM_TYPE types[] = { MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64 };
					IMUConfig initial_cfg;
					initial_cfg.offset[0] = initial_cfg.offset[1] = initial_cfg.offset[2] = 0;
					initial_cfg.scale[0] = initial_cfg.scale[1] = initial_cfg.scale[2] = 1;
					initial_cfg.STTemperature = 0;
					initial_cfg.TemperatureCoefficient[0] = initial_cfg.TemperatureCoefficient[1] = initial_cfg.TemperatureCoefficient[2] = 0;
					ParamGroupRegister( name+"_Mag", 3, IMUConfigLength, types, 0, (uint64_t*)&initial_cfg );
				}
				return (uint32_t)sensor;
			}
			return 0;
		}
		bool IMUMagnetometerUnRegister( uint8_t index,uint32_t key, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(MagnetometersMutex,TIMEOUT_Ticks) )
			{
				if( Magnetometers[index]==0 || ( (uint32_t)Magnetometers[index]!=key ) )
				{
					xSemaphoreGive(MagnetometersMutex);
					return false;
				}
				delete Magnetometers[index];
				Magnetometers[index] = 0;
				
				xSemaphoreGive(MagnetometersMutex);
				return true;
			}
			return false;
		}
	/*IMU传感器注册函数*/
		
	/*IMU传感器更新函数*/
		static uint8_t IMU_Info_inited = false;
		static float FcDir = 0;	static double fcSinDir=0, fcCosDir=1;
		static float Mag0Dir = 0;	static double mag0SinDir=0, mag0CosDir=1;
		static float Mag1Dir = 0;	static double mag1SinDir=0, mag1CosDir=1;
		
		static inline void getCompVec( vector3<double>& res, const vector3<double>& data, const double& sensitivity, const vector3<double>& scale, const vector3<double>& offset, const double& sinDir, const double& cosDir  )
		{
			double res_x = (data.x - offset.x) * sensitivity * scale.x;
			double res_y = (data.y - offset.y) * sensitivity * scale.y;
			res.x = BodyHeading2ENU_x( res_x, res_y, sinDir, cosDir );
			res.y = BodyHeading2ENU_y( res_x, res_y, sinDir, cosDir );
			res.z = (data.z - offset.z) * sensitivity * scale.z;
		}
		
		bool IMUAccelerometerUpdate( uint8_t index,uint32_t key, vector3<int32_t> data, bool data_error, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			if( IMU_Info_inited == false )
				return false;
					
			vector3<double> offset(0,0,0);			
			vector3<double> scale(1,1,1);			
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(AccelerometersMutex,TIMEOUT_Ticks) )
			{
				if( Accelerometers[index]==0 || ( (uint32_t)Accelerometers[index]!=key ) )
				{
					xSemaphoreGive(AccelerometersMutex);
					return false;
				}
				
				IMU_Sensor* sensor = Accelerometers[index];
				//读取参数				
				IMUConfig cfg;	bool is_new;
				if( ReadParamGroup( sensor->name + "_Acc", (uint64_t*)&cfg, &is_new ) == PR_OK )
				{
					if( !is_new &&
							isvalid(cfg.offset[0]) && isvalid(cfg.offset[1]) && isvalid(cfg.offset[2]) &&
							isvalid(cfg.scale[0]) && isvalid(cfg.scale[1]) && isvalid(cfg.scale[2])
						)
					{
						offset.set_vector( cfg.offset[0], cfg.offset[1], cfg.offset[2] );
						scale.set_vector( cfg.scale[0], cfg.scale[1], cfg.scale[2] );
						sensor->calibrated = true;
					}
					else
						sensor->calibrated = false;
				}
				else
				{
					xSemaphoreGive(AccelerometersMutex);
					return false;
				}
				
				//写入更新时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//写入传感器数据
				sensor->have_temperature = false;
				sensor->data_error = data_error;
				sensor->data_raw = data;
				sensor->data_rawTC.set_vector( data.x, data.y, data.z );
				getCompVec( sensor->data, sensor->data_rawTC, sensor->sensitivity, scale, offset, fcSinDir, fcCosDir );
				
				xSemaphoreGive(AccelerometersMutex);
				return true;
			}
			return false;
		}
		bool IMUAccelerometerUpdateTC( uint8_t index,uint32_t key, vector3<int32_t> data, bool data_error, double temperature, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			if( IMU_Info_inited == false )
				return false;
			
			vector3<double> offset(0,0,0);			
			vector3<double> scale(1,1,1);			
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(AccelerometersMutex,TIMEOUT_Ticks) )
			{
				if( Accelerometers[index]==0 || ( (uint32_t)Accelerometers[index]!=key ) )
				{
					xSemaphoreGive(AccelerometersMutex);
					return false;
				}
				
				IMU_Sensor* sensor = Accelerometers[index];
				//读取参数				
				IMUConfig cfg;	bool is_new;
				if( ReadParamGroup( sensor->name + "_Acc", (uint64_t*)&cfg, &is_new ) == PR_OK )
				{
					if( !is_new &&
							isvalid(cfg.offset[0]) && isvalid(cfg.offset[1]) && isvalid(cfg.offset[2]) &&
							isvalid(cfg.scale[0]) && isvalid(cfg.scale[1]) && isvalid(cfg.scale[2])
						)
					{
						offset.set_vector( cfg.offset[0], cfg.offset[1], cfg.offset[2] );
						scale.set_vector( cfg.scale[0], cfg.scale[1], cfg.scale[2] );
						sensor->calibrated = true;
					}
					else
						sensor->calibrated = false;
				}
				else
				{
					xSemaphoreGive(AccelerometersMutex);
					return false;
				}
				
				//写入更新时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//写入传感器数据
				sensor->have_temperature = true;
				sensor->data_error = data_error;
			 	sensor->temperature = temperature;
				sensor->data_raw = data;
				double temperature_err = temperature - cfg.STTemperature;
				sensor->data_rawTC.set_vector(
					sensor->data_raw.x - temperature_err*cfg.TemperatureCoefficient[0],
					sensor->data_raw.y - temperature_err*cfg.TemperatureCoefficient[1],
					sensor->data_raw.z - temperature_err*cfg.TemperatureCoefficient[2] );
				getCompVec( sensor->data, sensor->data_rawTC, sensor->sensitivity, scale, offset, fcSinDir, fcCosDir );
				
				xSemaphoreGive(AccelerometersMutex);
				return true;
			}
			return false;
		}
		
		bool IMUGyroscopeUpdate( uint8_t index,uint32_t key, vector3<int32_t> data, bool data_error, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			if( IMU_Info_inited == false )
			{
				float _dir[2];
				if( ReadParam( "POfs_Fc_Dir", 0, 0, (uint64_t*)_dir, 0 ) == PR_OK )
				{
					FcDir = degree2rad(_dir[0]);
					fcSinDir = sin(FcDir);
					fcCosDir = cos(FcDir);
					IMU_Info_inited = true;
				}
				else
					return false;
				
				if( ReadParam( "POfs_Mag0_Dir", 0, 0, (uint64_t*)_dir, 0 ) == PR_OK )
				{
					Mag0Dir = degree2rad(_dir[0]);
					mag0SinDir = sin(Mag0Dir);
					mag0CosDir = cos(Mag0Dir);
				}
				if( ReadParam( "POfs_Mag1_Dir", 0, 0, (uint64_t*)_dir, 0 ) == PR_OK )
				{
					Mag1Dir = degree2rad(_dir[0]);
					mag1SinDir = sin(Mag1Dir);
					mag1CosDir = cos(Mag1Dir);
				}
			}
			
			vector3<double> offset(0,0,0);
			vector3<double> scale(1,1,1);
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(GyroscopesMutex,TIMEOUT_Ticks) )
			{
				if( Gyroscopes[index]==0 || ( (uint32_t)Gyroscopes[index]!=key ) )
				{
					xSemaphoreGive(GyroscopesMutex);
					return false;
				}
				
				IMU_Sensor* sensor = Gyroscopes[index];
				//读取参数
				IMUConfig cfg;	bool is_new;
				if( ReadParamGroup( sensor->name + "_Gyro", (uint64_t*)&cfg, &is_new ) == PR_OK )
				{
					if( !is_new &&
							isvalid(cfg.offset[0]) && isvalid(cfg.offset[1]) && isvalid(cfg.offset[2]) &&
							isvalid(cfg.scale[0]) && isvalid(cfg.scale[1]) && isvalid(cfg.scale[2])
						)
					{
						offset.set_vector( cfg.offset[0], cfg.offset[1], cfg.offset[2] );
						scale.set_vector( cfg.scale[0], cfg.scale[1], cfg.scale[2] );
						sensor->calibrated = true;
					}
					else
						sensor->calibrated = false;
				}
				else
				{
					xSemaphoreGive(GyroscopesMutex);
					return false;
				}
				
				//写入更新时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//写入传感器数据
				sensor->have_temperature = false;
				sensor->data_error = data_error;
				sensor->data_raw = data;
				sensor->data_rawTC.set_vector( data.x, data.y, data.z );
				getCompVec( sensor->data, sensor->data_rawTC, sensor->sensitivity, scale, offset, fcSinDir, fcCosDir );
				
				xSemaphoreGive(GyroscopesMutex);
				return true;
			}
			return false;
		}
		bool IMUGyroscopeUpdateTC( uint8_t index,uint32_t key, vector3<int32_t> data, bool data_error, double temperature, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			if( IMU_Info_inited == false )
			{
				float _dir[2];
				if( ReadParam( "POfs_Fc_Dir", 0, 0, (uint64_t*)_dir, 0 ) == PR_OK )
				{
					FcDir = degree2rad(_dir[0]);
					fcSinDir = sin(FcDir);
					fcCosDir = cos(FcDir);
					IMU_Info_inited = true;
				}
				else
					return false;
				
				if( ReadParam( "POfs_Mag0_Dir", 0, 0, (uint64_t*)_dir, 0 ) == PR_OK )
				{
					Mag0Dir = degree2rad(_dir[0]);
					mag0SinDir = sin(Mag0Dir);
					mag0CosDir = cos(Mag0Dir);
				}
				if( ReadParam( "POfs_Mag1_Dir", 0, 0, (uint64_t*)_dir, 0 ) == PR_OK )
				{
					Mag1Dir = degree2rad(_dir[0]);
					mag1SinDir = sin(Mag1Dir);
					mag1CosDir = cos(Mag1Dir);
				}
			}
			
			vector3<double> offset(0,0,0);
			vector3<double> scale(1,1,1);
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(GyroscopesMutex,TIMEOUT_Ticks) )
			{
				if( Gyroscopes[index]==0 || ( (uint32_t)Gyroscopes[index]!=key ) )
				{
					xSemaphoreGive(GyroscopesMutex);
					return false;
				}
				
				IMU_Sensor* sensor = Gyroscopes[index];
				//读取参数
				IMUConfig cfg;	bool is_new;
				if( ReadParamGroup( sensor->name + "_Gyro", (uint64_t*)&cfg, &is_new ) == PR_OK )
				{
					if( !is_new &&
							isvalid(cfg.offset[0]) && isvalid(cfg.offset[1]) && isvalid(cfg.offset[2]) &&
							isvalid(cfg.scale[0]) && isvalid(cfg.scale[1]) && isvalid(cfg.scale[2])
						)
					{
						offset.set_vector( cfg.offset[0], cfg.offset[1], cfg.offset[2] );
						scale.set_vector( cfg.scale[0], cfg.scale[1], cfg.scale[2] );
						sensor->calibrated = true;
					}
					else
						sensor->calibrated = false;
				}
				else
				{
					xSemaphoreGive(GyroscopesMutex);
					return false;
				}
				
				//写入更新时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//写入传感器数据
				sensor->have_temperature = true;
				sensor->temperature = temperature;
				sensor->data_error = data_error;
				sensor->data_raw = data;
				double temperature_err = temperature - cfg.STTemperature;
				sensor->data_rawTC.set_vector(
					sensor->data_raw.x - temperature_err*cfg.TemperatureCoefficient[0],
					sensor->data_raw.y - temperature_err*cfg.TemperatureCoefficient[1],
					sensor->data_raw.z - temperature_err*cfg.TemperatureCoefficient[2] );
				getCompVec( sensor->data, sensor->data_rawTC, sensor->sensitivity, scale, offset, fcSinDir, fcCosDir );
				
//				Quaternion quat;
//				quat.rotate_delta_angle( vector3<double>( degree2rad(0.1),degree2rad(-0.3),degree2rad(-0.3)) );
//				sensor->data = quat.rotate(sensor->data);
				
				xSemaphoreGive(GyroscopesMutex);
				return true;
			}
			return false;
		}
		bool IMUGyroscopeAddBodyOffset( uint8_t index, vector3<double> offset, bool st, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			if( IMU_Info_inited == false )
				return false;
			
			IMU_Sensor sensor;
			if( GetGyroscope( index, &sensor, TIMEOUT ) )
			{
				//读取参数
				IMUConfig cfg;
				if( ReadParamGroup( sensor.name + "_Gyro", (uint64_t*)&cfg, 0 ) == PR_OK )
				{
					if( !is_zero(sensor.sensitivity) )
					{
						double invSense = 1.0 / sensor.sensitivity;
						
						vector3<double> res_offset;
						double res_offset_x = offset.x * invSense;
						double res_offset_y = offset.y * invSense;
						res_offset.x = ENU2BodyHeading_x( res_offset_x, res_offset_y, fcSinDir, fcCosDir );
						res_offset.y = ENU2BodyHeading_y( res_offset_x, res_offset_y, fcSinDir, fcCosDir );
						res_offset.z = offset.z * invSense;
						
						cfg.offset[0] += res_offset.x;
						cfg.offset[1] += res_offset.y;
						cfg.offset[2] += res_offset.z;
						
						PR_RESULT res = UpdateParamGroup( sensor.name+"_Gyro", (uint64_t*)&cfg, 0, IMUConfigLength, st );
						if( res==PR_OK )
							return true;
						return false;
					}
					return false;
				}
				else
					return false;
			}
			return false;
		}
		bool IMUGyroscopeSaveBodyOffset( uint8_t index, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			if( IMU_Info_inited == false )
				return false;
			
			IMU_Sensor sensor;
			if( GetGyroscope( index, &sensor, TIMEOUT ) )
			{
				PR_RESULT res = SaveParamGroup( sensor.name+"_Gyro" );
				if( res==PR_OK )
					return true;
				return false;
			}
			return false;
		}
		
		bool IMUMagnetometerUpdate( uint8_t index,uint32_t key, vector3<int32_t> data, bool data_error, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			if( IMU_Info_inited == false )
				return false;
			
			vector3<double> offset(0,0,0);
			vector3<double> scale(1,1,1);
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(MagnetometersMutex,TIMEOUT_Ticks) )
			{
				if( Magnetometers[index]==0 || ( (uint32_t)Magnetometers[index]!=key ) )
				{
					xSemaphoreGive(MagnetometersMutex);
					return false;
				}
				
				IMU_Sensor* sensor = Magnetometers[index];
				//读取参数
				IMUConfig cfg;	bool is_new;
				if( ReadParamGroup( sensor->name + "_Mag", (uint64_t*)&cfg, &is_new ) == PR_OK )
				{
					if( !is_new &&
							isvalid(cfg.offset[0]) && isvalid(cfg.offset[1]) && isvalid(cfg.offset[2]) &&
							isvalid(cfg.scale[0]) && isvalid(cfg.scale[1]) && isvalid(cfg.scale[2])
						)
					{
						offset.set_vector( cfg.offset[0], cfg.offset[1], cfg.offset[2] );
						scale.set_vector( cfg.scale[0], cfg.scale[1], cfg.scale[2] );
						sensor->calibrated = true;
					}
					else
						sensor->calibrated = false;
				}
				else
				{
					xSemaphoreGive(MagnetometersMutex);
					return false;
				}
				
				//写入更新时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//写入传感器数据
				sensor->have_temperature = false;
				sensor->data_error = data_error;
				sensor->data_raw = data;
				sensor->data_rawTC.set_vector( data.x, data.y, data.z );
				double sinDir, cosDir;
				switch(index)
				{
					case 0:
						sinDir = mag0SinDir;
						cosDir = mag0CosDir;
						break;
					case 1:
						sinDir = mag1SinDir;
						cosDir = mag1CosDir;
						break;
					case Internal_Magnetometer_Index:
						sinDir = fcSinDir;
						cosDir = fcCosDir;
						break;
					default:
						sinDir = 0;
						cosDir = 1;
						break;
				}
				getCompVec( sensor->data, sensor->data_rawTC, sensor->sensitivity, scale, offset, sinDir, cosDir );
				
				xSemaphoreGive(MagnetometersMutex);
				return true;
			}
			return false;
		}
		bool IMUMagnetometerUpdateTC( uint8_t index,uint32_t key, vector3<int32_t> data, bool data_error, double temperature, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			if( IMU_Info_inited == false )
				return false;
			
			vector3<double> offset(0,0,0);
			vector3<double> scale(1,1,1);
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(MagnetometersMutex,TIMEOUT_Ticks) )
			{
				if( Magnetometers[index]==0 || ( (uint32_t)Magnetometers[index]!=key ) )
				{
					xSemaphoreGive(MagnetometersMutex);
					return false;
				}
				
				IMU_Sensor* sensor = Magnetometers[index];
				//读取参数
				IMUConfig cfg;	bool is_new;
				if( ReadParamGroup( sensor->name + "_Mag", (uint64_t*)&cfg, &is_new ) == PR_OK )
				{
					if( !is_new &&
							isvalid(cfg.offset[0]) && isvalid(cfg.offset[1]) && isvalid(cfg.offset[2]) &&
							isvalid(cfg.scale[0]) && isvalid(cfg.scale[1]) && isvalid(cfg.scale[2])
						)
					{
						offset.set_vector( cfg.offset[0], cfg.offset[1], cfg.offset[2] );
						scale.set_vector( cfg.scale[0], cfg.scale[1], cfg.scale[2] );
						sensor->calibrated = true;
					}
					else
						sensor->calibrated = false;
				}
				else
				{
					xSemaphoreGive(MagnetometersMutex);
					return false;
				}
				
				//写入更新时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//写入传感器数据
				sensor->have_temperature = true;
				sensor->temperature = temperature;
				sensor->data_error = data_error;
				sensor->data_raw = data;
				double temperature_err = temperature - cfg.STTemperature;
				sensor->data_rawTC.set_vector(
					sensor->data_raw.x - temperature_err*cfg.TemperatureCoefficient[0],
					sensor->data_raw.y - temperature_err*cfg.TemperatureCoefficient[1],
					sensor->data_raw.z - temperature_err*cfg.TemperatureCoefficient[2] );
				double sinDir, cosDir;
				switch(index)
				{
					case 0:
						sinDir = mag0SinDir;
						cosDir = mag0CosDir;
						break;
					case 1:
						sinDir = mag1SinDir;
						cosDir = mag1CosDir;
						break;
					case Internal_Magnetometer_Index:
						sinDir = fcSinDir;
						cosDir = fcCosDir;
						break;
					default:
						sinDir = 0;
						cosDir = 1;
						break;
				}
				getCompVec( sensor->data, sensor->data_rawTC, sensor->sensitivity, scale, offset, sinDir, cosDir );
				
				xSemaphoreGive(MagnetometersMutex);				
				return true;
			}
			return false;
		}
	/*IMU传感器更新函数*/
		
	/*IMU传感器读取函数*/
		bool GetAccelerometer( uint8_t index, IMU_Sensor* sensor, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(AccelerometersMutex,TIMEOUT_Ticks) )
			{
				if( Accelerometers[index] == 0 )
				{
					xSemaphoreGive(AccelerometersMutex);
					return false;
				}
				if( Accelerometers[index]->last_update_time.get_pass_time() > 1.0 )
					Accelerometers[index]->sample_time = -1;
				if( Accelerometers[index]->sample_time < 0 )
				{
					xSemaphoreGive(AccelerometersMutex);
					return false;
				}
				*sensor = *Accelerometers[index];
				
				xSemaphoreGive(AccelerometersMutex);
				return true;
			}
			return false;
		}
		bool GetGyroscope( uint8_t index, IMU_Sensor* sensor, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(GyroscopesMutex,TIMEOUT_Ticks) )
			{
				if( Gyroscopes[index] == 0 )
				{
					xSemaphoreGive(GyroscopesMutex);
					return false;
				}
				if( Gyroscopes[index]->last_update_time.get_pass_time() > 1.0 )
					Gyroscopes[index]->sample_time = -1;
				if( Gyroscopes[index]->sample_time < 0 )
				{
					xSemaphoreGive(GyroscopesMutex);
					return false;
				}
				*sensor = *Gyroscopes[index];
				
				xSemaphoreGive(GyroscopesMutex);
				return true;
			}
			return false;
		}
		bool GetMagnetometer( uint8_t index, IMU_Sensor* sensor, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(MagnetometersMutex,TIMEOUT_Ticks) )
			{
				if( Magnetometers[index] == 0 )
				{
					xSemaphoreGive(MagnetometersMutex);
					return false;
				}
				if( Magnetometers[index]->last_update_time.get_pass_time() > 1.0 )
					Magnetometers[index]->sample_time = -1;
				if( Magnetometers[index]->sample_time < 0 )
				{
					xSemaphoreGive(MagnetometersMutex);
					return false;
				}				
				*sensor = *Magnetometers[index];
				
				xSemaphoreGive(MagnetometersMutex);
				return true;
			}
			return false;
		}
	/*IMU传感器读取函数*/
		
/*IMU*/
		
/*双天线传感器*/
	static SemaphoreHandle_t DAOSensorMutex = xSemaphoreCreateMutex();
	static DAO_Sensor* DAO_Sensors[DAO_Sensors_Count] = {0};
	
	/*DAO传感器注册函数*/
		uint32_t DAOSensorRegister( uint8_t index, SName name, vector3<double> st_relPos, bool slam, double delay, double TIMEOUT )
		{
			if( index >= DAO_Sensors_Count )
				return 0;
			if( st_relPos.get_square() < 10*10 )
				return 0;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(DAOSensorMutex,TIMEOUT_Ticks) )
			{
				if( DAO_Sensors[index] != 0 )
				{
					xSemaphoreGive(DAOSensorMutex);
					return 0;
				}
				DAO_Sensors[index] = new DAO_Sensor;
				if( DAO_Sensors[index] == 0 )
				{
					xSemaphoreGive(DAOSensorMutex);
					return 0;
				}
				DAO_Sensor* sensor = DAO_Sensors[index];
				sensor->name = name;
				sensor->available = false;
				sensor->last_update_time = TIME::now();
				sensor->st_relPos = st_relPos;
				sensor->relPos.zero();
				sensor->slam = slam;
				sensor->delay = delay > 0 ? delay : 0;
				//sensor->real_yaw = 0;
				
				xSemaphoreGive(DAOSensorMutex);
				return (uint32_t)(DAO_Sensors[index]);
			}
			return 0;
		}
		bool DAOSensorUnRegister( uint8_t index,uint32_t key, double TIMEOUT )
		{
			if( index >= DAO_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(DAOSensorMutex,TIMEOUT_Ticks) )
			{	//锁定传感器
				if( DAO_Sensors[index] == 0 )
				{	//传感器未注册
					xSemaphoreGive(DAOSensorMutex);
					return false;
				}
				if( (uint32_t)(DAO_Sensors[index]) != key )
				{	//key不正确
					xSemaphoreGive(DAOSensorMutex);
					return false;
				}
				
				DAO_Sensor* sensor = DAO_Sensors[index];
				delete sensor;
				DAO_Sensors[index] = 0;
				
				xSemaphoreGive(DAOSensorMutex);
				return true;
			}	//解锁传感器
			return false;
		}
	/*DAO传感器注册函数*/
		
	/*DAO传感器更新函数*/
		bool DAOSensorUpdate( uint8_t index,uint32_t key, vector3<double> relPos, bool available, double delay, double TIMEOUT )
		{
			if( index >= DAO_Sensors_Count )
				return false;

			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(DAOSensorMutex,TIMEOUT_Ticks) )
			{
				if( DAO_Sensors[index] == 0 )
				{	//传感器未注册
					xSemaphoreGive(DAOSensorMutex);
					return false;
				}
				if( (uint32_t)(DAO_Sensors[index]) != key )
				{	//key不正确
					xSemaphoreGive(DAOSensorMutex);
					return false;
				}
				
				DAO_Sensor* sensor = DAO_Sensors[index];
				
				//写入更新时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//写入传感器数据
				sensor->available = available;
				sensor->relPos = relPos;
				if( delay >= 0 )
					sensor->delay = delay;
				
				xSemaphoreGive(DAOSensorMutex);
				return true;
			}
			return false;
		}
		bool DAOSensorSetInavailable( uint8_t index,uint32_t key, double TIMEOUT )
		{
			if( index >= DAO_Sensors_Count )
				return false;

			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(DAOSensorMutex,TIMEOUT_Ticks) )
			{
				if( DAO_Sensors[index] == 0 )
				{	//传感器未注册
					xSemaphoreGive(DAOSensorMutex);
					return false;
				}
				if( (uint32_t)(DAO_Sensors[index]) != key )
				{	//key不正确
					xSemaphoreGive(DAOSensorMutex);
					return false;
				}
				
				DAO_Sensor* sensor = DAO_Sensors[index];
				
				//写入更新时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//写入传感器数据
				sensor->available = false;
				
				xSemaphoreGive(DAOSensorMutex);
				return true;
			}
			return false;
		}
	/*DAO传感器更新函数*/
		
	/*DAO传感器读取函数*/
		bool GetDAOSensor( uint8_t index, DAO_Sensor* sensor, double TIMEOUT )
		{
			if( index >= DAO_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(DAOSensorMutex,TIMEOUT_Ticks) )
			{
				if( DAO_Sensors[index] == 0 )
				{
					xSemaphoreGive(DAOSensorMutex);
					return false;
				}
				if( DAO_Sensors[index]->last_update_time.get_pass_time() > 1.5 )
					DAO_Sensors[index]->available = false;
				*sensor = *DAO_Sensors[index];
				
				xSemaphoreGive(DAOSensorMutex);
				return true;
			}
			return false;
		}
	/*DAO传感器读取函数*/
/*双天线传感器*/
		
/*位置传感器*/
		
	const uint8_t default_insAlign_sensor_index = 12;
		
	const uint8_t default_ultrasonic_sensor_index = 1;
	const uint8_t default_mmWaveRadarAH_sensor_index = 10;
	const uint8_t default_optical_flow_index = 8;
	const uint8_t external_baro_sensor_index = 3;
	const uint8_t internal_baro_sensor_index = 4;
	const uint8_t default_rtk_sensor_index = 6;
	const uint8_t default_gps_sensor_index = 7;
	const uint8_t default_uwb_sensor_index = 9;
		
	static SemaphoreHandle_t Position_Sensors_Mutex[Position_Sensors_Count] = {0};
	static Position_Sensor* Position_Sensors[Position_Sensors_Count];
		
	/*位置传感器注册函数*/
		uint32_t PositionSensorRegister( 
			uint8_t index ,\
			SName name ,\
			Position_Sensor_Type sensor_type ,\
			Position_Sensor_DataType sensor_data_type ,\
			Position_Sensor_frame sensor_frame ,\
			double delay ,\
			double xy_trustD , \
			double z_trustD , \
			const double* addition_inf,	\
			double xy_LTtrustD ,\
			double z_LTtrustD , \
			double TIMEOUT \
		)
		{
			if( index >= Position_Sensors_Count )
				return false;			
							
			if( delay < 0 )
				return false;
			if( xy_trustD < 0 )
				return false;
			if( z_trustD < 0 )
				return false;
			
			Quaternion quat;
			get_AirframeY_quat(&quat);
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{
				if( Position_Sensors[index] != 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return 0;
				}
				switch(sensor_frame)
				{
					case Position_Sensor_frame_ENU:
					case Position_Sensor_frame_BodyHeading:
						break;
					default:
					{
						xSemaphoreGive(Position_Sensors_Mutex[index]);
						return 0;
					}
				}
				
				Position_Sensors[index] = new Position_Sensor;
				Position_Sensor* sensor = Position_Sensors[index];
				sensor->data.available = false;
				sensor->data.available_status_update_time = TIME::now();
				sensor->data.last_update_time = TIME::now();
				sensor->data.sensor_type = sensor_type;
				sensor->data.sensor_DataType = sensor_data_type;
				sensor->data.data_frame = sensor_frame;
				sensor->data.delay = delay;
				sensor->data.xy_trustD = xy_trustD;
				sensor->data.z_trustD = z_trustD;
				sensor->data.sample_time = -1;
				sensor->data.mp.initialized = false;				
				sensor->data.data_quat = quat;
				if( xy_LTtrustD >= 0 )
					sensor->data.xy_LTtrustD = xy_LTtrustD;
				else
					sensor->data.xy_LTtrustD = xy_trustD;
				if( z_LTtrustD >= 0 )
					sensor->data.z_LTtrustD = z_LTtrustD;
				else
					sensor->data.z_LTtrustD = z_trustD;
				
				//记录附加信息
				sensor->inf.name = name;
				if(addition_inf)
					memcpy( sensor->inf.addition_inf, addition_inf, 8*sizeof(double) );
				else
					memset( sensor->inf.addition_inf, 0, 8*sizeof(double) );
				
				uint32_t key = (uint32_t)sensor;
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return key;
			}	//解锁传感器
			return 0;
		}
		uint32_t PositionSlamSensorRegister( 
			uint8_t index ,\
			SName name ,\
			Position_Sensor_Type sensor_type ,\
			Position_Sensor_DataType sensor_data_type ,\
			Position_Sensor_frame sensor_frame ,\
			double delay ,\
			double angleOffset ,\
			double xy_trustD , \
			double z_trustD , \
			const double* addition_inf,	\
			double xy_LTtrustD ,\
			double z_LTtrustD , \
			double TIMEOUT \
		)
		{
			if( index >= Position_Sensors_Count )
				return false;			
							
			if( delay < 0 )
				return false;
			if( xy_trustD < 0 )
				return false;
			if( z_trustD < 0 )
				return false;
			
			Quaternion quat;
			get_AirframeY_quat(&quat);
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{
				if( Position_Sensors[index] != 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return 0;
				}
				switch(sensor_frame)
				{
					case Position_Sensor_frame_SLAM:
					case Position_Sensor_frame_PSlamVFlu:
						break;
					default:
					{
						xSemaphoreGive(Position_Sensors_Mutex[index]);
						return 0;
					}
				}
				
				Position_Sensors[index] = new Position_Sensor;
				Position_Sensor* sensor = Position_Sensors[index];
				sensor->data.available = false;
				sensor->data.available_status_update_time = TIME::now();
				sensor->data.last_update_time = TIME::now();
				sensor->data.sensor_type = sensor_type;
				sensor->data.sensor_DataType = sensor_data_type;
				sensor->data.data_frame = sensor_frame;
				sensor->data.mp.lat0_rad = angleOffset;
				sensor->data.mp.lon0_rad = 0;
				sensor->data.delay = delay;
				sensor->data.xy_trustD = xy_trustD;
				sensor->data.z_trustD = z_trustD;
				sensor->data.sample_time = -1;
				sensor->data.mp.initialized = false;				
				sensor->data.data_quat = quat;
				if( xy_LTtrustD >= 0 )
					sensor->data.xy_LTtrustD = xy_LTtrustD;
				else
					sensor->data.xy_LTtrustD = xy_trustD;
				if( z_LTtrustD >= 0 )
					sensor->data.z_LTtrustD = z_LTtrustD;
				else
					sensor->data.z_LTtrustD = z_trustD;
				
				//记录附加信息
				sensor->inf.name = name;
				if(addition_inf)
					memcpy( sensor->inf.addition_inf, addition_inf, 8*sizeof(double) );
				else
					memset( sensor->inf.addition_inf, 0, 8*sizeof(double) );
				
				uint32_t key = (uint32_t)sensor;
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return key;
			}	//解锁传感器
			return 0;
		}
		bool PositionSensorUnRegister( uint8_t index,uint32_t key, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//锁定传感器
				if( Position_Sensors[index]==0 || ( key!=POSOVERIDEKEY && (uint32_t)Position_Sensors[index]!=key ) )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				delete sensor;
				Position_Sensors[index] = 0;
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//解锁传感器
			return false;
		}
	/*位置传感器注册函数*/
	
	//更改位置传感器DataType
	bool PositionSensorChangeDataType( uint8_t index,uint32_t key, Position_Sensor_DataType datatype, double TIMEOUT )
	{
		if( index >= Position_Sensors_Count )
			return false;
		
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
		{	//锁定传感器
			if( Position_Sensors[index]==0 || ( key!=POSOVERIDEKEY && (uint32_t)Position_Sensors[index]!=key ) )
			{
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return false;
			}
			
			Position_Sensor* sensor = Position_Sensors[index];
			sensor->data.sensor_DataType = datatype;
			
			xSemaphoreGive(Position_Sensors_Mutex[index]);
			return true;
		}	//解锁传感器
		return false;
	}
		
	
	//重置SLAM传感器航向角
	bool PositionSlamSensorResetAngleOffset( uint8_t index,uint32_t key, double angleOffset , double TIMEOUT )
	{
		if( index >= Position_Sensors_Count )
			return false;

		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;

		if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
		{	//锁定传感器
			if( Position_Sensors[index]==0 || ( key!=POSOVERIDEKEY && (uint32_t)Position_Sensors[index]!=key ) )
			{
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return false;
			}
			
			Position_Sensor* sensor = Position_Sensors[index];
			bool res = false;
			
			if( sensor->data.data_frame==Position_Sensor_frame_SLAM || sensor->data.data_frame==Position_Sensor_frame_PSlamVFlu )
			{
				sensor->data.mp.lat0_rad = angleOffset;
				sensor->data.mp.lon0_rad = 0;
				res = true;
			}
			
			xSemaphoreGive(Position_Sensors_Mutex[index]);
			return res;
		}	//解锁传感器
		return false;
	}
	
	/*位置传感器更新函数*/
		
		//失能位置传感器
		bool PositionSensorSetInavailable( uint8_t index,uint32_t key, const double* addition_inf, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			bool unLocked;
			is_Attitude_Control_Enabled(&unLocked);
			uint64_t log = 0;
			ReadParam( "SDLog_PosSensor", 0, 0, (uint64_t*)&log, 0 );
			
			Quaternion quat;
			get_AirframeY_quat(&quat);
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//锁定传感器
				if( Position_Sensors[index]==0 || ( key!=POSOVERIDEKEY && (uint32_t)Position_Sensors[index]!=key ) )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				//更新可用状态
				if( sensor->data.available != false )
					sensor->data.available_status_update_time = TIME::now();
				sensor->data.available = false;
				//更新采样时间
				sensor->data.sample_time = sensor->data.last_update_time.get_pass_time_st();
				//复位姿态
				sensor->data.data_quat = quat;
				
				//记录附加信息
				if(addition_inf)
					memcpy( sensor->inf.addition_inf, addition_inf, 8*sizeof(double) );
				
				//记录位置数据
				if(unLocked && log)
					SDLog_Msg_PosSensor( index, *sensor );
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);				
				return true;
			}	//解锁传感器
			return false;
		}
	
		bool PositionSensorUpdatePositionGlobal( uint8_t index,uint32_t key, vector3<double> position_Global, bool available, double delay, double xy_trustD, double z_trustD, const double* addition_inf, double xy_LTtrustD, double z_LTtrustD, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			bool unLocked;
			is_Attitude_Control_Enabled(&unLocked);
			uint64_t log = 0;
			ReadParam( "SDLog_PosSensor", 0, 0, (uint64_t*)&log, 0 );
			
			//读取偏移校准
			char ofs_ind[2] = {0};
			ofs_ind[0] = index + '0';
			SName ofs_name = SName("POfs_S")+SName(ofs_ind);
			float ofs_value_x[2]={0};					
			float ofs_value_y[2]={0};	
			float ofs_value_z[2]={0};	
			ReadParam( ofs_name+SName("_x"), 0, 0, (uint64_t*)ofs_value_x, 0 );
			ReadParam( ofs_name+SName("_y"), 0, 0, (uint64_t*)ofs_value_y, 0 );
			ReadParam( ofs_name+SName("_z"), 0, 0, (uint64_t*)ofs_value_z, 0 );
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			
			//获取历史姿态
			if( delay >= 0 )
			{
				if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
				{	//锁定传感器获取延时时间
					Position_Sensor* sensor = Position_Sensors[index];
					delay = sensor->data.delay;
					
					xSemaphoreGive(Position_Sensors_Mutex[index]);
				}
				else
					return false;
			}
			//获取延时补偿姿态
			Quaternion quat;
			get_history_AirframeQuatY( &quat, delay );		
			
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//锁定传感器
				if( Position_Sensors[index]==0 || ( key!=POSOVERIDEKEY && (uint32_t)Position_Sensors[index]!=key ) )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				
				//判断传感器类型、数据是否正确
				bool data_effective;
				switch( sensor->data.sensor_DataType )
				{
					case Position_Sensor_DataType_s_xy:
					case Position_Sensor_DataType_s_xy_nAC:
						if( __ARM_isnan( position_Global.x ) || __ARM_isnan( position_Global.y ) || \
								__ARM_isinf( position_Global.x ) || __ARM_isinf( position_Global.y ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_s_z:
					case Position_Sensor_DataType_s_z_nAC:
						if( __ARM_isnan( position_Global.z ) || __ARM_isinf( position_Global.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_s_xyz:
					case Position_Sensor_DataType_s_xyz_nAC:
						if( __ARM_isnan( position_Global.x ) || __ARM_isnan( position_Global.y ) || __ARM_isnan( position_Global.z ) || \
								__ARM_isinf( position_Global.x ) || __ARM_isinf( position_Global.y ) || __ARM_isinf( position_Global.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					default:
						data_effective = false;
						break;
				}
				if( !data_effective )
				{	//数据出错退出
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}								
				
				//更新可用状态
				if( sensor->data.available != available )
					sensor->data.available_status_update_time = TIME::now();
				sensor->data.available = available;
				
				//更新采样时间
				sensor->data.sample_time = sensor->data.last_update_time.get_pass_time_st();				
				//更新延时时间
				if( delay >= 0 )
					sensor->data.delay = delay;
				//更新信任度
				if( xy_trustD >= 0 )
					sensor->data.xy_trustD = xy_trustD;
				if( z_trustD >= 0 )
					sensor->data.z_trustD = z_trustD;
				//更新长期信任度
				if( xy_LTtrustD >= 0 )
					sensor->data.xy_LTtrustD = xy_LTtrustD;
				if( z_LTtrustD >= 0 )
					sensor->data.z_LTtrustD = z_LTtrustD;
				
				//更新数据
				sensor->data.position_Global = position_Global;				
				
				//经纬坐标平面投影
				if( available )
				{					
					if( sensor->data.mp.initialized == false )
					{
						map_projection_init( &sensor->data.mp , position_Global.x , position_Global.y );
					}
					double pos_x , pos_y;
					map_projection_project( &sensor->data.mp , position_Global.x , position_Global.y , &pos_x , &pos_y );
					
					vector3<double> offset_comp = quat.rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) );
					sensor->data.position.x = pos_x - offset_comp.x;
					sensor->data.position.y = pos_y - offset_comp.y;
					sensor->data.position.z = position_Global.z - offset_comp.z;
				}
					
				//记录姿态
				sensor->data.data_quat = quat;
				
				//记录附加信息
				if(addition_inf)
					memcpy( sensor->inf.addition_inf, addition_inf, 8*sizeof(double) );
				
				//记录位置数据
				if(unLocked && log)
					SDLog_Msg_PosSensor( index, *sensor );
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//解锁传感器
			return false;
		}
		bool PositionSensorUpdatePosition( uint8_t index,uint32_t key, vector3<double> position, bool available, double delay, double xy_trustD, double z_trustD, const double* addition_inf, double xy_LTtrustD, double z_LTtrustD, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			bool unLocked;
			is_Attitude_Control_Enabled(&unLocked);
			uint64_t log = 0;
			ReadParam( "SDLog_PosSensor", 0, 0, (uint64_t*)&log, 0 );
			
			//读取偏移校准
			char ofs_ind[2] = {0};
			ofs_ind[0] = index + '0';
			SName ofs_name = SName("POfs_S")+SName(ofs_ind);
			float ofs_value_x[2]={0};					
			float ofs_value_y[2]={0};	
			float ofs_value_z[2]={0};	
			ReadParam( ofs_name+SName("_x"), 0, 0, (uint64_t*)ofs_value_x, 0 );
			ReadParam( ofs_name+SName("_y"), 0, 0, (uint64_t*)ofs_value_y, 0 );
			ReadParam( ofs_name+SName("_z"), 0, 0, (uint64_t*)ofs_value_z, 0 );
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			
			//获取历史姿态
			if( delay >= 0 )
			{
				if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
				{	//锁定传感器获取延时时间
					Position_Sensor* sensor = Position_Sensors[index];
					delay = sensor->data.delay;
					
					xSemaphoreGive(Position_Sensors_Mutex[index]);
				}
				else
					return false;
			}
			//获取延时补偿姿态
			Quaternion quat;
			get_history_AirframeQuatY( &quat, delay );
			
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//锁定传感器
				if( Position_Sensors[index]==0 || ( key!=POSOVERIDEKEY && (uint32_t)Position_Sensors[index]!=key ) )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				//判断传感器类型、数据是否正确
				bool data_effective;
				switch( sensor->data.sensor_DataType )
				{
					case Position_Sensor_DataType_s_xy:
					case Position_Sensor_DataType_s_xy_nAC:
						if( __ARM_isnan( position.x ) || __ARM_isnan( position.y ) || \
								__ARM_isinf( position.x ) || __ARM_isinf( position.y ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_s_z:
					case Position_Sensor_DataType_s_z_nAC:
						if( __ARM_isnan( position.z ) || __ARM_isinf( position.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_s_xyz:
					case Position_Sensor_DataType_s_xyz_nAC:
						if( __ARM_isnan( position.x ) || __ARM_isnan( position.y ) || __ARM_isnan( position.z ) || \
								__ARM_isinf( position.x ) || __ARM_isinf( position.y ) || __ARM_isinf( position.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					default:
						data_effective = false;
						break;
				}
				if( !data_effective )
				{	//数据出错退出
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				//更新可用状态
				if( sensor->data.available != available )
					sensor->data.available_status_update_time = TIME::now();
				sensor->data.available = available;
				
				//更新采样时间
				sensor->data.sample_time = sensor->data.last_update_time.get_pass_time_st();	
				//延时大于0更新延时
				if( delay > 0 )
					sensor->data.delay = delay;
				//更新信任度
				if( xy_trustD >= 0 )
					sensor->data.xy_trustD = xy_trustD;
				if( z_trustD >= 0 )
					sensor->data.z_trustD = z_trustD;		
				//更新长期信任度
				if( xy_LTtrustD >= 0 )
					sensor->data.xy_LTtrustD = xy_LTtrustD;
				if( z_LTtrustD >= 0 )
					sensor->data.z_LTtrustD = z_LTtrustD;				
				
				//更新数据
				vector3<double> offset_comp = quat.rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) );
				switch(sensor->data.data_frame)
				{
					case Position_Sensor_frame_SLAM:
					case Position_Sensor_frame_PSlamVFlu:
					{
						double yaw = sensor->data.mp.lat0_rad + sensor->data.mp.lon0_rad;
						double sin_Yaw, cos_Yaw;
						fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
						double posX = position.x;
						double posY = position.y;
						position.x = BodyHeading2ENU_x( posX , posY , sin_Yaw , cos_Yaw );
						position.y = BodyHeading2ENU_y( posX , posY , sin_Yaw , cos_Yaw );
						break;
					}
					default:
						break;
				}
				sensor->data.position = position - offset_comp;					
				
				//记录姿态
				sensor->data.data_quat = quat;
				
				//记录附加信息
				if(addition_inf)
					memcpy( sensor->inf.addition_inf, addition_inf, 8*sizeof(double) );
				
				//记录位置数据
				if(unLocked && log)
					SDLog_Msg_PosSensor( index, *sensor );
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//解锁传感器
			return false;
		}
		bool PositionSensorUpdatePositionGlobalVel( uint8_t index,uint32_t key, vector3<double> position_Global, vector3<double> vel, bool available, double delay, double xy_trustD, double z_trustD, const double* addition_inf, double xy_LTtrustD, double z_LTtrustD, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			bool unLocked;
			is_Attitude_Control_Enabled(&unLocked);
			uint64_t log = 0;
			ReadParam( "SDLog_PosSensor", 0, 0, (uint64_t*)&log, 0 );
			
			//读取偏移校准
			char ofs_ind[2] = {0};
			ofs_ind[0] = index + '0';
			SName ofs_name = SName("POfs_S")+SName(ofs_ind);
			float ofs_value_x[2]={0};					
			float ofs_value_y[2]={0};	
			float ofs_value_z[2]={0};	
			ReadParam( ofs_name+SName("_x"), 0, 0, (uint64_t*)ofs_value_x, 0 );
			ReadParam( ofs_name+SName("_y"), 0, 0, (uint64_t*)ofs_value_y, 0 );
			ReadParam( ofs_name+SName("_z"), 0, 0, (uint64_t*)ofs_value_z, 0 );
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			
			//获取历史姿态
			if( delay >= 0 )
			{
				if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
				{	//锁定传感器获取延时时间
					Position_Sensor* sensor = Position_Sensors[index];
					delay = sensor->data.delay;
					
					xSemaphoreGive(Position_Sensors_Mutex[index]);
				}
				else
					return false;
			}
			//获取延时补偿姿态
			Quaternion quat;
			get_history_AirframeQuatY( &quat, delay );
			
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//锁定传感器
				if( Position_Sensors[index]==0 || ( key!=POSOVERIDEKEY && (uint32_t)Position_Sensors[index]!=key ) )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				//判断传感器类型、数据是否正确
				bool data_effective;
				switch( sensor->data.sensor_DataType )
				{
					case Position_Sensor_DataType_sv_xy:
					case Position_Sensor_DataType_sv_xy_nAC:
						if( __ARM_isnan( position_Global.x ) || __ARM_isnan( position_Global.y ) || \
								__ARM_isinf( position_Global.x ) || __ARM_isinf( position_Global.y ) || \
								__ARM_isnan( vel.x ) || __ARM_isnan( vel.y ) || \
								__ARM_isinf( vel.x ) || __ARM_isinf( vel.y ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_sv_z:
					case Position_Sensor_DataType_sv_z_nAC:
						if( __ARM_isnan( position_Global.z ) || __ARM_isinf( position_Global.z ) || \
								__ARM_isnan( vel.z ) || __ARM_isinf( vel.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_sv_xyz:
					case Position_Sensor_DataType_sv_xyz_nAC:
						if( __ARM_isnan( position_Global.x ) || __ARM_isnan( position_Global.y ) || __ARM_isnan( position_Global.z ) || \
								__ARM_isinf( position_Global.x ) || __ARM_isinf( position_Global.y ) || __ARM_isinf( position_Global.z ) || \
								__ARM_isnan( vel.x ) || __ARM_isnan( vel.y ) || __ARM_isnan( vel.z ) || \
								__ARM_isinf( vel.x ) || __ARM_isinf( vel.y ) || __ARM_isinf( vel.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					default:
						data_effective = false;
						break;
				}
				if( !data_effective )
				{	//数据出错退出
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				//更新可用状态
				if( sensor->data.available != available )
					sensor->data.available_status_update_time = TIME::now();
				sensor->data.available = available;
				
				//更新采样时间
				sensor->data.sample_time = sensor->data.last_update_time.get_pass_time_st();				
				//更新延时时间
				if( delay > 0 )
					sensor->data.delay = delay;
				//更新信任度
				if( xy_trustD >= 0 )
					sensor->data.xy_trustD = xy_trustD;
				if( z_trustD >= 0 )
					sensor->data.z_trustD = z_trustD;
				//更新长期信任度
				if( xy_LTtrustD >= 0 )
					sensor->data.xy_LTtrustD = xy_LTtrustD;
				if( z_LTtrustD >= 0 )
					sensor->data.z_LTtrustD = z_LTtrustD;
				
				//更新数据
				sensor->data.position_Global = position_Global;
				vector3<double> offset_comp = quat.rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) );
				vector3<double> d_offset;
				switch(sensor->data.data_frame)
				{
					case Position_Sensor_frame_ENU:
					{
						d_offset = offset_comp - sensor->data.data_quat.rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) );
						break;
					}
					case Position_Sensor_frame_BodyHeading:
					{
						d_offset = quat.get_RP_quat().rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) )
										-sensor->data.data_quat.get_RP_quat().rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) );
						break;
					}
					default:
					{
						xSemaphoreGive(Position_Sensors_Mutex[index]);
						return false;
					}
				}
				
				double sample_freq = 0;
				if( sensor->data.sample_time > 0.0001 )
					sample_freq = 1.0 / sensor->data.sample_time;
				sensor->data.velocity = vel - d_offset*sample_freq;
				//经纬坐标平面投影				
				if( available )
				{
					if( sensor->data.mp.initialized == false )
					{
						map_projection_init( &sensor->data.mp , position_Global.x , position_Global.y );
					}
					double pos_x , pos_y;
					map_projection_project( &sensor->data.mp , position_Global.x , position_Global.y , &pos_x , &pos_y );
					
					sensor->data.position.x = pos_x - offset_comp.x;
					sensor->data.position.y = pos_y - offset_comp.y;
					sensor->data.position.z = position_Global.z - offset_comp.z;
				}
					
				//记录姿态
				sensor->data.data_quat = quat;
				
				//记录附加信息
				if(addition_inf)
					memcpy( sensor->inf.addition_inf, addition_inf, 8*sizeof(double) );
				
				//记录位置数据
				if(unLocked && log)
					SDLog_Msg_PosSensor( index, *sensor );
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//解锁传感器
			return false;
		}
		bool PositionSensorUpdatePositionVel( uint8_t index,uint32_t key, vector3<double> position, vector3<double> vel, bool available, double delay, double xy_trustD, double z_trustD, const double* addition_inf, double xy_LTtrustD, double z_LTtrustD, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			bool unLocked;
			is_Attitude_Control_Enabled(&unLocked);
			uint64_t log = 0;
			ReadParam( "SDLog_PosSensor", 0, 0, (uint64_t*)&log, 0 );
			
			//读取偏移校准
			char ofs_ind[2] = {0};
			ofs_ind[0] = index + '0';
			SName ofs_name = SName("POfs_S")+SName(ofs_ind);
			float ofs_value_x[2]={0};					
			float ofs_value_y[2]={0};	
			float ofs_value_z[2]={0};	
			ReadParam( ofs_name+SName("_x"), 0, 0, (uint64_t*)ofs_value_x, 0 );
			ReadParam( ofs_name+SName("_y"), 0, 0, (uint64_t*)ofs_value_y, 0 );
			ReadParam( ofs_name+SName("_z"), 0, 0, (uint64_t*)ofs_value_z, 0 );
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			
			//获取历史姿态
			if( delay >= 0 )
			{
				if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
				{	//锁定传感器获取延时时间
					Position_Sensor* sensor = Position_Sensors[index];
					delay = sensor->data.delay;
					
					xSemaphoreGive(Position_Sensors_Mutex[index]);
				}
				else
					return false;
			}
			//获取延时补偿姿态
			Quaternion quat;
			get_history_AirframeQuatY( &quat, delay );
			
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//锁定传感器
				if( Position_Sensors[index]==0 || ( key!=POSOVERIDEKEY && (uint32_t)Position_Sensors[index]!=key ) )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				bool data_effective;
				switch( sensor->data.sensor_DataType )
				{
					case Position_Sensor_DataType_sv_xy:
					case Position_Sensor_DataType_sv_xy_nAC:
						if( __ARM_isnan( position.x ) || __ARM_isnan( position.y ) || \
								__ARM_isinf( position.x ) || __ARM_isinf( position.y ) || \
								__ARM_isnan( vel.x ) || __ARM_isnan( vel.y ) || \
								__ARM_isinf( vel.x ) || __ARM_isinf( vel.y ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_sv_z:
					case Position_Sensor_DataType_sv_z_nAC:
						if( __ARM_isnan( position.z ) || __ARM_isinf( position.z ) || \
								__ARM_isnan( vel.z ) || __ARM_isinf( vel.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_sv_xyz:
					case Position_Sensor_DataType_sv_xyz_nAC:
						if( __ARM_isnan( position.x ) || __ARM_isnan( position.y ) || __ARM_isnan( position.z ) || \
								__ARM_isinf( position.x ) || __ARM_isinf( position.y ) || __ARM_isinf( position.z ) || \
								__ARM_isnan( vel.x ) || __ARM_isnan( vel.y ) || __ARM_isnan( vel.z ) || \
								__ARM_isinf( vel.x ) || __ARM_isinf( vel.y ) || __ARM_isinf( vel.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					default:
						data_effective = false;
						break;
				}
				if( !data_effective )
				{	//数据出错退出
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				//更新可用状态
				if( sensor->data.available != available )
					sensor->data.available_status_update_time = TIME::now();
				sensor->data.available = available;
				
				//更新采样时间
				sensor->data.sample_time = sensor->data.last_update_time.get_pass_time_st();							
				//延时大于0更新延时
				if( delay > 0 )
					sensor->data.delay = delay;
				//更新信任度
				if( xy_trustD >= 0 )
					sensor->data.xy_trustD = xy_trustD;
				if( z_trustD >= 0 )
					sensor->data.z_trustD = z_trustD;
				//更新长期信任度
				if( xy_LTtrustD >= 0 )
					sensor->data.xy_LTtrustD = xy_LTtrustD;
				if( z_LTtrustD >= 0 )
					sensor->data.z_LTtrustD = z_LTtrustD;
				
				//更新数据
				vector3<double> offset_comp = quat.rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) );
				vector3<double> d_offset;
				switch(sensor->data.data_frame)
				{
					case Position_Sensor_frame_SLAM:
					{
						double yaw = sensor->data.mp.lat0_rad + sensor->data.mp.lon0_rad;
						double sin_Yaw, cos_Yaw;
						fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
						double velX = vel.x;
						double velY = vel.y;
						vel.x = BodyHeading2ENU_x( velX , velY , sin_Yaw , cos_Yaw );
						vel.y = BodyHeading2ENU_y( velX , velY , sin_Yaw , cos_Yaw );
						double posX = position.x;
						double posY = position.y;
						position.x = BodyHeading2ENU_x( posX , posY , sin_Yaw , cos_Yaw );
						position.y = BodyHeading2ENU_y( posX , posY , sin_Yaw , cos_Yaw );
					}
					case Position_Sensor_frame_ENU:
					{
						d_offset = offset_comp - sensor->data.data_quat.rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) );
						break;
					}
					
					case Position_Sensor_frame_PSlamVFlu:
					{
						double yaw = sensor->data.mp.lat0_rad + sensor->data.mp.lon0_rad;
						double sin_Yaw, cos_Yaw;
						fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
						double posX = position.x;
						double posY = position.y;
						position.x = BodyHeading2ENU_x( posX , posY , sin_Yaw , cos_Yaw );
						position.y = BodyHeading2ENU_y( posX , posY , sin_Yaw , cos_Yaw );
					}
					case Position_Sensor_frame_BodyHeading:
					{
						d_offset = quat.get_RP_quat().rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) )
										-sensor->data.data_quat.get_RP_quat().rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) );
						break;
					}
				}
				double sample_freq = 0;
				if( sensor->data.sample_time > 0.0001 )
					sample_freq = 1.0 / sensor->data.sample_time;
				sensor->data.position = position - offset_comp;				
				sensor->data.velocity = vel - d_offset*sample_freq;
				
				//记录姿态
				sensor->data.data_quat = quat;
				
				//记录附加信息
				if(addition_inf)
					memcpy( sensor->inf.addition_inf, addition_inf, 8*sizeof(double) );
				
				//记录位置数据
				if(unLocked && log)
					SDLog_Msg_PosSensor( index, *sensor );
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//解锁传感器
			return false;
		}
		bool PositionSensorUpdateVel( uint8_t index,uint32_t key, vector3<double> vel, bool available, double delay, double xy_trustD, double z_trustD, const double* addition_inf, double xy_LTtrustD, double z_LTtrustD, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			bool unLocked;
			is_Attitude_Control_Enabled(&unLocked);
			uint64_t log = 0;
			ReadParam( "SDLog_PosSensor", 0, 0, (uint64_t*)&log, 0 );
			
			//读取偏移校准
			char ofs_ind[2] = {0};
			ofs_ind[0] = index + '0';
			SName ofs_name = SName("POfs_S")+SName(ofs_ind);
			float ofs_value_x[2]={0};					
			float ofs_value_y[2]={0};	
			float ofs_value_z[2]={0};	
			ReadParam( ofs_name+SName("_x"), 0, 0, (uint64_t*)ofs_value_x, 0 );
			ReadParam( ofs_name+SName("_y"), 0, 0, (uint64_t*)ofs_value_y, 0 );
			ReadParam( ofs_name+SName("_z"), 0, 0, (uint64_t*)ofs_value_z, 0 );
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			
			//获取历史姿态
			if( delay >= 0 )
			{
				if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
				{	//锁定传感器获取延时时间
					Position_Sensor* sensor = Position_Sensors[index];
					delay = sensor->data.delay;
					
					xSemaphoreGive(Position_Sensors_Mutex[index]);
				}
				else
					return false;
			}
			//获取延时补偿姿态
			Quaternion quat;
			get_history_AirframeQuatY( &quat, delay );
			
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//锁定传感器
				if( Position_Sensors[index]==0 || ( key!=POSOVERIDEKEY && (uint32_t)Position_Sensors[index]!=key ) )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				
				//判断传感器类型、数据是否正确
				bool data_effective;
				switch( sensor->data.sensor_DataType )
				{
					case Position_Sensor_DataType_v_xy:
					case Position_Sensor_DataType_v_xy_nAC:
						if( __ARM_isnan( vel.x ) || __ARM_isnan( vel.y ) || \
								__ARM_isinf( vel.x ) || __ARM_isinf( vel.y ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_v_z:
					case Position_Sensor_DataType_v_z_nAC:
						if( __ARM_isnan( vel.z ) || __ARM_isinf( vel.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_v_xyz:
					case Position_Sensor_DataType_v_xyz_nAC:
						if( __ARM_isnan( vel.x ) || __ARM_isnan( vel.y ) || __ARM_isnan( vel.z ) || \
								__ARM_isinf( vel.x ) || __ARM_isinf( vel.y ) || __ARM_isinf( vel.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					default:
						data_effective = false;
						break;
				}
				if( !data_effective )
				{	//数据出错退出
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				//更新可用状态
				if( sensor->data.available != available )
					sensor->data.available_status_update_time = TIME::now();
				sensor->data.available = available;
				
				//更新采样时间
				sensor->data.sample_time = sensor->data.last_update_time.get_pass_time_st();							
				//延时大于0更新延时
				if( delay > 0 )
					sensor->data.delay = delay;
				//更新信任度
				if( xy_trustD >= 0 )
					sensor->data.xy_trustD = xy_trustD;
				if( z_trustD >= 0 )
					sensor->data.z_trustD = z_trustD;
				//更新长期信任度
				if( xy_LTtrustD >= 0 )
					sensor->data.xy_LTtrustD = xy_LTtrustD;
				if( z_LTtrustD >= 0 )
					sensor->data.z_LTtrustD = z_LTtrustD;
				
				//更新数据
				vector3<double> offset_comp = quat.rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) );
				vector3<double> d_offset;
				switch(sensor->data.data_frame)
				{
					case Position_Sensor_frame_SLAM:
					{
						double yaw = sensor->data.mp.lat0_rad + sensor->data.mp.lon0_rad;
						double sin_Yaw, cos_Yaw;
						fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
						double velX = vel.x;
						double velY = vel.y;
						vel.x = BodyHeading2ENU_x( velX , velY , sin_Yaw , cos_Yaw );
						vel.y = BodyHeading2ENU_y( velX , velY , sin_Yaw , cos_Yaw );
					}
					case Position_Sensor_frame_ENU:
					{
						d_offset = offset_comp - sensor->data.data_quat.rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) );
						break;
					}
					case Position_Sensor_frame_PSlamVFlu:
					case Position_Sensor_frame_BodyHeading:
					{
						d_offset = quat.get_RP_quat().rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) )
										-sensor->data.data_quat.get_RP_quat().rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) );
						break;
					}
				}
				double sample_freq = 0;
				if( sensor->data.sample_time > 0.0001 )
					sample_freq = 1.0 / sensor->data.sample_time;		
				sensor->data.velocity = vel - d_offset*sample_freq;
				
				//记录姿态
				sensor->data.data_quat = quat;
				
				//记录附加信息
				if(addition_inf)
					memcpy( sensor->inf.addition_inf, addition_inf, 8*sizeof(double) );
				
				//记录位置数据
				if(unLocked && log)
					SDLog_Msg_PosSensor( index, *sensor );
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//解锁传感器
			return false;
		}
		
		bool __PositionSensorIncreaseSlamAngle( uint8_t index, const double& angle, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;

			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;

			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//锁定传感器
				Position_Sensor* sensor = Position_Sensors[index];
				bool res = false;
				if( sensor->data.data_frame==Position_Sensor_frame_SLAM )
				{
					sensor->data.mp.lon0_rad += angle;
					double sin_Yaw, cos_Yaw;
					fast_sin_cos( angle, &sin_Yaw, &cos_Yaw );
					double velX = sensor->data.velocity.x;
					double velY = sensor->data.velocity.y;
					sensor->data.velocity.x = BodyHeading2ENU_x( velX , velY , sin_Yaw , cos_Yaw );
					sensor->data.velocity.y = BodyHeading2ENU_y( velX , velY , sin_Yaw , cos_Yaw );
					double posX = sensor->data.position.x;
					double posY = sensor->data.position.y;
					sensor->data.position.x = BodyHeading2ENU_x( posX , posY , sin_Yaw , cos_Yaw );
					sensor->data.position.y = BodyHeading2ENU_y( posX , posY , sin_Yaw , cos_Yaw );
					res = true;
				}
				else if( sensor->data.data_frame==Position_Sensor_frame_PSlamVFlu )
				{
					sensor->data.mp.lon0_rad += angle;
					double sin_Yaw, cos_Yaw;
					fast_sin_cos( angle, &sin_Yaw, &cos_Yaw );
					double posX = sensor->data.position.x;
					double posY = sensor->data.position.y;
					sensor->data.position.x = BodyHeading2ENU_x( posX , posY , sin_Yaw , cos_Yaw );
					sensor->data.position.y = BodyHeading2ENU_y( posX , posY , sin_Yaw , cos_Yaw );
					res = true;
				}
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return res;
			}	//解锁传感器
			return false;
		}
	/*位置传感器更新函数*/
		
	/*位置传感器读取函数*/
		bool GetPositionSensor( uint8_t index, Position_Sensor* result_sensor, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//锁定传感器
				if( Position_Sensors[index] == 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				*result_sensor = *sensor;
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//解锁传感器
			return false;
		}
		bool GetPositionSensorData( uint8_t index, Position_Sensor_Data* result_sensor, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//锁定传感器
				if( Position_Sensors[index] == 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				*result_sensor = sensor->data;
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//解锁传感器
			return false;
		}
	/*位置传感器读取函数*/
	
/*位置传感器*/
		
void init_Sensors()
{
	//初始化传感器互斥锁
	for( int i = 0; i < Position_Sensors_Count; ++i )
		Position_Sensors_Mutex[i] = xSemaphoreCreateMutex();
	
	/*注册传感器位置偏移参数*/
		SensorPosOffset initial_posoffset;
		//飞控方向
		initial_posoffset.Fc_dir[0] = 0;
		initial_posoffset.Mag0_dir[0] = 0;
		initial_posoffset.Mag1_dir[0] = 0;
		//飞控偏移
		initial_posoffset.Fc_x[0] = 0;
		initial_posoffset.Fc_y[0] = 0;
		initial_posoffset.Fc_z[0] = 0;
		//传感器0偏移
		initial_posoffset.S0_x[0] = 0;
		initial_posoffset.S0_y[0] = 0;
		initial_posoffset.S0_z[0] = 0;
		//传感器1偏移
		initial_posoffset.S1_x[0] = 0;
		initial_posoffset.S1_y[0] = 0;
		initial_posoffset.S1_z[0] = 0;
		//传感器2偏移
		initial_posoffset.S2_x[0] = 0;
		initial_posoffset.S2_y[0] = 0;
		initial_posoffset.S2_z[0] = 0;
		//传感器3偏移
		initial_posoffset.S3_x[0] = 0;
		initial_posoffset.S3_y[0] = 0;
		initial_posoffset.S3_z[0] = 0;
		//传感器4偏移
		initial_posoffset.S4_x[0] = 0;
		initial_posoffset.S4_y[0] = 0;
		initial_posoffset.S4_z[0] = 0;
		//传感器5偏移
		initial_posoffset.S5_x[0] = 0;
		initial_posoffset.S5_y[0] = 0;
		initial_posoffset.S5_z[0] = 0;
		//传感器6偏移
		initial_posoffset.S6_x[0] = 0;
		initial_posoffset.S6_y[0] = 0;
		initial_posoffset.S6_z[0] = 0;
		//传感器7偏移
		initial_posoffset.S7_x[0] = 0;
		initial_posoffset.S7_y[0] = 0;
		initial_posoffset.S7_z[0] = 0;
		//传感器8偏移
		initial_posoffset.S8_x[0] = 0;
		initial_posoffset.S8_y[0] = 0;
		initial_posoffset.S8_z[0] = 0;
		//传感器9偏移
		initial_posoffset.S9_x[0] = 0;
		initial_posoffset.S9_y[0] = 0;
		initial_posoffset.S9_z[0] = 0;
		//传感器10偏移
		initial_posoffset.S10_x[0] = 0;
		initial_posoffset.S10_y[0] = 0;
		initial_posoffset.S10_z[0] = 0;
		//传感器11偏移
		initial_posoffset.S11_x[0] = 0;
		initial_posoffset.S11_y[0] = 0;
		initial_posoffset.S11_z[0] = 0;
		//传感器12偏移
		initial_posoffset.S12_x[0] = 0;
		initial_posoffset.S12_y[0] = 0;
		initial_posoffset.S12_z[0] = 0;
		//传感器13偏移
		initial_posoffset.S13_x[0] = 0;
		initial_posoffset.S13_y[0] = 0;
		initial_posoffset.S13_z[0] = 0;
		//传感器14偏移
		initial_posoffset.S14_x[0] = 0;
		initial_posoffset.S14_y[0] = 0;
		initial_posoffset.S14_z[0] = 0;
		//传感器15偏移
		initial_posoffset.S15_x[0] = 0;
		initial_posoffset.S15_y[0] = 0;
		initial_posoffset.S15_z[0] = 0;
	
		MAV_PARAM_TYPE param_types[] = {
			//飞控方向
			MAV_PARAM_TYPE_REAL32 ,	//dir
			MAV_PARAM_TYPE_REAL32 ,	//dir
			MAV_PARAM_TYPE_REAL32 ,	//dir
			//飞控偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器0偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器1偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器2偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器3偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器4偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器5偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器6偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器7偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器8偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器9偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器10偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器11偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器12偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器13偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器14偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器15偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
		};
		
		//飞控偏移		
		SName param_names[] = {
			//飞控方向
			"POfs_Fc_Dir" ,	//dir
			"POfs_Mag0_Dir" ,	//dir
			"POfs_Mag1_Dir" ,	//dir
			//飞控偏移
			"POfs_Fc_x" ,	//x
			"POfs_Fc_y" ,	//y
			"POfs_Fc_z" ,	//z
			//传感器0偏移
			"POfs_S0_x" ,	//x
			"POfs_S0_y" ,	//y
			"POfs_S0_z" ,	//z
			//传感器1偏移
			"POfs_S1_x" ,	//x
			"POfs_S1_y" ,	//y
			"POfs_S1_z" ,	//z
			//传感器2偏移
			"POfs_S2_x" ,	//x
			"POfs_S2_y" ,	//y
			"POfs_S2_z" ,	//z
			//传感器3偏移
			"POfs_S3_x" ,	//x
			"POfs_S3_y" ,	//y
			"POfs_S3_z" ,	//z
			//传感器4偏移
			"POfs_S4_x" ,	//x
			"POfs_S4_y" ,	//y
			"POfs_S4_z" ,	//z
			//传感器5偏移
			"POfs_S5_x" ,	//x
			"POfs_S5_y" ,	//y
			"POfs_S5_z" ,	//z
			//传感器6偏移
			"POfs_S6_x" ,	//x
			"POfs_S6_y" ,	//y
			"POfs_S6_z" ,	//z
			//传感器7偏移
			"POfs_S7_x" ,	//x
			"POfs_S7_y" ,	//y
			"POfs_S7_z" ,	//z
			//传感器8偏移
			"POfs_S8_x" ,	//x
			"POfs_S8_y" ,	//y
			"POfs_S8_z" ,	//z
			//传感器9偏移
			"POfs_S9_x" ,	//x
			"POfs_S9_y" ,	//y
			"POfs_S9_z" ,	//z
			//传感器10偏移
			"POfs_S10_x" ,	//x
			"POfs_S10_y" ,	//y
			"POfs_S10_z" ,	//z
			//传感器11偏移
			"POfs_S11_x" ,	//x
			"POfs_S11_y" ,	//y
			"POfs_S11_z" ,	//z
			//传感器12偏移
			"POfs_S12_x" ,	//x
			"POfs_S12_y" ,	//y
			"POfs_S12_z" ,	//z
			//传感器13偏移
			"POfs_S13_x" ,	//x
			"POfs_S13_y" ,	//y
			"POfs_S13_z" ,	//z
			//传感器14偏移
			"POfs_S14_x" ,	//x
			"POfs_S14_y" ,	//y
			"POfs_S14_z" ,	//z
			//传感器15偏移
			"POfs_S15_x" ,	//x
			"POfs_S15_y" ,	//y
			"POfs_S15_z" ,	//z
		};
		ParamGroupRegister( "PosOffset", 1, sizeof(initial_posoffset)/8, param_types, param_names, (uint64_t*)&initial_posoffset );
}