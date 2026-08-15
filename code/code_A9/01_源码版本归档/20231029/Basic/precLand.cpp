#include "precLand.hpp"

#include "semphr.h"
#include "MeasurementSystem.hpp"
#include "Parameters.hpp"

static SemaphoreHandle_t precLandSemphr = xSemaphoreCreateMutex();
static precLandSensor* sensor = 0;

static inline bool Lock_precLand( double TIMEOUT = -1 )
{
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake( precLandSemphr , TIMEOUT_Ticks ) )
		return true;
	return false;
}
static inline void UnLock_precLand()
{
	xSemaphoreGive(precLandSemphr);
}

/*
	注册精准降落传感器

	返回值：
	非0:添加成功
	0:添加失败（已有传感器或内存不足）
*/
uint32_t precLandSensorRegister( double TIMEOUT )
{
	if( Lock_precLand(TIMEOUT) )
	{
		if( sensor != 0 )
		{	//传感器已存在
			UnLock_precLand();
			return 0;
		}
		
		sensor = new precLandSensor;
		if(sensor)
		{
			sensor->pos.zero();
			sensor->delay = 0;
			sensor->updateTime.set_invalid();
			sensor->available = false;
			
			sensor->yaw = 0;
			sensor->yaw_available = false;
			sensor->yaw_source = -1;
			sensor->yawUpdateTime.set_invalid();
		}
		UnLock_precLand();
		return (uint32_t)sensor;
	}
	return 0;
}
/*
	取消注册精准降落传感器

	返回值：
	true:移除成功
	false:移除失败
*/
bool precLandSensorUnRegister( double TIMEOUT )
{
	if( Lock_precLand(TIMEOUT) )
	{
		if( sensor == 0 )
		{	//传感器不存在
			UnLock_precLand();
			return false;
		}
		
		delete sensor;
		sensor = 0;
		
		UnLock_precLand();
		return true;
	}
	return false;
}

/*
	获取精准降落传感器

	返回值：
	true:成功
	false:失败
*/
bool get_precLandSensor( precLandSensor* res_sensor, double TIMEOUT )
{
	if( Lock_precLand(TIMEOUT) )
	{
		if( sensor == 0 )
		{	//传感器不存在
			UnLock_precLand();
			return false;
		}
		
		if( sensor->updateTime.get_pass_time() > 2 )
			sensor->available = false;
		*res_sensor = *sensor;
		
		UnLock_precLand();
		return true;
	}
	return false;
}

/*
	更新精准降落传感器

	返回值：
	true:成功
	false:失败
*/
bool update_precLandSensorENU( uint32_t key, vector3<double> pos, bool available, bool z_available, double TIMEOUT )
{
	if( Lock_precLand(TIMEOUT) )
	{
		if( sensor == 0 )
		{	//传感器不存在
			UnLock_precLand();
			return false;
		}
		if( key != (uint32_t)sensor )
		{	//钥匙不正确
			UnLock_precLand();
			return false;
		}
		
		PrecLandInf precLandInf;
		if( ReadParamGroup( "precLand", (uint64_t*)&precLandInf, 0 ) == PR_OK )
		{
			//获取延时补偿姿态
			Quaternion quat;
			get_history_AirframeQuatY( &quat, precLandInf.delay[0] );
			sensor->delay = precLandInf.delay[0];
			
			//补偿传感器位置偏移
			vector3<double> offset_comp = quat.rotate( vector3<double>(precLandInf.pofs_x[0],precLandInf.pofs_y[0],precLandInf.pofs_z[0]) );
			pos.x -= offset_comp.x;
			pos.y -= offset_comp.y;
			pos.z -= offset_comp.z;
			
			//补偿位置偏移
			switch(precLandInf.ofsType[0])
			{
				case 0:
				{	//前左上 通过yaw信息计算前向方向
					if( sensor->yaw_available )
					{
						double sinYaw, cosYaw;
						fast_sin_cos( sensor->yaw, &sinYaw, &cosYaw );
						pos.x += BodyHeading2ENU_x( precLandInf.ofs_x[0] , precLandInf.ofs_y[0] , sinYaw , cosYaw );
						pos.y += BodyHeading2ENU_y( precLandInf.ofs_x[0] , precLandInf.ofs_y[0] , sinYaw , cosYaw );
					}
					break;
				}
				case 1:
				{	//东北天
					pos.x += precLandInf.ofs_x[0];
					pos.y += precLandInf.ofs_y[0];
					break;
				}
			}
		}
		sensor->pos = pos;
		sensor->updateTime = TIME::now();
		sensor->available = available;
		sensor->z_available = z_available;

		UnLock_precLand();
		return true;
	}
	return false;
}

/*
	更新精准降落传感器航向
	注意：此为辅助信息，为精准降落传感器以外的传感器输入
				一旦精准降落传感器本身拥有航向信息，则此接口会返回false

	返回值：
	true:成功
	false:失败
*/
bool update_precLandSensorYaw( double yaw, bool available, double TIMEOUT )
{
	if( Lock_precLand(TIMEOUT) )
	{
		if( sensor == 0 )
		{	//传感器不存在
			UnLock_precLand();
			return false;
		}
		if( sensor->yaw_source == 0 )
		{	//主传感器有偏航信息
			UnLock_precLand();
			return false;
		}
		
		sensor->yaw = yaw;
		sensor->yawUpdateTime = TIME::now();
		sensor->yaw_available = available;
		sensor->yaw_source = 1;

		UnLock_precLand();
		return true;
	}
	return false;
}

/*
	失能精准降落传感器

	返回值：
	true:成功
	false:失败
*/
bool setInavailable_precLandSensor( uint32_t key, double TIMEOUT )
{
	if( Lock_precLand(TIMEOUT) )
	{
		if( sensor == 0 )
		{	//传感器不存在
			UnLock_precLand();
			return false;
		}
		if( key != (uint32_t)sensor )
		{	//钥匙不正确
			UnLock_precLand();
			return false;
		}	
		
		sensor->updateTime = TIME::now();
		sensor->available = false;

		UnLock_precLand();
		return true;
	}
	return false;
}