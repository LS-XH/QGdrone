#include "hitTarget.hpp"

#include "semphr.h"
#include "MeasurementSystem.hpp"
#include "Parameters.hpp"

static SemaphoreHandle_t hitSemphr = xSemaphoreCreateMutex();
static hitSensor* sensor = 0;
static uint16_t sensorTempInd = 0;

static inline bool Lock_hit( double TIMEOUT = -1 )
{
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake( hitSemphr , TIMEOUT_Ticks ) )
		return true;
	return false;
}
static inline void UnLock_hit()
{
	xSemaphoreGive(hitSemphr);
}

/*
	注册精准降落传感器

	返回值：
	非0:添加成功
	0:添加失败（已有传感器或内存不足）
*/
uint32_t hitSensorRegister( double TIMEOUT )
{
	if( Lock_hit(TIMEOUT) )
	{
		if( sensor != 0 )
		{	//传感器已存在
			UnLock_hit();
			return 0;
		}
		
		sensor = new hitSensor;
		if(sensor)
		{
			sensor->attQuat = Quaternion(1,0,0,0);
			sensor->camQuat = Quaternion(1,0,0,0);
			sensor->angle.zero();
			sensor->updateTime.set_invalid();
			sensor->available = false;
			sensor->delay = 0;
			
			if( ++sensorTempInd == 0 )
				sensorTempInd = 1;
		}
		UnLock_hit();
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
bool hitSensorUnRegister( double TIMEOUT )
{
	if( Lock_hit(TIMEOUT) )
	{
		if( sensor == 0 )
		{	//传感器不存在
			UnLock_hit();
			return false;
		}
		
		delete sensor;
		sensor = 0;
		
		UnLock_hit();
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
uint16_t get_hitSensor( hitSensor* res_sensor, double TIMEOUT )
{
	if( Lock_hit(TIMEOUT) )
	{
		if( sensor == 0 )
		{	//传感器不存在
			UnLock_hit();
			return false;
		}
		
		if( sensor->updateTime.get_pass_time() > 1.5 )
		{
			sensor->available = false;
		}
		*res_sensor = *sensor;

		UnLock_hit();
		return sensorTempInd;
	}
	return 0;
}

/*
	更新精准降落传感器

	返回值：
	true:成功
	false:失败
*/
bool update_hitSensor( uint32_t key, Quaternion camQuat, vector2<double> angle, bool available, double delay, double TIMEOUT )
{
	if( Lock_hit(TIMEOUT) )
	{
		if( sensor == 0 )
		{	//传感器不存在
			UnLock_hit();
			return false;
		}
		if( key != (uint32_t)sensor )
		{	//钥匙不正确
			UnLock_hit();
			return false;
		}	
		
		Quaternion quat;
		get_AirframeY_quat(&quat);
		sensor->attQuat = quat;
		sensor->camQuat = camQuat;
		sensor->angle = angle;
		if( delay >= 0 )
			sensor->delay = delay;
		sensor->updateTime = TIME::now();
		sensor->available = available;

		UnLock_hit();
		return true;
	}
	return false;
}
/*
	是能精准降落传感器

	返回值：
	true:成功
	false:失败
*/
bool setInavailable_hitSensor( uint32_t key, double TIMEOUT )
{
	if( Lock_hit(TIMEOUT) )
	{
		if( sensor == 0 )
		{	//传感器不存在
			UnLock_hit();
			return false;
		}
		if( key != (uint32_t)sensor )
		{	//钥匙不正确
			UnLock_hit();
			return false;
		}	
		
		sensor->updateTime = TIME::now();
		sensor->available = false;

		UnLock_hit();
		return true;
	}
	return false;
}