#include "followTarget.hpp"

#include "semphr.h"
#include "MeasurementSystem.hpp"
#include "Parameters.hpp"

static SemaphoreHandle_t followSemphr = xSemaphoreCreateMutex();
static followSensor* sensor = 0;
static uint16_t sensorTempInd = 0;

static inline bool Lock_follow( double TIMEOUT = -1 )
{
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake( followSemphr , TIMEOUT_Ticks ) )
		return true;
	return false;
}
static inline void UnLock_follow()
{
	xSemaphoreGive(followSemphr);
}

/*
	注册精准降落传感器

	返回值：
	非0:添加成功
	0:添加失败（已有传感器或内存不足）
*/
uint32_t followSensorRegister( followDataType dataType, double TIMEOUT )
{
	if( Lock_follow(TIMEOUT) )
	{
		if( sensor != 0 )
		{	//传感器已存在
			UnLock_follow();
			return 0;
		}
		
		sensor = new followSensor;
		if(sensor)
		{
			sensor->dataType = dataType;
			sensor->pos.zero();
			sensor->vel.zero();
			sensor->updateTime.set_invalid();
			sensor->available = false;
			sensor->globalXYPosAvailable = false;
			sensor->delay = 0;
			
			if( ++sensorTempInd == 0 )
				sensorTempInd = 1;
		}
		UnLock_follow();
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
bool followSensorUnRegister( double TIMEOUT )
{
	if( Lock_follow(TIMEOUT) )
	{
		if( sensor == 0 )
		{	//传感器不存在
			UnLock_follow();
			return false;
		}
		
		delete sensor;
		sensor = 0;
		
		UnLock_follow();
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
uint16_t get_followSensor( followSensor* res_sensor, double TIMEOUT )
{
	if( Lock_follow(TIMEOUT) )
	{
		if( sensor == 0 )
		{	//传感器不存在
			UnLock_follow();
			return false;
		}
		
		if( sensor->updateTime.get_pass_time() > 1 )
		{
			sensor->vel.zero();
		}
		*res_sensor = *sensor;

		UnLock_follow();
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
bool update_followSensor( uint32_t key, vector3<double> pos, vector3<double> vel, double yaw, bool available, double delay, double TIMEOUT )
{
	if( Lock_follow(TIMEOUT) )
	{
		if( sensor == 0 )
		{	//传感器不存在
			UnLock_follow();
			return false;
		}
		if( key != (uint32_t)sensor )
		{	//钥匙不正确
			UnLock_follow();
			return false;
		}	
		
		switch(sensor->dataType)
		{
			case followDataType_Gsv_xyz:
			case followDataType_Gsv_xy:
			case followDataType_nRTL_Gsv_xyz:
			case followDataType_nRTL_Gsv_xy:
			case followDataType_track_Gs_xy:
			case followDataType_track_Gs_xyz:
			{
				sensor->lat = pos.x;
				sensor->lon = pos.y;
				sensor->pos.z = pos.z;
				
				PosSensorHealthInf2 global_inf;
				if( get_OptimalGlobal_XY( &global_inf ) )
				{	//获取指定经纬度平面坐标
					map_projection_project( &global_inf.mp, sensor->lat, sensor->lon, &sensor->pos.x, &sensor->pos.y );
					sensor->pos.x -= global_inf.HOffset.x;
					sensor->pos.y -= global_inf.HOffset.y;
					
					sensor->globalXYPosAvailable = true;
				}
				else
					sensor->globalXYPosAvailable = available = false;
				
				break;
			}
			default:
			{
				sensor->pos = pos;
				sensor->globalXYPosAvailable = false;
				break;
			}
		}
		sensor->vel = vel;
		sensor->yaw = yaw;
		if( delay >= 0 )
			sensor->delay = delay;
		
		FollowInf followInf;
		if( ReadParamGroup( "followInf", (uint64_t*)&followInf, 0 ) == PR_OK )
		{
			//补偿位置偏移
			switch(followInf.ofsType[0])
			{
				case 0:
				{	//前左上 通过yaw信息计算前向方向
					if( sensor->yaw>-2*PI && sensor->yaw<2*PI )
					{
						double sinYaw, cosYaw;
						fast_sin_cos( sensor->yaw, &sinYaw, &cosYaw );
						sensor->pos.x += BodyHeading2ENU_x( followInf.ofs_x[0] , followInf.ofs_y[0] , sinYaw , cosYaw );
						sensor->pos.y += BodyHeading2ENU_y( followInf.ofs_x[0] , followInf.ofs_y[0] , sinYaw , cosYaw );
					}
					break;
				}
				case 1:
				{	//东北天
					sensor->pos.x += followInf.ofs_x[0];
					sensor->pos.y += followInf.ofs_y[0];
					break;
				}
			}
		}
		
		sensor->updateTime = TIME::now();
		sensor->available = available;

		UnLock_follow();
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
bool setInavailable_followSensor( uint32_t key, double TIMEOUT )
{
	if( Lock_follow(TIMEOUT) )
	{
		if( sensor == 0 )
		{	//传感器不存在
			UnLock_follow();
			return false;
		}
		if( key != (uint32_t)sensor )
		{	//钥匙不正确
			UnLock_follow();
			return false;
		}	
		
		sensor->updateTime = TIME::now();
		sensor->available = false;

		UnLock_follow();
		return true;
	}
	return false;
}