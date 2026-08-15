#pragma once

#include "Basic.hpp"
#include "vector3.hpp"

//精准降落信息
struct PrecLandInf
{
	/*降落偏移坐标系
		0-前左上 通过yaw信息计算前向方向
		1-东北天
	*/
	uint32_t ofsType[2];
	//降落偏移
	float ofs_x[2];
	float ofs_y[2];
	float ofs_z[2];
	//延时
	float delay[2];
	//传感器偏移
	float pofs_x[2];
	float pofs_y[2];
	float pofs_z[2];
}__PACKED;

struct precLandSensor
{
	vector3<double> pos;
	double yaw;
	double delay;
	TIME updateTime;
	TIME yawUpdateTime;
	
	bool available;
	bool z_available;
	bool yaw_available;
	int8_t yaw_source;
};

/*
	注册精准降落传感器

	返回值：
	非0:添加成功
	0:添加失败（已有传感器或内存不足）
*/
uint32_t precLandSensorRegister( double TIMEOUT=-1 );
/*
	取消注册精准降落传感器

	返回值：
	true:移除成功
	false:移除失败
*/
bool precLandSensorUnRegister( double TIMEOUT=-1 );

/*
	获取精准降落传感器

	返回值：
	true:成功
	false:失败
*/
bool get_precLandSensor( precLandSensor* res_sensor, double TIMEOUT=-1 );

/*
	更新精准降落传感器

	返回值：
	true:成功
	false:失败
*/
bool update_precLandSensorENU( uint32_t key, vector3<double> pos, bool available, bool z_available, double TIMEOUT=-1 );

/*
	更新精准降落传感器航向
	注意：此为辅助信息，为精准降落传感器以外的传感器输入
				一旦精准降落传感器本身拥有航向信息，则此接口会返回false

	返回值：
	true:成功
	false:失败
*/
bool update_precLandSensorYaw( double yaw, bool available, double TIMEOUT=-1 );

/*
	失能精准降落传感器

	返回值：
	true:成功
	false:失败
*/
bool setInavailable_precLandSensor( uint32_t key, double TIMEOUT=-1 );