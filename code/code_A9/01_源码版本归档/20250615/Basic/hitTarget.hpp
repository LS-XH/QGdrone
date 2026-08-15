#pragma once

#include "Basic.hpp"
#include "vector2.hpp"
#include "Quaternion.hpp"

struct hitSensor
{
	//是否可用
	bool available;
	//当前时刻姿态四元数
	Quaternion attQuat;
	//相机中轴变换四元数(flu->相机中轴)
	Quaternion camQuat;
	//目标角度rad x:俯仰 y:偏航
	vector2<double> angle;
	//延迟时间
	double delay;
	//上次更新时间
	TIME updateTime;
};

/*
	注册打击传感器

	返回值：
	非0:添加成功
	0:添加失败（已有传感器或内存不足）
*/
uint32_t hitSensorRegister( double TIMEOUT=-1 );
/*
	取消注册打击传感器

	返回值：
	true:移除成功
	false:移除失败
*/
bool hitSensorUnRegister( double TIMEOUT=-1 );

/*
	获取打击传感器

	返回值
	0：失败
	>1：成功 每次注册传感器时自增
*/
uint16_t get_hitSensor( hitSensor* res_sensor, double TIMEOUT=-1 );

/*
	更新打击传感器

	key: 注册时返回的密钥
	angle: 脱靶角度
					x: 俯仰脱靶量rad 0°为水平(大地)
					y: 偏航脱靶量rad 0°为机头朝东
	angularRate: 速度
	available: 是否可用
	delay: 延迟时间

	返回值：
	true:成功
	false:失败
*/
bool update_hitSensor( uint32_t key, Quaternion camQuat, vector2<double> angle, bool available, double delay=0, double TIMEOUT=-1 );

/*
	失能打击传感器

	返回值：
	true:成功
	false:失败
*/
bool setInavailable_hitSensor( uint32_t key, double TIMEOUT=-1 );