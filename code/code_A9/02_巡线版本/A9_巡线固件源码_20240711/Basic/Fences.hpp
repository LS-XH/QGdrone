#pragma once

#include "Basic.hpp"

struct FenceInf
{
	uint16_t cmd;
	uint8_t frame;
	uint8_t autocontinue;
	uint8_t rsv[4];
	double params[7];
}__attribute__((__packed__));

/*
	设置当前任务
	wpInd：当前任务序号
*/
bool setCurrentFence( int16_t fenceInd );
/*
	读取当前任务序号
*/
int16_t getCurrentFenceInd();

/*
	获取航点任务个数
*/
uint16_t getFencesCount();
/*
	获取正在上传航点个数
*/
uint16_t getUploadingFencesCount();

/*
	清除所有航点任务
	TIMEOUT: 线程同步超时时间

	返回值：
	true:添加成功
	false:添加失败（航点过多或航点信息错误或参数表错误）
*/
bool clearFences( double TIMEOUT = -1 );

/*
	添加航点任务
	wp_inf：航点信息
	st：是否写入存储器（只有当前实际航点数量为0才可以缓存不写入存储器）
	TIMEOUT: 线程同步超时时间

	返回值：
	true:添加成功
	false:添加失败（航点过多或航点信息错误或参数表错误）
*/
bool addFence( FenceInf wp_inf, bool st = true, double TIMEOUT = -1 );

/*
	保存航点任务
	TIMEOUT: 线程同步超时时间

	返回值：
	true:保存成功
	false:保存失败（超时）
*/
bool saveFences( double TIMEOUT = -1 );

/*
	读取航点任务
	wp_ind：航点序号
	wp_inf：获取的航点信息
	TIMEOUT: 线程同步超时时间

	返回值：
	true:获取成功
	false:获取失败（无航点或参数表错误）
*/
bool ReadFence( uint16_t wp_ind, FenceInf* wp_inf, double TIMEOUT = -1 );

void init_Fences();