#pragma once

#include "Basic.hpp"
#include "vector3.hpp"

//跟随偏移信息
struct FollowInf
{
	/*跟随偏移坐标系
		0-前左上 通过yaw信息计算前向方向
		1-东北天
	*/
	uint32_t ofsType[2];
	//跟随偏移
	float ofs_x[2];
	float ofs_y[2];
	float ofs_z[2];
}__PACKED;

enum followDataType
{
	//经纬度全球坐标系
	followDataType_Gsv_xyz=0,
	followDataType_Gsv_xy,
	followDataType_Gsv_z,
	
	//本地坐标系
	followDataType_sv_xyz,
	followDataType_sv_xy,
	followDataType_sv_z,
	
	//不成为返航目标
	
	//经纬度全球坐标系
	followDataType_nRTL_Gsv_xyz=50,
	followDataType_nRTL_Gsv_xy,
	followDataType_nRTL_Gsv_z,
	
	//本地坐标系
	followDataType_nRTL_sv_xyz,
	followDataType_nRTL_sv_xy,
	followDataType_nRTL_sv_z,
	
	//追踪目标(不会成为返航目标)
	//控制上仅保持距离不保持相对位置
	
	//经纬度全球坐标系
	followDataType_track_Gs_xy=100,
	followDataType_track_Gs_xyz,
	
	//本地坐标系
	followDataType_track_s_xy,
	followDataType_track_s_xyz,
};
#define isFollowDataType_GlobalXY(x) \
	(x==followDataType_Gsv_xyz || x==followDataType_Gsv_xy || \
	 x==followDataType_nRTL_Gsv_xyz || x==followDataType_nRTL_Gsv_xy || \
	 x==followDataType_track_Gs_xy || x==followDataType_track_Gs_xyz)
#define isFollowDataType_XY(x) \
	(x==followDataType_Gsv_xyz || x==followDataType_Gsv_xy || \
	 x==followDataType_sv_xyz || x==followDataType_sv_xy || \
	 x==followDataType_nRTL_Gsv_xyz || x==followDataType_nRTL_Gsv_xy || \
	 x==followDataType_nRTL_sv_xyz || x==followDataType_nRTL_sv_xy || \
	 x==followDataType_track_Gs_xy || x==followDataType_track_Gs_xyz || \
	 x==followDataType_track_s_xy || x==followDataType_track_s_xyz)
#define isFollowDataType_XYZ(x) \
	(x==followDataType_Gsv_xyz || x==followDataType_sv_xyz || \
	 x==followDataType_nRTL_Gsv_xyz || x==followDataType_nRTL_sv_xyz || \
	 x==followDataType_track_Gs_xyz || x==followDataType_track_s_xyz)
#define isFollowDataType_Z(x) \
	(x==followDataType_Gsv_z || x==followDataType_sv_z || \
	 x==followDataType_nRTL_Gsv_z || x==followDataType_nRTL_sv_z || \
	 x==followDataType_Gsv_xyz || x==followDataType_sv_xyz || \
	 x==followDataType_nRTL_Gsv_xyz || x==followDataType_nRTL_sv_xyz )
#define isFollowDataType_Point(x) (x < 100)
#define isFollowDataType_Track(x) (x >= 100)
#define isFollowDataType_NeedRTL(x) (x < 50)

struct followSensor
{
	bool available;
	bool globalXYPosAvailable;
	followDataType dataType;
	double lat, lon;
	vector3<double> pos;
	vector3<double> vel;
	double yaw;
	double delay;
	inline bool yawAvailable() const { return (yaw>=-2.0*Pi && yaw<=2.0*Pi); }
	TIME updateTime;
};

/*
	注册精准降落传感器

	返回值：
	非0:添加成功
	0:添加失败（已有传感器或内存不足）
*/
uint32_t followSensorRegister( followDataType dataType, double TIMEOUT=-1 );
/*
	取消注册精准降落传感器

	返回值：
	true:移除成功
	false:移除失败
*/
bool followSensorUnRegister( double TIMEOUT=-1 );

/*
	获取精准降落传感器

	返回值
	0：失败
	>1：成功 每次注册传感器时自增
*/
uint16_t get_followSensor( followSensor* res_sensor, double TIMEOUT=-1 );

/*
	更新精准降落传感器

	key: 注册时返回的密钥
	pos: 位置
	vel: 速度
	yaw: 偏航角 不可用给100
	available: 是否可用

	返回值：
	true:成功
	false:失败
*/
bool update_followSensor( uint32_t key, vector3<double> pos, vector3<double> vel, double yaw, bool available, double delay=0, double TIMEOUT=-1 );

/*
	失能精准降落传感器

	返回值：
	true:成功
	false:失败
*/
bool setInavailable_followSensor( uint32_t key, double TIMEOUT=-1 );