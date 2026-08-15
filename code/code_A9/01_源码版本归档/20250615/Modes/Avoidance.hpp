#pragma once

#include "Basic.hpp"
#include "vector3.hpp"
#include "vector2.hpp"

struct AvoidanceCfg
{
	//机架轴距(cm)
	float wheelbase[2];
	//避障策略
	//bit0:避障使能
	//bit1:使能向下避障(临近地面减速)
	//bit8:允许向上主动避障
	//bit9:允许向左主动避障
	//bit10:允许向右主动避障
	//bit11:允许向下主动避障
	//bit16:优先向上主动避障
	//bit17:优先向左主动避障
	//bit18:优先向右主动避障
	//bit19:优先向下主动避障
	uint32_t AvoidanceMode[2];
	//避障距离（离障碍物距离cm）
	float AvoidanceDist[2];
	
	//围栏开关
	uint32_t fenceEnable[2];
	//围栏类型
	uint32_t fenceType[2];
	//围栏动作
	uint32_t fenceAction[2];
	//圆形围栏半径
	float fenceRadius[2];
	//最大最小高度
	float fenceMinAlt[2];
	float fenceMaxAlt[2];
	
}__PACKED;

#define AvModeFlag_Ena (1<<0)
#define AvModeFlag_DownsideAv_Ena (1<<1)
#define AvModeFlag_Fence_Ena (1<<3)
#define AvModeFlag_UpsideAcav_Ena (1<<8)
#define AvModeFlag_LeftsideAcav_Ena (1<<9)
#define AvModeFlag_RightsideAcav_Ena (1<<10)
#define AvModeFlag_DownsideAcav_Ena (1<<11)

#define AvMode_Enabled(x) (x & AvModeFlag_Ena)

//简单围栏使能(半径和高度)
#define FenceEnable_SplFenceFlag (1<<0)
//复杂围栏使能(地面站画的复杂围栏)
#define FenceEnable_CpxFenceFlag (1<<1)

/*避障*/
	enum AvTargetType
	{
		//三维点目标
		AvTargetType_3dPoint = 0,
		AvTargetType_2dPoint,
		
		//水平直线(竖直无限长直线墙面目标)(xy+z斜率)
		AvTargetType_XYStraightLine,
		//水平线段(竖直直线段墙面目标)(pos:xy+pos2:xy)
		AvTargetType_XYLineSegment,
		//三维线段(pos:xyz+pos2:xyz)
		AvTargetType_3dLineSegment,
		
		//Z平面(无限大Z平面目标)(z为墙面z坐标)
		AvTargetType_ZSurface,
	};
	struct AvoidanceTarget
	{
		//是否注册
		bool registered;
		//是否可用
		bool available;
		//目标类型
		AvTargetType type;

		//目标位置
		vector3<double> pos;
		vector3<double> pos2;
		//更新时间
		TIME last_update_TIME;
	};
	#define max_AvTargets 8

	//获取避障目标个数
	uint8_t getAvTargetsCount();
	
	//注册避障目标
	//返回：-1无法注册 >=0避障目标id
	int8_t registere_AvTarget( double TIMEOUT=-1 );
	//注销避障目标
	int8_t unregistere_AvTarget( uint8_t id, double TIMEOUT=-1 );
	
	//获取避障目标信息
	bool get_AvTarget( uint8_t id, AvoidanceTarget* resTarget, double TIMEOUT=-1 );
	
	//将避障目标设为不可用
	bool set_AvTargetInavailable( uint8_t id, double TIMEOUT=-1 );
	
	/*三维点目标*/
		//设置避障目标（相对当前ENU位置）
		bool set_AvTarget3dPoint_RelativeEnu( uint8_t id, vector3<double> dis, double TIMEOUT=-1 );
		//设置避障目标（相对当前FLU位置）
		bool set_AvTargetPoint3dPoint_RelativeFlu( uint8_t id, vector3<double> dis, double TIMEOUT=-1 );
	/*三维点目标*/
	
	/*XY墙面*/
		bool set_AvTargetXYStraightLine_RelativeEnu( uint8_t id, vector2<double> dis, double angle, double TIMEOUT=-1 );
		bool set_AvTargetXYStraightLine_RelativeFlu( uint8_t id, vector2<double> dis, double angle=0, double TIMEOUT=-1 );
	/*XY墙面*/
	
	/*获取期望速度方向上最近障碍物信息（速度ENU系）
		返回：true前方有障碍物 false前方无障碍物
		resAvDistance：返回的前方障碍物距离(cm) 
			数组长度必须大于offsetsCount+1
			0：无偏移避障计算结果
			1-offsetsCount：当前位置偏移为tagetOffsets[i]的避障计算结果
		targetVel：速度方向(前进方向)
		inRange：避障本体直径(飞机大小cm)
		tagetOffsets：当前位置偏移数组 偏移必须垂直于速度方向 水平于速度方向的分量将被去除
		offsetsCount：位置偏移数组长度
	*/
	bool get_AvLineDistanceEnu(double* resAvDistance, vector3<double> targetVel, double inRange, const vector3<double>* posOffsets=0, uint8_t offsetsCount=0, double TIMEOUT=-1);
	//获取期望速度方向上最近障碍物信息（速度FLU系）
	bool get_AvLineDistanceFlu(double* resAvDistance, vector3<double> targetVel, double inRange, const vector3<double>* posOffsets=0, uint8_t offsetsCount=0, double TIMEOUT=-1);
/*避障*/

/*围栏*/
	enum FRS
	{
		//不在可飞行区域内
		FRS_NotInFlyZone = -1,
		//在禁飞区内
		FRS_NotOuNFlyZone = -2,
		//无围栏
		FRS_NoFence = 0,
		//在可飞行区域内
		FRS_InFlyZone = 1,
	};
	/*计算飞机当前是否在围栏内
		返回值:
			true:在围栏范围内，distance正数为距离围栏边界距离，负数为无围栏信息
			false:不在围栏范围内，distance负数时表示速度方向不指向当前围栏内部，正数时表示指向围栏范围内部，与内部边界的距离
		pos:飞机当前位置(cm)
		targetVel：速度方向
		distance：返回的距离边界距离(cm)
		fenceRs：返回的边界结果
	*/
	bool is_insideFence(const vector3<double>& pos, const vector3<double>& targetVel=vector3<double>(0,0,0), double* distance=0, FRS* fenceRs=0 );
	
	/*计算飞机当前是否在圆形围栏内
		返回值:
			true:在围栏范围内，distance正数为距离围栏边界距离，负数为无围栏信息
			false:不在围栏范围内，distance负数时表示速度方向不指向当前围栏内部，正数时表示指向围栏范围内部，与内部边界的距离
		pos:飞机当前位置(cm)
		targetVel：速度方向
		circleO_x:圆心x   circleO_y: 圆心y   circleO_r: 圆半径
		distance：返回的距离边界距离(cm)
	*/
	bool is_insideCircleFence(const vector3<double>& pos, const vector3<double>& targetVel,
														double circleO_x, double circleO_y, double circle_r,
														double* distance );
/*围栏*/