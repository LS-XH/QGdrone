#include "Basic.hpp"
#include "Avoidance.hpp"
#include "ctrl_Main.hpp"
#include "AC_Math.hpp"
#include "vector3.hpp"
#include "MeasurementSystem.hpp"
#include <list>
#include "mavlink.h"
#include "Fences.hpp"
#include "StorageSystem.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

/*围栏*/
	#define isInclusionFence(x) (x==MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION || x==MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION)
	#define isExclusionFence(x) (x==MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION || x==MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION)
	enum FENCE_RS
	{
		//错误
		FENCE_RS_NOFENCE = -1,
		FENCE_RS_FENCEERR = -2,
		FENCE_RS_NOPOSITIONING = -3,
		FENCE_RS_TYPEERR = -4,
		//失败结果
		FENCE_RS_NotInFence = 0,	//不在围栏内且速度方向不指向围栏内部
		FENCE_RS_NotExFence = 1,	//不在围栏限制区域外且速度方向不指向区域外
		FENCE_RS_NotInFence_VelInFence = 2,	//不在围栏内，但速度方向指向围栏内部(distance=速度方向距离围栏边界距离)
		FENCE_RS_NotExFence_VelExFence = 3,	//不在围栏限制区域外，但速度方向指向区域外(distance=速度方向距离围栏边界距离)
		//成功结果
		FENCE_RS_InFence = 10,	//在围栏内(distance=速度方向距离围栏边界距离)
		FENCE_RS_ExFence = 11,	//在围栏限制范围外(distance=速度方向距离围栏边界距离 负数表示无)
	};
	enum FENCE_TYPE
	{
		FENCE_TYPE_ALL = 3,
		FENCE_TYPE_INCLUSION = 1,
		FENCE_TYPE_EXCLUSION = 2,
	};
	//计算单一围栏
	static inline FENCE_RS check_SingleFence(int16_t& fenceInd, FENCE_TYPE fenceType, const vector3<double>& pos, const vector3<double>& targetVel, const double& sinVel, const double& cosVel, double* distance)
	{
		if( fenceInd < 0 )
			return FENCE_RS_NOFENCE;
		
		FenceInf fenceInf;
		if( ReadFence(fenceInd, &fenceInf) )
		{	//获取到围栏信息
			if( (fenceType & FENCE_TYPE_INCLUSION) && isInclusionFence(fenceInf.cmd) )
				goto CHKFENCE;
			if( (fenceType & FENCE_TYPE_EXCLUSION) && isExclusionFence(fenceInf.cmd) )
				goto CHKFENCE;
			switch(fenceInf.cmd)
			{
				default:
				case MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
				case MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
				{	//圆形
					++fenceInd;
					break;
				}
				
				case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
				case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
				{	//多边形
					uint16_t vertexs = fenceInf.params[0];
					fenceInd += vertexs;
					break;
				}
			}
			return FENCE_RS_TYPEERR;
CHKFENCE:
			switch(fenceInf.cmd)
			{
				case MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
				{	//在圆内
					++fenceInd;
					//获取最优全球定位传感器信息
					PosSensorHealthInf2 global_inf;
					if( get_OptimalGlobal_XY( &global_inf ) == false )
					{	//无经纬度传感器
						return FENCE_RS_NOPOSITIONING;
					}
					//获取指定经纬度平面坐标
					double x, y;
					double offsetX = global_inf.HOffset.x + pos.x;
					double offsetY = global_inf.HOffset.y + pos.y;
					map_projection_project( &global_inf.mp, fenceInf.params[4], fenceInf.params[5], &x, &y );
					x -= offsetX;
					y -= offsetY;
					
					//投影圆心到速度方向
					vector2<double> CPos;
					CPos.x = ENU2BodyHeading_x( x, y, sinVel, cosVel );
					CPos.y = ENU2BodyHeading_y( x, y, sinVel, cosVel );
					double r = fenceInf.params[0]*100;
					if( fabs(CPos.y)>=r || fabs(CPos.x)>=r )
					{	//圆完全在原点外
						return FENCE_RS_NotInFence;
					}
					else
					{
						double xC = safe_sqrt( sq(r) - sq(CPos.y) );
						double A = CPos.x - xC;
						double B = CPos.x + xC;
						if( A<0 && B>=0 )
						{	//在圆内
							if(distance)
								*distance = B;
							return FENCE_RS_InFence;
						}
						else if( A<0 && B<=0 )
						{	//两个点都在x轴负方向
							return FENCE_RS_NotInFence;
						}
						else
						{	//两个点都在x+
							if(distance)
								*distance = A;
							return FENCE_RS_NotInFence_VelInFence;
						}
					}
					break;
				}
				case MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
				{	//在圆外
					++fenceInd;
					//获取最优全球定位传感器信息
					PosSensorHealthInf2 global_inf;
					if( get_OptimalGlobal_XY( &global_inf ) == false )
					{	//无经纬度传感器
						return FENCE_RS_NOPOSITIONING;
					}
					//获取指定经纬度平面坐标
					double x, y;
					double offsetX = global_inf.HOffset.x + pos.x;
					double offsetY = global_inf.HOffset.y + pos.y;
					map_projection_project( &global_inf.mp, fenceInf.params[4], fenceInf.params[5], &x, &y );
					x -= offsetX;
					y -= offsetY;
					
					//投影圆心到速度方向
					vector2<double> CPos;
					CPos.x = ENU2BodyHeading_x( x, y, sinVel, cosVel );
					CPos.y = ENU2BodyHeading_y( x, y, sinVel, cosVel );
					double r = fenceInf.params[0]*100;
					if( fabs(CPos.y)>=r || CPos.x<-r )
					{	//圆完全在原点外
						if(distance)
							*distance = -1;
						return FENCE_RS_ExFence;
					}
					else
					{
						double xC = safe_sqrt( sq(r) - sq(CPos.y) );
						double A = CPos.x - xC;
						double B = CPos.x + xC;
						if( A<0 && B>=0 )
						{	//在圆内
							if(distance)
								*distance = B;
							if( B > -A )
								return FENCE_RS_NotExFence;
							else
								return FENCE_RS_NotExFence_VelExFence;
						}
						else if( A<0 && B<=0 )
						{	//两个点都在x轴负方向
							if(distance)
								*distance = -1;
							return FENCE_RS_ExFence;
						}
						else
						{	//两个点都在x+
							if(distance)
								*distance = A;
							return FENCE_RS_ExFence;
						}
					}
					break;
				}
				
				case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
				{	//在多边形内
					uint16_t vertexs = fenceInf.params[0];
					uint16_t cmd = fenceInf.cmd;
					//获取最优全球定位传感器信息
					PosSensorHealthInf2 global_inf;
					if( get_OptimalGlobal_XY( &global_inf ) == false )
					{	//无经纬度传感器
						fenceInd += vertexs;
						return FENCE_RS_NOPOSITIONING;
					}
					++fenceInd;
					//获取指定经纬度平面坐标
					double x, y;
					double offsetX = global_inf.HOffset.x + pos.x;
					double offsetY = global_inf.HOffset.y + pos.y;
					map_projection_project( &global_inf.mp, fenceInf.params[4], fenceInf.params[5], &x, &y );
					x -= offsetX;
					y -= offsetY;
					vector2<double> vertex0;
					vector2<double> last_vertex;
					last_vertex.x = ENU2BodyHeading_x( x, y, sinVel, cosVel );
					last_vertex.y = ENU2BodyHeading_y( x, y, sinVel, cosVel );
					vertex0 = last_vertex;
					//统计交点个数判断是否在范围内
					uint16_t crCount = 0;
					double min_x = -1;
					for( uint8_t i=1; i<vertexs; ++i )
					{
						if( ReadFence(fenceInd, &fenceInf) )
						{	//获取到围栏信息
							if( fenceInf.cmd != cmd )
								return FENCE_RS_FENCEERR;
							if( fenceInf.params[0] != vertexs )
								return FENCE_RS_FENCEERR;
							++fenceInd;
							//计算旋转投影坐标点
							map_projection_project( &global_inf.mp, fenceInf.params[4], fenceInf.params[5], &x, &y );
							x -= offsetX;
							y -= offsetY;
							vector2<double> vertex;
							vertex.x = ENU2BodyHeading_x( x, y, sinVel, cosVel );
							vertex.y = ENU2BodyHeading_y( x, y, sinVel, cosVel );
checkFENCE_POLYGON_VERTEX_INCLUSION:
							//判断是否为交点
							if( last_vertex.y*vertex.y <= 0 )
							{	//可能存在交点
								double crX = vertex.y - last_vertex.y;
								if( is_zero(crX) )
								{	//水平线
									if( vertex.x * last_vertex.x < 0 )
									{	//水平线穿过原点
										//压围栏判断不在围栏内
										fenceInd += vertexs - i - 1;
										return FENCE_RS_NotInFence;
									}
									else
									{	//水平线不经过原点
										//如果两个点都在x轴右侧则选取坐标最小的点计算距离
										if( vertex.x > 0 )
										{
											++crCount;
											double dis = vertex.x < last_vertex.x ? vertex.x : last_vertex.x;
											//更新最小距离
											if( min_x<0 || dis<min_x )
												min_x = dis;
										}
									}
								}
								else
								{	//非水平线
									//求与x轴交点坐标
									crX = ( last_vertex.x*vertex.y - vertex.x*last_vertex.y ) / crX;
									if( crX > 0 )
									{	//交点在x轴右侧
										++crCount;
										//更新最小距离
										if( min_x<0 || crX<min_x )
											min_x = crX;
									}
								}
							}
							//保存旧坐标点
							last_vertex = vertex;
							if( i==vertexs-1 && vertexs>=3 )
							{	//检查n-0边
								++i;
								vertex = vertex0;
								goto checkFENCE_POLYGON_VERTEX_INCLUSION;
							}
						}
						else
							return FENCE_RS_NOFENCE;
					}
					//遍历完成
					//奇数个交点证明在围栏内
					if( crCount & 1 )
					{	//在围栏内
						if( distance )
							*distance = min_x;
						return FENCE_RS_InFence;
					}
					else
					{	//在围栏外
						if( crCount == 0 )
							//无交点证明此方向无围栏
							return FENCE_RS_NotInFence;
						else
						{	//速度方向有围栏
							if( distance )
								*distance = min_x;
							return FENCE_RS_NotInFence_VelInFence;
						}
					}
					break;
				}
				case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
				{	//在多边形外
					uint16_t vertexs = fenceInf.params[0];
					uint16_t cmd = fenceInf.cmd;
					//获取最优全球定位传感器信息
					PosSensorHealthInf2 global_inf;
					if( get_OptimalGlobal_XY( &global_inf ) == false )
					{	//无经纬度传感器
						fenceInd += vertexs;
						return FENCE_RS_NOPOSITIONING;
					}
					++fenceInd;
					//获取指定经纬度平面坐标
					double x, y;
					double offsetX = global_inf.HOffset.x + pos.x;
					double offsetY = global_inf.HOffset.y + pos.y;
					map_projection_project( &global_inf.mp, fenceInf.params[4], fenceInf.params[5], &x, &y );
					x -= offsetX;
					y -= offsetY;
					vector2<double> vertex0;
					vector2<double> last_vertex;
					last_vertex.x = ENU2BodyHeading_x( x, y, sinVel, cosVel );
					last_vertex.y = ENU2BodyHeading_y( x, y, sinVel, cosVel );
					vertex0 = last_vertex;
					//统计交点个数判断是否在范围内
					uint16_t crCount = 0;
					double min_x = -1;
					double min_Nx = -1;
					for( uint8_t i=1; i<vertexs; ++i )
					{
						if( ReadFence(fenceInd, &fenceInf) )
						{	//获取到围栏信息
							if( fenceInf.cmd != cmd )
								return FENCE_RS_FENCEERR;
							if( fenceInf.params[0] != vertexs )
								return FENCE_RS_FENCEERR;
							++fenceInd;
							//计算旋转投影坐标点
							map_projection_project( &global_inf.mp, fenceInf.params[4], fenceInf.params[5], &x, &y );
							x -= offsetX;
							y -= offsetY;
							vector2<double> vertex;
							vertex.x = ENU2BodyHeading_x( x, y, sinVel, cosVel );
							vertex.y = ENU2BodyHeading_y( x, y, sinVel, cosVel );
checkFENCE_POLYGON_VERTEX_EXCLUSION:
							//判断是否为交点
							if( last_vertex.y*vertex.y <= 0 )
							{	//可能存在交点
								double crX = vertex.y - last_vertex.y;
								if( is_zero(crX) )
								{	//水平线
									if( vertex.x * last_vertex.x < 0 )
									{	//水平线穿过原点
										//压围栏判断不在围栏内
										fenceInd += vertexs - i - 1;
										return FENCE_RS_NotExFence;
									}
									else
									{	//水平线不经过原点
										//如果两个点都在x轴右侧则选取坐标最小的点计算距离
										if( vertex.x > 0 )
										{	//X+交点
											++crCount;
											double dis = vertex.x < last_vertex.x ? vertex.x : last_vertex.x;
											//更新最小距离
											if( min_x<0 || dis<min_x )
												min_x = dis;
										}
										else
										{	//X-交点
											double dis = vertex.x > last_vertex.x ? -vertex.x : -last_vertex.x;
											//更新最小距离
											if( min_Nx<0 || dis<min_Nx )
												min_Nx = dis;
										}
									}
								}
								else
								{	//非水平线
									//求与x轴交点坐标
									crX = ( last_vertex.x*vertex.y - vertex.x*last_vertex.y ) / crX;
									if( crX > 0 )
									{	//交点在x轴右侧
										++crCount;
										//更新最小距离
										if( min_x<0 || crX<min_x )
											min_x = crX;
									}
									else
									{	//X-交点
										double dis = -crX;
										if( min_Nx<0 || dis<min_Nx )
											min_Nx = dis;
									}
								}
							}
							//保存旧坐标点
							last_vertex = vertex;
							if( i==vertexs-1 && vertexs>=3 )
							{	//检查n-0边
								++i;
								vertex = vertex0;
								goto checkFENCE_POLYGON_VERTEX_EXCLUSION;
							}
						}
						else
							return FENCE_RS_NOFENCE;
					}
					//遍历完成
					//奇数个交点证明在围栏内
					if( crCount & 1 )
					{	//在禁飞区内
						if( min_x < min_Nx )
						{	//当前方向可飞出禁飞区
							if( distance )
								*distance = min_x;
							return FENCE_RS_NotExFence_VelExFence;
						}
						else
							return FENCE_RS_NotExFence;
					}
					else
					{	//在围栏外
						if( crCount == 0 )
						{	//无交点证明此方向无围栏
							if( distance )
								*distance = -1;
							return FENCE_RS_ExFence;
						}
						else
						{	//速度方向有围栏
							if( distance )
								*distance = min_x;
							return FENCE_RS_ExFence;
						}
					}
					break;
				}
				
				default:
				{	//未知指令
					++fenceInd;
					return FENCE_RS_FENCEERR;
					break;
				}
			}
		}
		else
		{	//无围栏信息
			return FENCE_RS_NOFENCE;
		}
	}
		
	//计算飞机当前是否在围栏内
	//true:在围栏范围内，distance正数为距离围栏边界距离，负数为无围栏信息
	//false:不在围栏范围内，distance负数时表示速度方向不指向当前围栏内部，正数时表示指向当前围栏内部与当前围栏的距离
	//不在围栏范围内时，fenceRs为-1表示不在可飞行区域内，-2表示当前在禁飞区内
	bool is_insideFence(const vector3<double>& pos, const vector3<double>& targetVel, double* distance, FRS* fenceRs )
	{
		//求预备信息
		double sinVel, cosVel;
		double velLen = safe_sqrt( targetVel.get_square() );
		if( is_zero(velLen) )
		{	//速度长度为0
			sinVel = 0;
			cosVel = 1;
		}
		else
		{
			double invVelLen = 1.0/velLen;
			sinVel = targetVel.y * invVelLen;
			cosVel = targetVel.x * invVelLen;
		}
		
		//获取当前围栏序号
		int16_t currentFence = getCurrentFenceInd();
		//当前读取的围栏序号
		int16_t fenceInd = 0;
		//inclusion围栏个数
		uint16_t inclusionFenceCount = 0;
		//当前围栏状态 -1-当前围栏不存在 0-在边界内 1-不在边界内 2-不在边界内但速度方向指向边界内
		int8_t currentFenceState = -1;
		//距离最远的inclusion围栏
		double max_InFenceDistance = -1;	int16_t max_InFenceDistanceInd = -1;
		//禁飞区状态 -1-不存在禁飞区 0-在禁飞区外 1-在禁飞区内但速度指向区域外 2-在禁飞区内
		int8_t exFenceState = -1;
		//距离最近的exclusion围栏
		double min_ExFenceDistance = -1;	double max_NotExFenceDistance = -1;
		//速度指向围栏内时，离围栏距离
		double NotInFence_VelInFence_distance = -1;
		while(1)
		{
			//遍历计算围栏坐标状态
			double dis;
			int16_t currentFenceInd = fenceInd;
			FENCE_RS rs = check_SingleFence( fenceInd, FENCE_TYPE_ALL, pos, targetVel, sinVel, cosVel, &dis );
			if( rs==FENCE_RS_InFence )
			{	//在围栏内
				//更新距离围栏边界最大距离
				++inclusionFenceCount;
				if( max_InFenceDistance<0 || dis>max_InFenceDistance )
				{
					max_InFenceDistance = dis;
					max_InFenceDistanceInd = currentFenceInd;
				}
				if( currentFence==currentFenceInd )
				{	//当前围栏在边界内
					currentFenceState = 0;
				}
			}
			else if( rs==FENCE_RS_NotInFence || rs==FENCE_RS_NotInFence_VelInFence )
			{	//不在围栏内
				++inclusionFenceCount;
				if( currentFence==currentFenceInd )
				{	//超出当前围栏范围
					if( rs==FENCE_RS_NotInFence_VelInFence && velLen>1 )
						currentFenceState = 2;
					else
						currentFenceState = 1;
					NotInFence_VelInFence_distance = dis;
				}
			}
			else if( rs==FENCE_RS_ExFence )
			{	//禁飞区外
				if( exFenceState < 0 )
					exFenceState = 0;
				if( min_ExFenceDistance<0 || dis<min_ExFenceDistance )
					min_ExFenceDistance = dis;
			}
			else if( rs==FENCE_RS_NotExFence_VelExFence )
			{	//禁飞区内但速度指向区域外
				if( exFenceState < 1 )
					exFenceState = 1;
				if( max_NotExFenceDistance<0 || dis>max_NotExFenceDistance )
					max_NotExFenceDistance = dis;
			}
			else if( rs==FENCE_RS_NotExFence )
			{	//禁飞区内
				if( exFenceState < 2 )
					exFenceState = 2;
				//直接退出
				if(distance)
					*distance = -1;
				if(fenceRs)
					*fenceRs = FRS_NotOuNFlyZone;
				return false;
			}
			else if( rs == FENCE_RS_NOFENCE )
			{	//无围栏退出
				break;
			}
		}
		
		double minDis = -1;
		if( exFenceState == 1 )
		{	//禁飞区内返回与边界距离
			if(distance)
				*distance = max_NotExFenceDistance;
			if(fenceRs)
				*fenceRs = FRS_NotOuNFlyZone;
			return false;
		}
		else if( exFenceState == 0 )
		{	//禁飞外更新边界距离
			minDis = min_ExFenceDistance;
		}
		if( inclusionFenceCount > 0 )
		{	//有inclusion围栏
			if( currentFenceState == 0 )
			{	//飞行器仍在当前围栏范围内
				if( max_InFenceDistanceInd != currentFence )
				{	//最大距离围栏不为当前围栏
					//更新当前围栏位最大距离围栏
					setCurrentFence(max_InFenceDistanceInd);
				}
				if( minDis<0 || minDis>max_InFenceDistance )
					minDis = max_InFenceDistance;
				if(distance)
					*distance = minDis;
				if(fenceRs)
					*fenceRs = FRS_InFlyZone;
				return true;
			}
			else if( currentFenceState==-1 || currentFenceState==1 )
			{	//当前围栏不存在或不在边界内
				if( max_InFenceDistanceInd >= 0 )
				{	//存在在内部的围栏
					//更新当前围栏
					setCurrentFence(max_InFenceDistanceInd);
					if( minDis<0 || minDis>max_InFenceDistance )
						minDis = max_InFenceDistance;
					if(distance)
						*distance = minDis;
					if(fenceRs)
						*fenceRs = FRS_InFlyZone;
					return true;
				}
				else
				{	//不存在在内部的围栏
					if(distance)
						*distance = -1;
					if(fenceRs)
						*fenceRs = FRS_NotInFlyZone;
					return false;
				}
			}
			else if( currentFenceState == 2 )
			{	//不在当前围栏内但速度方向指向边界内
				if( max_InFenceDistanceInd >= 0 )
				{	//存在在内部的围栏
					//更新当前围栏
					setCurrentFence(max_InFenceDistanceInd);
					if( minDis<0 || minDis>max_InFenceDistance )
						minDis = max_InFenceDistance;
					if(distance)
						*distance = minDis;
					if(fenceRs)
						*fenceRs = FRS_InFlyZone;
					return true;
				}
				else
				{	//不存在在内部的围栏
					if(distance)
						*distance = NotInFence_VelInFence_distance;
					if(fenceRs)
						*fenceRs = FRS_NotInFlyZone;
					return false;
				}
			}
		}
		else
		{	//无inclusion围栏
			if(distance)
				*distance = minDis;
			if( exFenceState < 0 )
			{
				if(fenceRs)
					*fenceRs = FRS_NoFence;
			}
			else
			{
				if(fenceRs)
					*fenceRs = FRS_InFlyZone;
			}
			return true;
		}
		if(distance)
			*distance = -1;
		return false;
	}
/*围栏*/
	
/*避障*/
	/*避障系统互斥锁*/
		static SemaphoreHandle_t AvMutex = xSemaphoreCreateRecursiveMutex();
		bool LockAv( double TIMEOUT )
		{
			TickType_t TIMTOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMTOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMTOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTakeRecursive( AvMutex, TIMTOUT_Ticks ) == pdTRUE )
				return true;
			else
				return false;
		}
		void UnlockAv()
		{
			xSemaphoreGiveRecursive(AvMutex);
		}
	/*避障系统互斥锁*/

	static AvoidanceTarget AvTargets[max_AvTargets] = {0};
	static uint8_t AvTargetsCount = 0;
	
	uint8_t getAvTargetsCount()
	{
		return AvTargetsCount;
	}
	
	int8_t registere_AvTarget( double TIMEOUT )
	{
		if( LockAv(TIMEOUT) )
		{
			int8_t id = -1;
			for( uint8_t i = 0; i < max_AvTargets; ++i )
			{	//选空位填充
				if( AvTargets[i].registered==false )
				{
					id = i;
					break;
				}
			}
			if( id < 0 )
			{	//无空位
				UnlockAv();
				return -1;
			}
			
			//插入目标
			AvTargets[id].registered = true;
			AvTargets[id].last_update_TIME = TIME::now();
			AvTargets[id].available = false;
			
			++AvTargetsCount;
			
			UnlockAv();
			return id;
		}
		return -1;
	}
	int8_t unregistere_AvTarget( uint8_t id, double TIMEOUT )
	{
		if( id >= max_AvTargets )
			return false;
		if( LockAv(TIMEOUT) )
		{
			if( AvTargets[id].registered == false )
			{	//不可用退出
				UnlockAv();
				return false;
			}
			
			//插入目标
			AvTargets[id].registered = false;
			AvTargets[id].available = false;
			
			--AvTargetsCount;
			
			UnlockAv();
			return id;
		}
		return -1;
	}
	
	bool get_AvTarget( uint8_t id, AvoidanceTarget* resTarget, double TIMEOUT )
	{
		if( id >= max_AvTargets )
			return false;
		
		if( LockAv(TIMEOUT) )
		{
			if( AvTargets[id].registered == false )
			{	//不可用退出
				UnlockAv();
				return false;
			}
			//插入目标
			*resTarget = AvTargets[id];
			
			UnlockAv();
			return true;
		}
		return false;
	}
	
	bool set_AvTargetInavailable( uint8_t id, double TIMEOUT )
	{
		if( id >= max_AvTargets )
			return false;
		
		if( LockAv(TIMEOUT) )
		{
			if( AvTargets[id].registered == false )
			{	//不可用退出
				UnlockAv();
				return false;
			}
			//插入目标
			AvTargets[id].available = false;
			AvTargets[id].last_update_TIME = TIME::now();
			
			UnlockAv();
			return true;
		}
		return false;
	}
	
	/*二维点目标*/
		bool set_AvTarget2dPoint_RelativeEnu( uint8_t id, vector2<double> dis, double TIMEOUT )
		{
			if( !isvalid(dis.x) || !isvalid(dis.y) )
				return false;
			if( id >= max_AvTargets )
				return false;
			
			//获取当前位置
			vector3<double> pos;
			get_Position_Ctrl(&pos);
			if( LockAv(TIMEOUT) )
			{
				if( AvTargets[id].registered == false )
				{	//不可用退出
					UnlockAv();
					return false;
				}

				//插入目标
				AvTargets[id].type = AvTargetType_2dPoint;
				AvTargets[id].available = true;
				AvTargets[id].pos.x = pos.x + dis.x;
				AvTargets[id].pos.y = pos.y + dis.y;
				AvTargets[id].pos.z = pos.z;
				AvTargets[id].last_update_TIME = TIME::now();
				
				UnlockAv();
				return true;
			}
			return false;
		}
		bool set_AvTargetPoint2dPoint_RelativeFlu( uint8_t id, vector2<double> dis, double TIMEOUT )
		{
			Quaternion quat;
			get_AirframeY_quat(&quat);
			
			double yaw = quat.getYaw();
			double sin_Yaw, cos_Yaw;
			fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
			vector2<double> disENU;
			disENU.x = BodyHeading2ENU_x( dis.x , dis.y , sin_Yaw , cos_Yaw );
			disENU.y = BodyHeading2ENU_y( dis.x , dis.y , sin_Yaw , cos_Yaw );
			
			return set_AvTarget2dPoint_RelativeEnu(id, disENU, TIMEOUT);
		}
	/*二维点目标*/
	
	/*三维点目标*/
		bool set_AvTarget3dPoint_RelativeEnu( uint8_t id, vector3<double> dis, double TIMEOUT )
		{
			if( !isvalid(dis.x) || !isvalid(dis.y) || !isvalid(dis.z) )
				return false;
			if( id >= max_AvTargets )
				return false;
			
			//获取当前位置
			vector3<double> pos;
			get_Position_Ctrl(&pos);
			if( LockAv(TIMEOUT) )
			{
				if( AvTargets[id].registered == false )
				{	//不可用退出
					UnlockAv();
					return false;
				}

				//插入目标
				AvTargets[id].type = AvTargetType_3dPoint;
				AvTargets[id].available = true;
				AvTargets[id].pos.x = pos.x + dis.x;
				AvTargets[id].pos.y = pos.y + dis.y;
				AvTargets[id].pos.z = pos.z + dis.z;
				AvTargets[id].last_update_TIME = TIME::now();
				
				UnlockAv();
				return true;
			}
			return false;
		}
		bool set_AvTargetPoint3dPoint_RelativeFlu( uint8_t id, vector3<double> dis, double TIMEOUT )
		{
			Quaternion quat;
			get_AirframeY_quat(&quat);
			
			double yaw = quat.getYaw();
			double sin_Yaw, cos_Yaw;
			fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
			vector3<double> disENU;
			disENU.x = BodyHeading2ENU_x( dis.x , dis.y , sin_Yaw , cos_Yaw );
			disENU.y = BodyHeading2ENU_y( dis.x , dis.y , sin_Yaw , cos_Yaw );
			disENU.z = dis.z;
			
			return set_AvTarget3dPoint_RelativeEnu(id, disENU, TIMEOUT);
		}
	/*三维点目标*/
		
	/*XY墙面*/
		bool set_AvTargetXYStraightLine_RelativeEnu( uint8_t id, vector2<double> dis, double angle, double TIMEOUT )
		{
			if( !isvalid(dis.x) || !isvalid(dis.y) || !isvalid(angle) )
				return false;
			if( id >= max_AvTargets )
				return false;
			
			//获取当前位置
			vector3<double> pos;
			get_Position_Ctrl(&pos);
			if( LockAv(TIMEOUT) )
			{
				if( AvTargets[id].registered == false )
				{	//不可用退出
					UnlockAv();
					return false;
				}

				//插入目标
				AvTargets[id].type = AvTargetType_XYStraightLine;
				AvTargets[id].available = true;
				AvTargets[id].pos.x = pos.x + dis.x;
				AvTargets[id].pos.y = pos.y + dis.y;
				AvTargets[id].pos.z = angle;
				AvTargets[id].last_update_TIME = TIME::now();
				
				UnlockAv();
				return true;
			}
			return false;
		}
		bool set_AvTargetXYStraightLine_RelativeFlu( uint8_t id, vector2<double> dis, double angle, double TIMEOUT )
		{
			Quaternion quat;
			get_AirframeY_quat(&quat);
			
			double yaw = quat.getYaw();
			double sin_Yaw, cos_Yaw;
			fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
			vector2<double> disENU;
			disENU.x = BodyHeading2ENU_x( dis.x , dis.y , sin_Yaw , cos_Yaw );
			disENU.y = BodyHeading2ENU_y( dis.x , dis.y , sin_Yaw , cos_Yaw );
			
			double disAngle = atan2(dis.y,dis.x);
			
			double lineK = yaw + disAngle + Pi*0.5 + angle;
			lineK = Mod( lineK, 2*Pi );
			if(lineK > Pi)
				lineK -= 2*Pi;
			else if(lineK < -Pi)
				lineK += 2*Pi;
			
			return set_AvTargetXYStraightLine_RelativeEnu(id, disENU, lineK, TIMEOUT);
		}
	/*XY墙面*/
	
	bool get_AvLineDistanceEnu(double* resAvDistance, vector3<double> targetVel, double inRange, const vector3<double>* posOffsets, uint8_t offsetsCount, double TIMEOUT)
	{
		double sqBA = targetVel.get_square();
		if( sqBA < 0.00001 )
			return false;
		
		//预先计算变量
		vector3<double> pos;
		get_Position_Ctrl(&pos);
		double m = 1.0 / sqBA;
		
		AvoidanceTarget targets[max_AvTargets];
		if( LockAv(TIMEOUT) )
		{
			memcpy( targets, AvTargets, sizeof(AvoidanceTarget)*max_AvTargets );
			UnlockAv();
		}
		else
			return false;
		
		//复位结果
		vector3<double>* rOffsets;
		if( posOffsets!=0 && offsetsCount>0 )
		{	//存在偏移计算目标
			rOffsets = new vector3<double>[offsetsCount];
			for( uint16_t i=0; i<=offsetsCount; ++i )
			{
				resAvDistance[i] = -1;
				//计算偏移
				if( i > 0 )
					rOffsets[i-1] = posOffsets[i-1] - targetVel*( (posOffsets[i-1]*targetVel)*m );
			}
		}
		else
			resAvDistance[0] = -1;
		
		for( uint8_t i = 0; i < max_AvTargets; ++i )
		{	//遍历所有目标
			AvoidanceTarget* target = &targets[i];
			if( target->available )
			{
				switch(target->type)
				{
					case AvTargetType_3dPoint:
					{	//三维点目标
						vector3<double> C = target->pos - pos;
						double CdA = C*targetVel;
						if( CdA > 0 ) 
						{ //目标在前方
							//计算垂足
							double k = CdA * m;
							vector3<double> foot_point = targetVel * k;
							
							//选取范围内最小距离点
							uint16_t i = 0;
							vector3<double> cOffset( 0, 0, 0 );
							do
							{
								if( i > 0 )
									cOffset = rOffsets[i-1];
								double hDis = (foot_point + cOffset - C).get_square();
								if( hDis <= sq(inRange) )
								{
									double rDis = foot_point.get_square();
									rDis = safe_sqrt(rDis);
									if( resAvDistance[i] < 0 )
										resAvDistance[i] = rDis;
									else if( rDis < resAvDistance[i] ) 
										resAvDistance[i] = rDis;
								}
								++i;
							}while( posOffsets && i<=offsetsCount );
						}
						break;
					}
					
					case AvTargetType_2dPoint:
					{	//二维点目标
						vector3<double> C = target->pos - pos;
						double CdA = C*targetVel;
						if( CdA > 0 )
						{	//目标在前方
							//以)A为x轴映射C点坐标
							double Atheta = atan2( targetVel.y, targetVel.x );
							double cosTheta, sinTheta;
							fast_sin_cos( Atheta, &sinTheta, &cosTheta );
							
							uint16_t i = 0;
							vector3<double> cOffset( 0, 0, 0 );
							do
							{
								if( i > 0 )
									cOffset = rOffsets[i-1];
								C = target->pos - pos - cOffset;
								double hDis = -C.x*sinTheta + C.y*cosTheta;
								if( fabs(hDis) < inRange )
								{
									double rDis = C.x*cosTheta + C.y*sinTheta;
									if( rDis >= 0 )
									{
										if( resAvDistance[i] < 0 )
											resAvDistance[i] = rDis;
										else if( rDis < resAvDistance[i] ) 
											resAvDistance[i] = rDis;
									}
								}
								++i;
							}while( posOffsets && i<=offsetsCount );
						}
						break;
					}
					
					case AvTargetType_XYStraightLine:
					{	//XY无限长直线墙面
						vector3<double> C = target->pos - pos;
						if( targetVel.get_square() > 0.001 )
						{	//以)A为x轴映射C点坐标
							double Atheta = atan2( targetVel.y, targetVel.x );
							double cosTheta, sinTheta;
							fast_sin_cos( Atheta, &sinTheta, &cosTheta );
							
							uint16_t i = 0;
							vector3<double> cOffset( 0, 0, 0 );
							do
							{
								if( i > 0 )
									cOffset = rOffsets[i-1];
								C = target->pos - pos - cOffset;
								vector2<double> Crotated;
								Crotated.x = C.x*cosTheta + C.y*sinTheta;
								Crotated.y = -C.x*sinTheta + C.y*cosTheta;
								
								//求投影后C直线斜率
								double Ctheta = target->pos.z - Atheta;
								//求接近点位置
								double CthetaTan = tan(Ctheta);
								if( is_zero(CthetaTan) )
								{	//平行线情况
									//只要水平距离在范围内即返回0距离
									if( fabs(Crotated.y) < inRange )
									{
										resAvDistance[i] = 0;
									}
								}
								else
								{	//非平行线
									//有两个接近点
									double invTan = 1.0 / CthetaTan;
									double x1 = Crotated.x + (inRange - Crotated.y) * invTan;
									double x2 = Crotated.x + (-inRange - Crotated.y) * invTan;
									double rDis = -1;
									if( x1*x2 < 0 )
									{	//两接近点在原点两边
										//距离应为0
										resAvDistance[i] = 0;
									}
									else if( x1 > 0 )
									{	//接近点都在原点右侧
										if( x1 < x2 )
											rDis = x1;
										else
											rDis = x2;
									}
									if( rDis >= 0 )
									{
										if( resAvDistance[i] < 0 )
											resAvDistance[i] = rDis;
										else if( rDis < resAvDistance[i] ) 
											resAvDistance[i] = rDis;
									}
								}
								++i;
							}while( posOffsets && i<=offsetsCount );
						}
						break;
					}
				}
			}
		}
		if( posOffsets!=0 && offsetsCount>0 )
			delete[] rOffsets;
		return true;
	}
	bool get_AvLineDistanceFlu(double* resAvDistance, vector3<double> targetVel, double inRange, const vector3<double>* posOffsets, uint8_t offsetsCount, double TIMEOUT)
	{
		Quaternion quat;
		get_AirframeY_quat(&quat);
		
		double yaw = quat.getYaw();
		double sin_Yaw, cos_Yaw;
		fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
		vector3<double> targetVelENU;
		targetVelENU.x = BodyHeading2ENU_x( targetVel.x , targetVel.y , sin_Yaw , cos_Yaw );
		targetVelENU.y = BodyHeading2ENU_y( targetVel.x , targetVel.y , sin_Yaw , cos_Yaw );
		targetVelENU.z = targetVel.z;
		
		vector3<double>* posOffsetsENU = 0;
		if( posOffsets!=0 && offsetsCount>0 )
		{	//存在偏移计算目标
			posOffsetsENU = new vector3<double>[offsetsCount];
			if(!posOffsetsENU)
				return false;
			for( uint16_t i=0; i<offsetsCount; ++i )
			{	//FLU偏移转换为ENU
				posOffsetsENU[i].x = BodyHeading2ENU_x( posOffsets[i].x , posOffsets[i].y , sin_Yaw , cos_Yaw );
				posOffsetsENU[i].y = BodyHeading2ENU_y( posOffsets[i].x , posOffsets[i].y , sin_Yaw , cos_Yaw );
				posOffsetsENU[i].z = posOffsets[i].z;
			}
		}
		bool res = get_AvLineDistanceEnu(resAvDistance, targetVelENU, inRange, posOffsetsENU, offsetsCount, TIMEOUT);
		if( posOffsets!=0 && offsetsCount>0 )
			delete[] posOffsetsENU;
		return res;
	}
/*避障*/