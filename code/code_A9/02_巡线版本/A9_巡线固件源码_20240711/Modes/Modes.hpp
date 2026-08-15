#pragma once

#include "Basic.hpp"
#include "mavlink.h"
#include "Receiver.hpp"

#define CMD_TYPE_MAVLINK (1<<4)
#define CMD_TYPE_MASK 0xf0
#define CMD_TYPE_PORT_MASK 0x0f
struct ModeMsg
{
	//高4位：1-mavlink消息
	//低4位：接收port
	uint8_t cmd_type;
	uint8_t sd_sysid;
	uint8_t sd_compid;
	uint8_t frame;
	uint32_t cmd;
	double params[16];
};
bool ModeReceiveMsg( ModeMsg* msg, double TIMEOUT );
bool SendMsgToMode( ModeMsg msg, double TIMEOUT );

//模式功能选项
struct ModeFuncCfg
{
	//PA-按钮1解锁前功能（模式序号）
	uint8_t Bt1PAFunc1[8];
	uint8_t Bt1PAFunc2[8];
	uint8_t Bt1PAFunc3[8];
	uint8_t Bt1PAFunc4[8];
	uint8_t Bt1PAFunc5[8];
	uint8_t Bt1PAFunc6[8];
	
	//A-按钮1解锁后功能（1-定高模式 2-位置模式 3-运动模式 22-任务模式 23-返航模式）
	uint8_t Bt1AFunc1[8];
	uint8_t Bt1AFunc2[8];
	uint8_t Bt1AFunc3[8];
	uint8_t Bt1AFunc4[8];
	uint8_t Bt1AFunc5[8];
	uint8_t Bt1AFunc6[8];
	
	//任务执行按钮
	//2-4：对应按钮按下(100%)执行任务
	//12-14：对应按钮变化执行任务
	uint8_t MissionBt[8];
	
	//返航按钮
	//2-4：对应按钮按下(100%)返航
	//12-14：对应按钮变化返航
	uint8_t RTLBt[8];
	
	//安全按钮
	//0：无
	//2-4：对应按钮按下(100%)强制锁定电机
	//10：油门最下偏航最做强制锁定电机
	uint8_t SafeBt[8];
	
	//中位死区
	float NeutralZone[2];
	//位置速度响应曲线系数
	float PosVelAlpha[2];
	//姿态响应曲线系数
	float AttAlpha[2];
	
	//配置选项开关
	//0-解锁后复位到0号航点
	//1-航线过程失控返航
	//2-自动模式禁止摇杆操作
	//3-摇杆操作退出自动模式内3秒自动恢复自动模式(防止误触)
	
	//8-航向异常禁止解锁
	//9-无定位禁止解锁
	//10-电压不正常允许解锁
	//11-位置解算异常允许解锁
	//12-SD卡容量不足不允许解锁
	//13-无避障传感器允许在避障模式解锁
	
	//24-无fix状态不允许执行航线
	uint32_t configs[2];
	
}__PACKED;
#define is_AFunc_auto(x) (x>=20)
#define is_AFunc_NoPos(x) (x<2)

#define MCfg_RstWp0_Bit (1<<0)
#define MCfg_WpNRcRTL_Bit (1<<1)
#define MCfg_NoRcCtrlInAuto_Bit (1<<2)
#define MCfg_RcvAuto_Bit (1<<3)
#define MCfg_PosNRcHold_Bit (1<<6)

#define MCfg_NoUnlockHeadingAbnormal_Bit (1<<8)
#define MCfg_NoUnlockNoPositioning_Bit (1<<9)
#define MCfg_CanUnlockVoltageAbnormal_Bit (1<<10)
#define MCfg_CanUnlockPositioningAbnormal_Bit (1<<11)
#define MCfg_NoUnlockSdInsufficient_Bit (1<<12)
#define MCfg_CanUnlockWithouAVSYS_Bit (1<<13)

#define MCfg_NoAutoWithouFixed_Bit (1<<24)

enum AFunc
{
	AFunc_Stabilize = 0,
	AFunc_AltHold = 1,
	AFunc_PosHold = 2,
	AFunc_PosHoldAv = 11,
	AFunc_PosHoldNH = 12,	//无头模式
	AFunc_PosHoldNHAv = 13,
	AFunc_Loiter = 3,
	AFunc_ManualCircle = 5,
	AFunc_Offboard = 10,
	AFunc_Follow = 16,
	
	AFunc_TakeOff = 20,
	AFunc_Mission = 22,
	AFunc_RTL = 23,
	AFunc_Land = 24,
	AFunc_RTL2Point = 36,
};
//模式状态存储
bool setCurrentFlyMode( AFunc cMode );
AFunc getCurrentFlyMode();
	
//获取遥控按钮对应的区域（0-5）
inline int8_t get_RcButtonZone( double rc, uint8_t current_zone )
{
	double st = 1.0/6*100;
	if( current_zone<=5 && rc>=current_zone*st-1 && rc<(current_zone+1)*st+1 )
		return current_zone;
	
	if( rc < 1*st )
		return 0;
	else if( rc < 2*st )
		return 1;
	else if( rc < 3*st )
		return 2;
	else if( rc < 4*st )
		return 3;
	else if( rc < 5*st )
		return 4;
	else
		return 5;
}

class Mode_Base;
void ModeRegister( Mode_Base* mode, uint8_t id );

enum ModeResult
{
	MR_OK = 0 ,
	MR_Err ,
};

class Mode_Base
{
	private:
		
	public:
		SName name;
		Mode_Base( SName name, uint8_t mode_id )
		{
			this->name = name;
			ModeRegister( this, mode_id );
		}		
		//模式主函数
		virtual ModeResult main_func( void* param1, uint32_t param2 ) = 0;
		//获取模式Mavlink模式标号
		virtual void get_MavlinkMode( ModeFuncCfg cfg, Receiver rc, 
																	uint8_t btn_zones[4], AFunc* mode )
		{
			*mode = AFunc_Stabilize;
//			*mav_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
//			*mav_main_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED;
//			*mav_sub_mode = 0;
		}
};

//当前航点记录（用于断点续飞）
struct RestoreWpInf
{
	//当前航点序号
	int32_t CurrentWp[2];
	//航线向量AB A-目标 B起点
	double line_x;
	double line_y;
	double line_z;
	//航线完成距离
	double line_fs;
	//定距拍照距离
	double CamTrigDist;
	//飞行速度
	double vel;
	//航点半径
	double wpR;
}__PACKED;
static inline void reset_RestoreWpInf( RestoreWpInf& inf )
{
	inf.CurrentWp[0] = -1;
	inf.line_x = 0;
	inf.line_y = 0;
	inf.line_z = 0;
	inf.line_fs = -1;
	inf.CamTrigDist = 0;
	inf.vel = 0;
	inf.wpR = -1;
}
//航点恢复变量(非线程安全 仅限Modes线程使用)
extern RestoreWpInf restoreWpInf;
//设置需要保存航点恢复变量到存储器
void set_RestoreWpInf_needStore();

//读取航点恢复信息至restoreWpInf
bool read_restoreWpInf( double TIMEOUT=-1 );
//存储restoreWpInf至RAM
bool storeRAM_restoreWpInf( double TIMEOUT=-1 );
//存储restoreWpInf至存储器
bool store_restoreWpInf( double TIMEOUT=-1 );

//安全模式参数
struct MSafeCfg
{
	//自动返航模式
	//bit0 - 0:失控悬停 1-失控自动返航
	//bit1 - 0:返航过程不
	uint32_t SfRtMode[2];
	
	//返航速度
	float RtSpeed[2];
	
	//经纬度定位升高返航范围
	float GbRtHRange[2];
	//本地定位（无经纬度）升高返航范围
	float LcRtHRange[2];
	
	//经纬度定位升高高度（对地）
	float GbRtHeight[2];
	//本地定位（无经纬度）升高高度（对地）
	float LcRtHeight[2];
	
	float FcAlarmVolt[2];
	//强制返航电压
	float FcRTLVolt[2];
	//强制降落电压
	float FcLandVolt[2];
}__PACKED;
#define SfRtMode_AutoRTL(x) (x&(1<<0))


void init_Modes();