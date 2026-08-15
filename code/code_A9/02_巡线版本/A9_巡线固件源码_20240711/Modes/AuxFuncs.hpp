#pragma once

#include "Basic.hpp"
#include "Receiver.hpp"

/*** 遥控-数字云台控制 ***/
	#define DgYTRol 1        		// 横滚
	#define DgYTPit 2        		// 俯仰
	#define DgYTYaw 3        		// 偏航
	#define DgYTCenter 4 		 		// 回中
	#define DgYTZoom 5   		 		// 变倍
	#define DgYTMode 6   		 		// 模式
	#define DgYTFocus 7  		 		// 聚焦
	#define DgYTAutoFocus 8  		// 自动聚焦
	#define DgYTShoot 9      		// 拍照
	#define DgYTVideo 10      	// 录像
	#define DgYTPicInPic 11  		// 画中画
	#define DgYTOpenLaser 12 		// 开启激光
	#define DgYTOpenThermal 13  // 开启红外
	#define DgYTAutoTrack 14    // 自动跟踪
/*** 遥控-数字云台控制 ***/ 

/*AuxFuncs
	0:无
	1-16:映射遥控器对应通道（raw data）
	25-40:用遥控器对应通道进行相机快门触发（raw_data）
	49-64:用遥控器对应通道进行云台控制（raw_data）
*/
struct AuxFuncsConfig
{
	uint16_t Aux1Func[4];
	uint16_t Aux2Func[4];
	uint16_t Aux3Func[4];
	uint16_t Aux4Func[4];
	uint16_t Aux5Func[4];
	uint16_t Aux6Func[4];
	uint16_t Aux7Func[4];
	uint16_t Aux8Func[4];
/* 遥控通道1-16对应的数字云台功能 */	
	uint16_t RcYT1Func[4];	
	uint16_t RcYT2Func[4];	
	uint16_t RcYT3Func[4];	
	uint16_t RcYT4Func[4];	
	uint16_t RcYT5Func[4];	
	uint16_t RcYT6Func[4];	
	uint16_t RcYT7Func[4];	
	uint16_t RcYT8Func[4];	
	uint16_t RcYT9Func[4];  
	uint16_t RcYT10Func[4]; 
	uint16_t RcYT11Func[4]; 
	uint16_t RcYT12Func[4]; 
	uint16_t RcYT13Func[4]; 
	uint16_t RcYT14Func[4]; 
	uint16_t RcYT15Func[4]; 
	uint16_t RcYT16Func[4]; 
	
	float Aux1Param1[2];
	float Aux2Param1[2];
	float Aux3Param1[2];
	float Aux4Param1[2];
	float Aux5Param1[2];
	float Aux6Param1[2];
	float Aux7Param1[2];
	float Aux8Param1[2];
/* 遥控通道1-16对应的数字云台参数1 */
	float RcYT1Param1[2];	
	float RcYT2Param1[2];	
	float RcYT3Param1[2];
	float RcYT4Param1[2];	
	float RcYT5Param1[2];	
	float RcYT6Param1[2];	
	float RcYT7Param1[2];	
	float RcYT8Param1[2];	
	float RcYT9Param1[2];
	float RcYT10Param1[2];
	float RcYT11Param1[2];
	float RcYT12Param1[2];
	float RcYT13Param1[2];
	float RcYT14Param1[2];
	float RcYT15Param1[2];
	float RcYT16Param1[2];
	
	float Aux1Param2[2];
	float Aux2Param2[2];
	float Aux3Param2[2];
	float Aux4Param2[2];
	float Aux5Param2[2];
	float Aux6Param2[2];
	float Aux7Param2[2];
	float Aux8Param2[2];
/* 遥控通道1-16对应的数字云台参数2 */
	float RcYT1Param2[2];	
	float RcYT2Param2[2];	
	float RcYT3Param2[2];
	float RcYT4Param2[2];	
	float RcYT5Param2[2];	
	float RcYT6Param2[2];	
	float RcYT7Param2[2];	
	float RcYT8Param2[2];	
	float RcYT9Param2[2];
	float RcYT10Param2[2];
	float RcYT11Param2[2];
	float RcYT12Param2[2];
	float RcYT13Param2[2];
	float RcYT14Param2[2];
	float RcYT15Param2[2];
	float RcYT16Param2[2];
	
	uint8_t Aux_YTSurveyPic[8]; //数字云台是否需在测绘任务过程中拍照
	uint16_t Aux_CamOnPwm[4];
	uint16_t Aux_CamOffPwm[4];
	float Aux_CamShTime[2];
	uint8_t Aux_CamTrigEna[8];
	uint16_t Aux_BsYTPit0[4];
	uint16_t Aux_BsYTPit90[4];
	uint16_t Aux_StYTPit0[4];
	uint16_t Aux_StYTPit90[4];
	uint16_t Aux_StYT2Pit0[4];
	uint16_t Aux_StYT2Pit90[4];
	uint16_t Aux_StYTRolN45[4];
	uint16_t Aux_StYTRolP45[4];
	float Aux_YTPitMin[2];
	float Aux_YTPitMax[2];
	float Aux_YTRollMax[2];
	float Aux_Pump1Min[2];
	float Aux_Pump1St[2];
	float Aux_Pump1Max[2];
	float Aux_Pump1Sp[2];
};

/*数字云台通道*/
	struct YTCtrl
	{
		/*模式 
			0-角度模式
			1-角速度模式
		*/
		uint8_t mode;
		float value;
		TIME update_TIME;
	};
	bool get_DgYTRolCtrl( YTCtrl* res, double Sync_waitTime=-1 );
	bool get_DgYTPitCtrl( YTCtrl* res, double Sync_waitTime=-1 );
	bool get_DgYTYawCtrl( YTCtrl* res, double Sync_waitTime=-1 );
	bool get_DgYTZoomCtrl( YTCtrl* res, double Sync_waitTime=-1 );
	bool get_DgYTModeCtrl( YTCtrl* res, double Sync_waitTime=-1 );
	bool get_DgYTFocusCtrl( YTCtrl* res, double Sync_waitTime=-1 );
	bool get_DgYTShootCtrl( YTCtrl* res, double Sync_waitTime=-1 );
	bool get_DgYTReturnCtrl( YTCtrl* res, double Sync_waitTime=-1 );
	
	//记录Pos
	bool recordPos( bool acTrig );
	
	//数字云台功能接口
	struct DgYTFuncStruct
	{
		//是否可用
		bool available;
		//拍照: 0-不成功 1-成功 2-等待Trig脚位触发 3-等待内部触发
		//acTrig: 内部触发(加触发数量)
		uint8_t (*takePhoto)();
		//指定俯仰角
		bool (*setPitch)( double pit );
		//指定偏航角
		bool (*setYaw)( double yaw );
	};
	int16_t dgYtFunc_Register( const DgYTFuncStruct& func );
	bool dgYtFunc_UnRegister( int16_t ind );
	bool get_dgYtFunc( int16_t ind, DgYTFuncStruct* func );
/*数字云台通道*/

//初始化Aux处理
void init_process_AuxFuncs();
//进行Aux处理
void process_AuxFuncs(const Receiver* rc, double h);

//拍照
bool AuxCamTakePhoto();
//异步拍照
bool AuxCamTakePhotoAsync(double TIMEOUT=0.01);
//自动控制云台角度
bool AuxGimbalSetAngle( double angle );

//设置对应通道PWM值
bool setAuxPWM( float PWMus, uint8_t ind );
	
void init_AuxFuncs();