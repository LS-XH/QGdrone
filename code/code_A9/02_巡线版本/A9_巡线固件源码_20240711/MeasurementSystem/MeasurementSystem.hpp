#pragma once

#include "TimeBase.hpp"
#include "vector2.hpp"
#include "vector3.hpp"
#include "Quaternion.hpp"
#include "map_projection.hpp"

//获取三字节WGA识别码
void MS_get_WGA( uint32_t* WGA );
//获取正版验证结果
bool MS_WGA_Correct();

//获取当前使用的陀螺仪
uint8_t get_current_use_IMUGyroscope();
//获取当前使用的加速度计
uint8_t get_current_use_IMUAccelerometer();
//清空陀螺零偏数据
bool clearGyroOffsetEst( uint8_t imuInd, double TIMEOUT=-1 );

enum MS_Status
{
	MS_Initializing ,
	MS_Ready ,
	MS_Err ,
};

/*传递对准*/
	//注册传递对准 注册后解算系统需要对准信息才能完成初始化
	bool insAlignRegister();
	bool updateInsData( Quaternion quat, vector3<double> vel, double TIMEOUT=-1 );
	bool insAlignFinish();
/*传递对准*/

/*健康度信息*/
	struct PosSensorHealthInf1
	{
		//传感器序号
		uint8_t sensor_ind;
		//解算位置
		vector3<double> PositionENU;
		//传感器位置
		double sensor_pos;
		//传感器偏移（传感器健康时更新）
		//HOffset+PositionENU = 传感器估计值
		double HOffset;
		//上次健康时间
		TIME last_healthy_TIME;
		//是否可用（不可用时噪声无效）
		bool available;
		//传感器噪声上下界（传感器-解算）
		double NoiseMin, NoiseMax;
		//速度噪声
		double VNoise;
	};
	struct PosSensorHealthInf2
	{
		//传感器序号
		uint8_t sensor_ind;
		//是否全球定位传感器
		bool global_sensor;
		//是否slam传感器
		bool slam_sensor;
		//定位坐标转换信息(global或slam)
		Map_Projection mp;
		//解算位置
		vector3<double> PositionENU;
		//传感器位置
		vector2<double> sensor_pos;
		//传感器偏移（传感器健康时更新）
		//HOffset+PositionENU = 传感器估计值
		vector2<double> HOffset;
		//上次健康时间
		vector2<TIME> last_healthy_TIME;
		//是否可用（不可用时噪声无效）
		bool available;
		//传感器噪声上下界（传感器-解算）
		vector2<double> NoiseMin, NoiseMax;
		//速度噪声
		vector2<double> VNoise;
	};
	struct PosSensorHealthInf3
	{
		//传感器序号
		uint8_t sensor_ind;
		//是否全球定位传感器
		bool global_sensor;
		//是否slam传感器
		bool slam_sensor;
		//定位坐标转换信息(global或slam)
		Map_Projection mp;
		//解算位置
		vector3<double> PositionENU;
		//传感器位置
		vector3<double> sensor_pos;
		//传感器偏移（传感器健康时更新）
		//HOffset+PositionENU = 传感器估计值
		vector3<double> HOffset;
		//上次健康时间
		vector3<TIME> last_healthy_TIME;
		//是否可用（不可用时噪声无效）
		bool available;
		//传感器噪声上下界（传感器-解算）
		vector3<double> NoiseMin, NoiseMax;
		//速度噪声
		vector3<double> VNoise;		
	};

	
	
	/*XY传感器健康度*/
		//获取当前XY传感器
		int8_t get_Current_XYSensor();
		//是否存在可用的XY全球定位传感器
		bool isGlobalXYSensorExist();
		//获取XY解算健康度%
		float get_MSHealthXY();
	
		//指定序号传感器健康度
		bool get_PosSensorHealth_XY( PosSensorHealthInf2* result, uint8_t sensor_ind, double TIMEOUT = -1 );
		//当前传感器健康度
		bool get_Health_XY( PosSensorHealthInf2* result, double TIMEOUT = -1 );
		//最优测距传感器健康度
		bool get_OptimalRange_XY( PosSensorHealthInf2* result, double TIMEOUT = -1 );
		//最优全球定位传感器健康度
		bool get_OptimalGlobal_XY( PosSensorHealthInf2* result, double TIMEOUT = -1 );
	/*XY传感器健康度*/
	
	/*Z传感器健康度*/
		//获取当前Z传感器
		int8_t get_Current_ZSensor();
		//获取Z解算健康度%
		float get_MSHealthZ();
	
		//指定序号传感器健康度
		bool get_PosSensorHealth_Z( PosSensorHealthInf1* result, uint8_t sensor_ind, double TIMEOUT = -1 );
		//当前传感器健康度
		bool get_Health_Z( PosSensorHealthInf1* result, double TIMEOUT = -1 );
		//最优测距传感器健康度
		bool get_OptimalRange_Z( PosSensorHealthInf1* result, double TIMEOUT = -1 );
		//最优全球定位传感器健康度
		bool get_OptimalGlobal_Z( PosSensorHealthInf1* result, double TIMEOUT = -1 );
	/*Z传感器健康度*/
	
	/*XYZ传感器健康度*/
		//指定序号传感器健康度
		bool get_PosSensorHealth_XYZ( PosSensorHealthInf3* result, uint8_t sensor_ind, double TIMEOUT = -1 );
		//最优测距传感器健康度
		bool get_OptimalRange_XYZ( PosSensorHealthInf3* result, double TIMEOUT = -1 );
		//最优全球定位传感器健康度
		bool get_OptimalGlobal_XYZ( PosSensorHealthInf3* result, double TIMEOUT = -1 );
	/*XYZ传感器健康度*/
	
	//获取航向健康估计信息（-100 to +100 负数代表不健康）
	float get_YawHealthEst();
	//获取是否存在测向传感器
	bool isDAOSensorFixed();
	//获取初始航向传感器
	int8_t get_init_YawSensor();
/*健康度信息*/

/*姿态信息获取接口*/
	//获取解算系统状态
	MS_Status get_Attitude_MSStatus();

	//获取用于控制的滤波后的角（加）速度
	bool get_AngularRate_Ctrl( vector3<double>* result, double TIMEOUT = -1 );
	bool get_AngularAcc_Ctrl( vector3<double>* result, double TIMEOUT = -1 );
	//获取姿态四元数
	bool get_Attitude_quat( Quaternion* result, double TIMEOUT = -1 );	
	//获取机体四元数（偏航不对准）
	bool get_Airframe_quat( Quaternion* result, double TIMEOUT = -1 );
	//获取机体四元数（偏航对准）
	bool get_AirframeY_quat( Quaternion* result, double TIMEOUT = -1  );
	//获取偏航对准偏角（磁偏角）
	bool get_YawDeclination( double* result, double TIMEOUT = -1  );
	//获取历史四元数
	bool get_history_AttitudeQuat( Quaternion* result, double t, double TIMEOUT = -1 );
	//获取历史机体四元数（偏航不对准）
	bool get_history_AirframeQuat( Quaternion* result, double t, double TIMEOUT = -1 );
	//获取历史机体四元数（偏航对准）
	bool get_history_AirframeQuatY( Quaternion* result, double t, double TIMEOUT = -1 );
/*姿态信息获取接口*/

/*位置信息获取接口*/
	//获取解算系统状态
	MS_Status get_Altitude_MSStatus();
	MS_Status get_Position_MSStatus();
	
	//获取实时位置
	bool get_Position( vector3<double>* result, double TIMEOUT = -1 );
	//获取实时速度（东北天方向）
	bool get_VelocityENU( vector3<double>* result, double TIMEOUT = -1 );
	//获取实时地理系加速度（东北天）
	bool get_AccelerationENU( vector3<double>* result, double TIMEOUT = -1 );
	//获取用于控制的滤波后的地理系加速度（东北天）
	bool get_AccelerationENU_Ctrl( vector3<double>* result, double TIMEOUT = -1 );
	//获取用于控制的滤波后的地理系速度（东北天）
	bool get_VelocityENU_Ctrl( vector3<double>* result, double TIMEOUT = -1 );
	//获取用于控制的滤波后的机体系加速度（前左上）
	bool get_VelocityFLU_Ctrl( vector3<double>* result, double TIMEOUT = -1 );
	bool get_Position_Ctrl( vector3<double>* result, double TIMEOUT = -1 );
	//获取低通滤波后的未补偿（零偏灵敏度温度）的陀螺加速度数据
	bool get_AccelerationNC_filted( vector3<double>* vec, double TIMEOUT = -1 );
	bool get_AngularRateNC_filted( vector3<double>* vec, double TIMEOUT = -1 );
	
	/*快速数据*/
		float get_Position_Ctrl_x();
		float get_Position_Ctrl_y();
		float get_Position_Ctrl_z();
		float get_VelocityENU_Ctrl_x();
		float get_VelocityENU_Ctrl_y();
		float get_VelocityENU_Ctrl_z();
	/*快速数据*/
/*位置信息获取接口*/

/*电池信息接口*/
	struct BatteryCfg
	{
		//标准电压（V）
		float STVoltage[2];
		//电压测量增益（V）
		float VoltMKp[2];
		//电流测量增益（A）
		float CurrentMKp[2];
		//容量（W*h）
		float Capacity[2];
		//功率电压点0（0%电量时相对标准电压的电压差，此序列必须递增）
		float VoltP0[2];
		//功率电压点1（10%电量时相对标准电压的电压差，此序列必须递增）
		float VoltP1[2];
		//功率电压点2（20%电量时相对标准电压的电压差，此序列必须递增）
		float VoltP2[2];
		//功率电压点3（30%电量时相对标准电压的电压差，此序列必须递增）
		float VoltP3[2];
		//功率电压点4（40%电量时相对标准电压的电压差，此序列必须递增）
		float VoltP4[2];
		//功率电压点5（50%电量时相对标准电压的电压差，此序列必须递增）
		float VoltP5[2];
		//功率电压点6（60%电量时相对标准电压的电压差，此序列必须递增）
		float VoltP6[2];
		//功率电压点7（70%电量时相对标准电压的电压差，此序列必须递增）
		float VoltP7[2];
		//功率电压点8（80%电量时相对标准电压的电压差，此序列必须递增）
		float VoltP8[2];
		//功率电压点9（90%电量时相对标准电压的电压差，此序列必须递增）
		float VoltP9[2];
		//功率电压点10（100%电量时相对标准电压的电压差，此序列必须递增）
		float VoltP10[2];
	}__PACKED;

	
	struct BatteryInfo
	{
		//原始总电压V
		float totalVoltRaw;
		//补偿总电压V
		float totalVolt;
		//总电流A
		float totalCurrent;
		//总容量Wh
		float totalCapacity;
		//已使用电量Wh
		float totalPowerUsage;
		//更新时间
		TIME last_update_TIME;
		float updateT;
		
		//基准电压V
		float stVolt;
		//电量百分比%
		float totalPercent;
		//滤波后总电压V
		float totalVoltRawFilted;
		float totalVoltFilted;
		//滤波后功率W
		float totalPowerFilted;
		
		//电池温度℃   <-300表示不可用
		float temperature;
		//电池错误标志
		uint32_t errorFlags;
		//电池可用 -1:不可用 0:电池不存在 1-可用
		int8_t available;
		//循环次数
		uint16_t cycle_count;
		//电芯数量
		int8_t cells;
		//是否存在电芯电压
		bool cellVoltAvailable;
		//电芯电压V
		float* cellVolts;
	};
	#define MAX_BATTERYS 3
	
	//注册电池
	uint32_t batteryRegister( uint8_t index, double TIMEOUT=-1 );
	//取消注册电池
	bool batteryUnRegister( uint8_t index,uint32_t key, double TIMEOUT=-1 );
	//更新电池信息
	bool batteryUpdate( uint8_t index,uint32_t key,
											bool available,
											float totalVoltRaw, float totalVolt, float stVolt,
											float totalCurrent,
											const float* totalPowerUsage, float totalCapacity, float totalPercent, 
											float temperature, uint16_t cycle_count, uint32_t errorFlags,
											uint8_t cells, const float* cellVolts,
											double TIMEOUT=-1 );
	//获取电池信息
	bool getBatteryInfo( uint8_t index, BatteryInfo* info, double TIMEOUT=-1 );
	//获取当前电池信息
	bool getCurrentBatteryInfo( BatteryInfo* info, int8_t* ind=0, double TIMEOUT=-1 );
	bool getCurrentBatteryTotalVoltRaw( float* res, int8_t* ind=0, double TIMEOUT=-1 );
	bool getCurrentBatteryTotalVoltRawFilted( float* res, int8_t* ind=0, double TIMEOUT=-1 );
	bool getCurrentBatteryTotalVoltFilted( float* res, int8_t* ind=0, double TIMEOUT=-1 );
	//获取最优电池信息
	bool getOptimalBatteryInfo( BatteryInfo* info, int8_t* ind=0, double TIMEOUT=-1 );
/*电池信息接口*/
