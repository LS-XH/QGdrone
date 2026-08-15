
#include "Basic.hpp"
#include "ctrl_Main.hpp"
#include "ctrl_Attitude.hpp"
#include "ctrl_Position.hpp"
#include "MeasurementSystem.hpp"
#include "ControlSystem.hpp"
#include "drv_ADC.hpp"
#include "Parameters.hpp"
#include "TD4.hpp"
#include "Filters_LP.hpp"
#include "Sensors.hpp"
#include "drv_PWMOut.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

TaskHandle_t ControlTaskHandle;

/*控制系统互斥锁*/
	static SemaphoreHandle_t CtrlMutex = xSemaphoreCreateRecursiveMutex();
	bool LockCtrl( double TIMEOUT )
	{
		TickType_t TIMTOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMTOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMTOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTakeRecursive( CtrlMutex, TIMTOUT_Ticks ) == pdTRUE )
			return true;
		else
			return false;
	}
	void UnlockCtrl()
	{
		xSemaphoreGiveRecursive(CtrlMutex);
	}
/*控制系统互斥锁*/
	
/*控制系统安全时间*/
	TIME last_XYCtrlTime;
	TIME last_ZCtrlTime;
	//获取控制器上次控制时间
	//太久不进行控制将进入MSafe模式
	bool get_lastXYCtrlTime( TIME* t, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*t = last_XYCtrlTime;
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool get_lastZCtrlTime( TIME* t, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*t = last_ZCtrlTime;
			UnlockCtrl();
			return true;
		}
		return false;
	}
	
	extern bool ForceRTL;
	//将控制器上次控制时间设为不可用
	//强制进入MSafe模式
	//MSafe模式下无法关闭位置控制器
	//同时作出XYZ位置控制可退出MSafe
	//（水平控制不可用时控制角度）
	//forceRTL：是否在有遥控器器时也执行返航（遥控器需要回中）
	//TIMEOUT：超时时间
	bool enter_MSafe( bool forceRTL, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			ForceRTL = forceRTL;
			last_XYCtrlTime.set_invalid();
			last_ZCtrlTime.set_invalid();
			UnlockCtrl();
			return true;
		}
		return false;
	}
	
	//强制Safe控制
	bool ForceMSafeCtrl = false;
	
	//获取是否进入了MSafe模式
	bool is_MSafeCtrl()
	{
		return ForceMSafeCtrl;
	}
/*控制系统安全时间*/

/*电池信息*/
	static SemaphoreHandle_t BatteryMutex = xSemaphoreCreateMutex();
	Static_DTCMBuf BatteryInfo batteryInfos[MAX_BATTERYS] = {0};
	//注册电池
	uint32_t batteryRegister( uint8_t index, double TIMEOUT )
	{
		if( index >= MAX_BATTERYS )
			return 0;
		
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake(BatteryMutex,TIMEOUT_Ticks) )
		{
			BatteryInfo* bat = &batteryInfos[index];
			if( bat->available != 0 )
			{
				xSemaphoreGive(BatteryMutex);
				return 0;
			}
			//初始化电池信息
			bat->available = -1;
			bat->totalVoltRaw = -1;
			bat->totalVolt = -1;
			bat->totalCurrent = -1;
			bat->totalCapacity = -1;
			bat->totalPowerUsage = 0;
			bat->stVolt = -1;
			bat->totalPercent = -1;
			bat->totalVoltRawFilted = -1;
			bat->totalVoltFilted = -1;
			bat->totalPowerFilted = -1;
			bat->errorFlags = 0;
			bat->cycle_count = UINT16_MAX;
			bat->temperature = -500;
			bat->cells = -1;
			bat->cellVoltAvailable = false;
			bat->cellVolts = 0;
			//时间
			bat->last_update_TIME.set_invalid();
			bat->updateT = -1;
			//释放mutex
			xSemaphoreGive(BatteryMutex);
			return (uint32_t)bat;
		}
		return 0;
	}
	//取消注册电池
	bool batteryUnRegister( uint8_t index,uint32_t key, double TIMEOUT )
	{
		if( index >= MAX_BATTERYS )
			return false;
		
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake(BatteryMutex,TIMEOUT_Ticks) )
		{
			BatteryInfo* bat = &batteryInfos[index];
			if( bat->available==0 || (uint32_t)bat!=key )
			{
				xSemaphoreGive(BatteryMutex);
				return false;
			}
			bat->available = 0;
			if( bat->cellVolts )
				delete[] bat->cellVolts;
			bat->cellVolts = 0;
			//释放mutex
			xSemaphoreGive(BatteryMutex);
			return true;
		}
		return false;
	}
	//更新电池信息
	bool batteryUpdate( uint8_t index,uint32_t key,
											bool available,
											float totalVoltRaw, float totalVolt, float stVolt,
											float totalCurrent,
											const float* totalPowerUsage, float totalCapacity, float totalPercent, 
											float temperature, uint16_t cycle_count, uint32_t errorFlags,
											uint8_t cells, const float* cellVolts,
											double TIMEOUT )
	{
		if( index >= MAX_BATTERYS )
			return false;
		
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake(BatteryMutex,TIMEOUT_Ticks) )
		{
			BatteryInfo* bat = &batteryInfos[index];
			if( bat->available==0 || (uint32_t)bat!=key )
			{
				xSemaphoreGive(BatteryMutex);
				return false;
			}
			//更新电压电流原始数据
			if(available)
				bat->available = 1;
			else
				bat->available = -1;
			bat->totalVoltRaw = totalVoltRaw;
			bat->totalVolt = totalVolt;
			bat->totalCurrent = totalCurrent;
			bat->totalCapacity = totalCapacity;
			bat->stVolt = stVolt;
			bat->totalPercent = totalPercent;
			bat->temperature = temperature;
			bat->cycle_count = cycle_count;
			bat->errorFlags = errorFlags;
			//时间
			if( bat->last_update_TIME.is_valid() )
				bat->updateT = bat->last_update_TIME.get_pass_time();
			else
				bat->updateT = 0;
			bat->last_update_TIME = TIME::now();
			//更新电池计算数据
			float power = totalVoltRaw*totalCurrent;
			if( totalPowerUsage )
				bat->totalPowerUsage = *totalPowerUsage;
			else
			{	//电池本身没有已使用电量信息
				//积分计算电量
				if( bat->totalPowerUsage < 0 )
					bat->totalPowerUsage = 0;
				bat->totalPowerUsage += bat->updateT*(1.0f/3600)*power;
			}
			//电池电压滤波
			if( bat->totalVoltRawFilted < 0 )
				bat->totalVoltRawFilted = totalVoltRaw;
			else
				bat->totalVoltRawFilted += 0.1f*bat->updateT*( totalVoltRaw - bat->totalVoltRawFilted );
			if( bat->totalVoltFilted < 0 )
				bat->totalVoltFilted = totalVolt;
			else
				bat->totalVoltFilted += 0.1f*bat->updateT*( totalVolt - bat->totalVoltFilted );
			//功率滤波
			if( bat->totalPowerFilted < 0 )
				bat->totalPowerFilted = power;
			else
				bat->totalPowerFilted += 0.1f*bat->updateT*( power - bat->totalPowerFilted );
			//更新电芯电压
			if( cellVolts && cells>0 && cells<128 )
			{	//存在电芯电压
				if( cells!=bat->cells || bat->cellVoltAvailable==false )
				{
					bat->cells = cells;
					if( bat->cellVolts )
						delete[] bat->cellVolts;
					bat->cellVolts = new float[cells];
				}
				memcpy( bat->cellVolts, cellVolts, cells*sizeof(float) );
				bat->cellVoltAvailable = true;
			}
			else
			{
				bat->cells = -1;
				if( bat->cellVolts )
					delete[] bat->cellVolts;
				bat->cellVolts = 0;
				bat->cellVoltAvailable = false;
			}
			//释放mutex
			xSemaphoreGive(BatteryMutex);
			return true;
		}
		return false;
	}
	//获取电池信息
	bool getBatteryInfo( uint8_t index, BatteryInfo* info, double TIMEOUT )
	{
		if( index >= MAX_BATTERYS )
			return false;
		
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake(BatteryMutex,TIMEOUT_Ticks) )
		{
			BatteryInfo* bat = &batteryInfos[index];
			if( bat->available == 0 )
			{
				xSemaphoreGive(BatteryMutex);
				return false;
			}
			*info = *bat;
			//释放mutex
			xSemaphoreGive(BatteryMutex);
			return true;
		}
		return false;
	}
	//获取当前电池信息
	static int8_t currentBatInd = -1;
	bool getCurrentBatteryInfo( BatteryInfo* info, int8_t* ind, double TIMEOUT )
	{
		bool unLocked;
		is_Attitude_Control_Enabled(&unLocked);
		if( unLocked && (currentBatInd<0 || currentBatInd>=MAX_BATTERYS) )
			return false;
		
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake(BatteryMutex,TIMEOUT_Ticks) )
		{
			if(!unLocked)
			{	//不在飞行状态才可以切换电池
				for( uint8_t i=0; i<MAX_BATTERYS; ++i )
				{	//找序号最小的电池
					BatteryInfo* bat = &batteryInfos[i];
					if( bat->available > 0 )
					{
						currentBatInd = i;
						break;
					}
				}
			}
			BatteryInfo* bat = &batteryInfos[currentBatInd];
			if( currentBatInd<0 || currentBatInd>=MAX_BATTERYS || bat->available<=0 )
			{
				xSemaphoreGive(BatteryMutex);
				return false;
			}
			*info = *bat;
			if(ind)
				*ind = currentBatInd;
			//释放mutex
			xSemaphoreGive(BatteryMutex);
			return true;
		}
		return false;
	}
	bool getCurrentBatteryTotalVoltRaw( float* res, int8_t* ind, double TIMEOUT )
	{
		bool unLocked;
		is_Attitude_Control_Enabled(&unLocked);
		if( unLocked && (currentBatInd<0 || currentBatInd>=MAX_BATTERYS) )
			return false;
		
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake(BatteryMutex,TIMEOUT_Ticks) )
		{
			if(!unLocked)
			{	//不在飞行状态才可以切换电池
				for( uint8_t i=0; i<MAX_BATTERYS; ++i )
				{	//找序号最小的电池
					BatteryInfo* bat = &batteryInfos[i];
					if( bat->available > 0 )
					{
						currentBatInd = i;
						break;
					}
				}
			}
			BatteryInfo* bat = &batteryInfos[currentBatInd];
			if( currentBatInd<0 || currentBatInd>=MAX_BATTERYS || bat->available<=0 )
			{
				xSemaphoreGive(BatteryMutex);
				return false;
			}
			*res = bat->totalVoltRaw;
			if(ind)
				*ind = currentBatInd;
			//释放mutex
			xSemaphoreGive(BatteryMutex);
			return true;
		}
		return false;
	}
	bool getCurrentBatteryTotalVoltRawFilted( float* res, int8_t* ind, double TIMEOUT )
	{
		bool unLocked;
		is_Attitude_Control_Enabled(&unLocked);
		if( unLocked && (currentBatInd<0 || currentBatInd>=MAX_BATTERYS) )
			return false;
		
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake(BatteryMutex,TIMEOUT_Ticks) )
		{
			if(!unLocked)
			{	//不在飞行状态才可以切换电池
				for( uint8_t i=0; i<MAX_BATTERYS; ++i )
				{	//找序号最小的电池
					BatteryInfo* bat = &batteryInfos[i];
					if( bat->available > 0 )
					{
						currentBatInd = i;
						break;
					}
				}
			}
			BatteryInfo* bat = &batteryInfos[currentBatInd];
			if( currentBatInd<0 || currentBatInd>=MAX_BATTERYS || bat->available<=0 )
			{
				xSemaphoreGive(BatteryMutex);
				return false;
			}
			*res = bat->totalVoltRawFilted;
			if(ind)
				*ind = currentBatInd;
			//释放mutex
			xSemaphoreGive(BatteryMutex);
			return true;
		}
		return false;
	}
	bool getCurrentBatteryTotalVoltFilted( float* res, int8_t* ind, double TIMEOUT )
	{
		bool unLocked;
		is_Attitude_Control_Enabled(&unLocked);
		if( unLocked && (currentBatInd<0 || currentBatInd>=MAX_BATTERYS) )
			return false;
		
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake(BatteryMutex,TIMEOUT_Ticks) )
		{
			if(!unLocked)
			{	//不在飞行状态才可以切换电池
				for( uint8_t i=0; i<MAX_BATTERYS; ++i )
				{	//找序号最小的电池
					BatteryInfo* bat = &batteryInfos[i];
					if( bat->available > 0 )
					{
						currentBatInd = i;
						break;
					}
				}
			}
			BatteryInfo* bat = &batteryInfos[currentBatInd];
			if( currentBatInd<0 || currentBatInd>=MAX_BATTERYS || bat->available<=0 )
			{
				xSemaphoreGive(BatteryMutex);
				return false;
			}
			*res = bat->totalVoltFilted;
			if(ind)
				*ind = currentBatInd;
			//释放mutex
			xSemaphoreGive(BatteryMutex);
			return true;
		}
		return false;
	}
	//获取最优电池信息
	bool getOptimalBatteryInfo( BatteryInfo* info, int8_t* ind, double TIMEOUT )
	{
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake(BatteryMutex,TIMEOUT_Ticks) )
		{
			int8_t id = -1;
			for( uint8_t i=0; i<MAX_BATTERYS; ++i )
			{	//找序号最小的电池
				BatteryInfo* bat = &batteryInfos[i];
				if( bat->available > 0 )
				{
					id = i;
					break;
				}
			}
			BatteryInfo* bat = &batteryInfos[id];
			if( id<0 || id>=MAX_BATTERYS || bat->available<=0 )
			{
				xSemaphoreGive(BatteryMutex);
				return false;
			}
			*info = *bat;
			if(ind)
				*ind = id;
			//释放mutex
			xSemaphoreGive(BatteryMutex);
			return true;
		}
		return false;
	}
	
	//使用率滤波器
	static BatteryCfg bat1Cfg;
	static TD4_Lite RMPercentFilter1;
	static inline float get_MainBatteryRMPercent( float volt, const BatteryCfg* cfg )
	{
		if( volt > cfg->STVoltage[0] + cfg->VoltP10[0] )
			return 100;
		for( int8_t i = 10; i >= 1 ; --i )
		{
			float P1 = cfg->STVoltage[0] + (&(cfg->VoltP0[0]))[(i-1)*2];					
			if( volt > P1 )
			{
				float P2 = cfg->STVoltage[0] + (&(cfg->VoltP0[0]))[(i-0)*2];
				return (i-1)*10 + 10*( volt - P1 ) / ( P2 - P1 );
			}
		}
		return 0;
	}
/*电池信息*/
	
/*滤波器*/
	static Filter_Butter4_LP acc_filters[3];
	static Filter_Butter4_LP gyro_filters[3];
	bool get_AccelerationNC_filted( vector3<double>* vec, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			vec->set_vector(
				acc_filters[0].get_result(),
				acc_filters[1].get_result(),
				acc_filters[2].get_result());
			UnlockCtrl();
			return true;
		}
		return false;
	} 
	bool get_AngularRateNC_filted( vector3<double>* vec, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			vec->set_vector(
				gyro_filters[0].get_result(),
				gyro_filters[1].get_result(),
				gyro_filters[2].get_result());
			UnlockCtrl();
			return true;
		}
		return false;
	} 
/*滤波器*/	

/*震动数据*/
	static vector3<float> vibration;
	bool get_Vibration( vector3<float>* vec, double TIMEOUT )
	{
		*vec = vibration;
		return true;
	} 
/*震动数据*/
	
void update_ESO_1();
void update_ESO_2();
static void ControlSystem_Task(void* pvParameters)
{
	//等待驱动初始化完成
	while( getInitializationCompleted() == false )
		os_delay(0.1);
	
	//等待姿态解算系统准备完成
	while( get_Attitude_MSStatus() != MS_Ready )
		os_delay(0.1);
	
	//注册电池
	uint32_t bat1Id = batteryRegister(1);
	//等待位置解算系统准备完成
	while( get_Altitude_MSStatus() != MS_Ready )
		os_delay(0.1);
	
	extern volatile bool ESC_Calib_Completed;
	while( ESC_Calib_Completed == false )
		os_delay(0.1);
	
	//根据机型设置初始主通道数目
	uint32_t uav_type[2];
	ReadParam("AC_UAVType", 0, 0, (uint64_t*)uav_type, 0 );
	UAV_MTCount mt_count = UAV_MainMotorCount(uav_type[0]);
	set_MainMotorCount( mt_count.MTCount, mt_count.STCount );
	
	//设置初始电压
	double thr = 0;
	get_OutputThrottle(&thr);
	//电池百分比
	float bat1Percent = 0;
	float bat2Percent = 0;
	/*电池1*/
		ReadParamGroup( "Battery", (uint64_t*)&bat1Cfg, 0 );
		float batVolt = adcGet_MainBaterry1_Voltage();
		float batVoltComp = batVolt - (getCtrlBatId()==1?getVoltKp():0)*thr;
		float batCurrent = adcGet_MainBaterry1_Current();
		bat1Percent = get_MainBatteryRMPercent( batVoltComp, &bat1Cfg );
		batteryUpdate( 1, bat1Id,
										batVolt>5,	//available
										batVolt,	//totalVoltRaw
										batVoltComp, //totalVolt
										bat1Cfg.STVoltage[0],	//stVolt
										batCurrent,	//total current
										0,	//power usage
										bat1Cfg.Capacity[0], //capacity
										bat1Percent,	//percent
										-500,	//temperature
										UINT16_MAX, //cycle count
										0,	//error flags
										0, 0 );
		RMPercentFilter1.reset();
		RMPercentFilter1.x1 = bat1Percent;
	/*电池1*/
	uint16_t VoltMeas_counter = 0;
	
	//滤波器
	acc_filters[0].set_cutoff_frequency( CtrlRateHz, 10 );
	acc_filters[1].set_cutoff_frequency( CtrlRateHz, 10 );
	acc_filters[2].set_cutoff_frequency( CtrlRateHz, 10 );
	gyro_filters[0].set_cutoff_frequency( CtrlRateHz, 30 );
	gyro_filters[1].set_cutoff_frequency( CtrlRateHz, 30 );
	gyro_filters[2].set_cutoff_frequency( CtrlRateHz, 30 );
	
	//分频
	uint16_t div_counter = 0;
	//准确周期延时
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		//800hz
		vTaskDelayUntil( &xLastWakeTime, 1 );
		
		//更新ESO估计1
		update_ESO_1();
		
		if( ++div_counter >= CtrlRateDiv )
		{	//400hz
			div_counter = 0;
			
			if( LockCtrl(0.02) )
			{
				ctrl_Position();
				ctrl_Attitude();
				
				/*陀螺加速度滤波*/
					IMU_Sensor sensor;
					GetAccelerometer( get_current_use_IMUAccelerometer(), &sensor );
					acc_filters[0].run(sensor.data_raw.x*sensor.sensitivity);
					acc_filters[1].run(sensor.data_raw.y*sensor.sensitivity);
					acc_filters[2].run(sensor.data_raw.z*sensor.sensitivity);
					/*包络求震动*/
						for( uint8_t i = 0; i < 3; ++i )
						{
							double vibe = fabs( acc_filters[i].get_result() - sensor.data_raw[i]*sensor.sensitivity );
							if( vibe > vibration[i] )
								vibration[i] = vibe;
							else
								vibration[i] += 3.0f*(1.0f/CtrlRateHz) * ( vibe - vibration[i] );
						}
					/*包络求震动*/
					GetGyroscope( get_current_use_IMUGyroscope(), &sensor );
					gyro_filters[0].run(sensor.data_raw.x*sensor.sensitivity);
					gyro_filters[1].run(sensor.data_raw.y*sensor.sensitivity);
					gyro_filters[2].run(sensor.data_raw.z*sensor.sensitivity);
				/*陀螺加速度滤波*/
				
				UnlockCtrl();
			}
						
			/*电压测量滤波*/
				#define BatMonT 0.2
				#define RMPercentFilterP 0.5
				if( ++VoltMeas_counter >= BatMonT*CtrlRateHz )
				{
					VoltMeas_counter = 0;
					
					/*电池1*/
						ReadParamGroup( "Battery", (uint64_t*)&bat1Cfg, 0 );
						float batVolt = adcGet_MainBaterry1_Voltage();
						float batVoltComp = batVolt - (getCtrlBatId()==1?getVoltKp():0)*thr;
						float batCurrent = adcGet_MainBaterry1_Current();
						bat1Percent = get_MainBatteryRMPercent( batVoltComp, &bat1Cfg );
						batteryUpdate( 1, bat1Id,
														batVolt>5,	//available
														batVolt,	//totalVoltRaw
														batVoltComp, //totalVolt
														bat1Cfg.STVoltage[0],	//stVolt
														batCurrent,	//total current
														0,	//power usage
														bat1Cfg.Capacity[0], //capacity
														RMPercentFilter1.get_x1(),	//percent
														-500,	//temperature
														UINT16_MAX, //cycle count
														0,	//error flags
														0, 0 );
					/*电池1*/
				}
				//电池剩余百分百滤波
				RMPercentFilter1.track4( bat1Percent, 1.0/CtrlRateHz, RMPercentFilterP,RMPercentFilterP,RMPercentFilterP,RMPercentFilterP );
			/*电压测量滤波*/
		}
		
		//更新ESO估计2
		update_ESO_2();
	}
}

#define CtrlMain_StackSize 3500
Static_DTCMBuf __attribute__((aligned(8))) StackType_t CtrlMain_Stack[CtrlMain_StackSize];
Static_DTCMBuf StaticTask_t CtrlMain_TaskBuffer;
void init_ControlSystem()
{
	init_Ctrl_Attitude();
	init_Ctrl_Position();
	//xTaskCreate( ControlSystem_Task, "ControlSystem", 4096, NULL, SysPriority_ControlSystem, &ControlTaskHandle );
	ControlTaskHandle = xTaskCreateStatic( ControlSystem_Task , "ControlSystem" ,CtrlMain_StackSize, NULL, SysPriority_ControlSystem, CtrlMain_Stack, &CtrlMain_TaskBuffer);
	
	/*注册电池参数*/
		bat1Cfg.STVoltage[0] = 11.6;
		bat1Cfg.VoltMKp[0] = 11;
		bat1Cfg.CurrentMKp[0] = 50.0;
		bat1Cfg.Capacity[0] = 300;
		bat1Cfg.VoltP0[0] = -1.0;
		bat1Cfg.VoltP1[0] = -0.6;
		bat1Cfg.VoltP2[0] = -0.4;
		bat1Cfg.VoltP3[0] = -0.2;
		bat1Cfg.VoltP4[0] = -0.0;
		bat1Cfg.VoltP5[0] = +0.1;
		bat1Cfg.VoltP6[0] = +0.2;
		bat1Cfg.VoltP7[0] = +0.3;
		bat1Cfg.VoltP8[0] = +0.4;
		bat1Cfg.VoltP9[0] = +0.5;
		bat1Cfg.VoltP10[0] = +0.6;
	
		MAV_PARAM_TYPE param_types[] = {
			MAV_PARAM_TYPE_REAL32 ,	//STVoltage
			MAV_PARAM_TYPE_REAL32 ,	//VoltMKp
			MAV_PARAM_TYPE_REAL32 ,	//CurrentMKp
			MAV_PARAM_TYPE_REAL32 ,	//Capacity
			MAV_PARAM_TYPE_REAL32 ,	//VoltP0
			MAV_PARAM_TYPE_REAL32 ,	//VoltP1
			MAV_PARAM_TYPE_REAL32 ,	//VoltP2
			MAV_PARAM_TYPE_REAL32 ,	//VoltP3
			MAV_PARAM_TYPE_REAL32 ,	//VoltP4
			MAV_PARAM_TYPE_REAL32 ,	//VoltP5
			MAV_PARAM_TYPE_REAL32 ,	//VoltP6
			MAV_PARAM_TYPE_REAL32 ,	//VoltP7
			MAV_PARAM_TYPE_REAL32 ,	//VoltP8
			MAV_PARAM_TYPE_REAL32 ,	//VoltP9
			MAV_PARAM_TYPE_REAL32 ,	//VoltP10
		};
		SName param_names[] = {
			"Bat_STVoltage" ,	//UAV Type
			"Bat_VoltMKp" ,	//VoltMKp
			"Bat_CurrentMKp" ,	//CurrentMKp
			"Bat_Capacity" ,	//Capacity
			"Bat_VoltP0" ,	//VoltP0
			"Bat_VoltP1" ,	//VoltP1
			"Bat_VoltP2" ,	//VoltP2
			"Bat_VoltP3" ,	//VoltP3
			"Bat_VoltP4" ,	//VoltP4
			"Bat_VoltP5" ,	//VoltP5
			"Bat_VoltP6" ,	//VoltP6
			"Bat_VoltP7" ,	//VoltP7
			"Bat_VoltP8" ,	//VoltP8
			"Bat_VoltP9" ,	//VoltP9
			"Bat_VoltP10" ,	//VoltP10
		};
		ParamGroupRegister( "Battery", 1, 15, param_types, param_names, (uint64_t*)&bat1Cfg );
	/*注册电池参数*/
}