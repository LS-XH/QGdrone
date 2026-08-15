#include "ctrl_Attitude.hpp"
#include "ControlSystem.hpp"
#include "ctrl_Main.hpp"
#include "Parameters.hpp"
#include "MeasurementSystem.hpp"
#include "Sensors.hpp"
#include "TD4.hpp"
#include "ESO_AngularRate.hpp"
#include "ESO_h.hpp"
#include "Filters_LP.hpp"
#include "drv_PWMOut.hpp"
#include "smooth_kp.hpp"
#include "TD3_3D.hpp"
#include "vector2.hpp"
#include "Commulink.hpp"
#include "Avoidance.hpp"
#include "RingQueue.hpp"

#include "StorageSystem.hpp"

/*参数*/
	static PosCtrlCfg cfg;
	
	static AvoidanceCfg avCfg;
/*参数*/	

/*参数接口*/
	const PosCtrlCfg* getPosCtrlCfg()
	{
		return &cfg;
	}
	
	const AvoidanceCfg* getAvCfg()
	{
		return &avCfg;
	}
/*参数接口*/
	
/*历史信息*/
	#define history_length 30
	#define historyLogHz 20
	static const uint8_t history_state_counter_max = CtrlRateHz / historyLogHz;
	static uint8_t history_state_counter = 0;
	static const double CtrlRate_h = 1.0 / CtrlRateHz;
	static RingQueue< vector3<double> > history_PositionErr(history_length);
	static vector3<double> current_PositionErr;
	static inline void update_history_PositionErr( vector3<double> data )
	{
		current_PositionErr = data;
		if( ++history_state_counter >= history_state_counter_max )
		{
			history_state_counter = 0;
			history_PositionErr.push(data);
		}
	}
	bool get_history_PositionErr( vector3<double>* result, double t, double TIMEOUT )
	{
		double delay_ind_f = t*historyLogHz - history_state_counter*CtrlRate_h;
		int16_t delay_ind;
		if( delay_ind_f < -0.5*history_state_counter*CtrlRate_h )
			delay_ind = -1;
		else
			delay_ind = round(t*historyLogHz - history_state_counter*CtrlRate_h);
		if( LockCtrl(TIMEOUT) )
		{
			if( delay_ind < 0 )
				*result = current_PositionErr;
			else
				*result = *history_PositionErr.get_member_ring( delay_ind );
			UnlockCtrl();
			return true;
		}
		return false;	
	}
/*历史信息*/
	
/*控制接口*/
	static bool Altitude_Control_Enabled = false;
	static bool Position_Control_Enabled = false;
	
	//位置控制模式
	static Position_ControlMode Altitude_ControlMode = Position_ControlMode_Position;
	static Position_ControlMode HorizontalPosition_ControlMode = Position_ControlMode_Position;
	
	//转弯半径
	static double TRadius = 0;
	bool Position_Control_set_TurnRaius( double radius, double TIMEOUT )
	{
		if( !isvalid(radius) )
			return false;

		if( LockCtrl(TIMEOUT) )
		{
			TRadius = radius;
			
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool Position_Control_reset_TurnRaius( double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			TRadius = cfg.TRadius[0];
			
			UnlockCtrl();
			return true;
		}
		return false;
	}
	
	//期望TD4滤波器
	static TD4_SL Target_tracker[3];
	static TD3_2DSL Target_tracker_xy;
	//期望速度低通滤波器
	static Filter_Butter4_LP TargetVelocityFilter[3];
	//垂直加速度误差TD4滤波器
	static TD4_Lite AccZ_Err_filter;
	
	/*航线预存*/
		//复位
		static void reset_prestoreMission();
		//直线航线预存
		static bool prestoreMissionLine( vector3<double> pD );
		static bool prestoreMissionLine( vector2<double> pD );
	/*航线预存*/
	
	/*目标模式
		OffBoard中：
			3-位置+速度+加速度控制
			2-速度+加速度控制
	*/
	static uint8_t OffBoardXY_target_mode = 0;
	static uint8_t OffBoardZ_target_mode = 0;
	static vector3<double> target_position;
	static vector3<double> target_velocity;
	static vector3<double> target_acc;
	static double VelCtrlMaxRoll = -1 , VelCtrlMaxPitch = -1;
	static double VelCtrlMaxAcc = -1;
	
	//获取模式及期望位置速度
	bool get_TargetPosInf( Position_ControlMode* pos_mode, Position_ControlMode* alt_mode, 
													vector3<double>* t_pos, vector3<double>* t_vel,
													double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			if(pos_mode)
				*pos_mode = HorizontalPosition_ControlMode;
			if(alt_mode)
				*alt_mode = Altitude_ControlMode;
			if(t_pos)
				*t_pos = target_position;
			if(t_vel)
				*t_vel = target_velocity;
			UnlockCtrl();
			return true;
		}
		return false;
	}
	
	/*高度*/
		bool is_Altitude_Control_Enabled( bool* ena, double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				*ena = Altitude_Control_Enabled;
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Altitude_Control_Enable( double TIMEOUT )
		{
			if( get_Altitude_MSStatus() != MS_Ready )
				return false;
			
			if( LockCtrl(TIMEOUT) )
			{
				if( Altitude_Control_Enabled == true )
				{	//控制器已打开
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_ZCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				Attitude_Control_Enable();
				Altitude_ControlMode = Position_ControlMode_Locking;
				Altitude_Control_Enabled = true;
				
				//更新控制时间
				if(!isMSafe)
					last_ZCtrlTime = TIME::now();
				
				//读参数
				if( ReadParamGroup( "PosCtrl", (uint64_t*)&cfg, 0, TIMEOUT ) != PR_OK )
				{
					UnlockCtrl();
					return false;
				}
				Position_Control_set_ZAutoSpeed( cfg.AutoVZUp[0], cfg.AutoVZDown[0] );
				Target_tracker[2].P1 = cfg.Z_TD4P1[0];
				Target_tracker[2].P2 = cfg.Z_TD4P2[0];
				Target_tracker[2].P3 = cfg.Z_TD4P3[0];
				Target_tracker[2].P4 = cfg.Z_TD4P4[0];
				
				AccZ_Err_filter.reset();
				
				Position_Control_reset_ZAutoSpeed();
				
				//读取避障参数
				ReadParamGroup( "Avoidance", (uint64_t*)&avCfg, 0, TIMEOUT );
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Altitude_Control_Disable( double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				if( Altitude_Control_Enabled == false )
				{
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( ForceMSafeCtrl && !isMSafe )
				{	//屏蔽用户控制
					UnlockCtrl();
					return false;
				}
				Position_Control_Disable();
				Altitude_ControlMode = Position_ControlMode_Null;
				Altitude_Control_Enabled = false;					
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		//获取飞行模式
		bool get_Altitude_ControlMode( Position_ControlMode* mode, double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && (Is_2DAutoMode(Altitude_ControlMode) || Is_3DAutoMode(Altitude_ControlMode)) )
				{	//用户控制访问且为自动模式时更新控制时间
					last_ZCtrlTime = TIME::now();
				}
				*mode = Altitude_ControlMode;
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		//设定Z自动飞行速度
		static double AutoVelZUp = 200;
		static double AutoVelZDown = 200;
		bool Position_Control_get_ZAutoSpeed( double* SpUp, double* SpDown, double TIMEOUT )
		{		
			if( LockCtrl(TIMEOUT) )
			{
				*SpUp = AutoVelZUp;
				*SpDown = AutoVelZDown;
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_reset_ZAutoSpeed( double TIMEOUT )
		{
			vector3<double> pos;
			get_Position_Ctrl(&pos);
			if( LockCtrl(TIMEOUT) )
			{
				AutoVelZUp = cfg.AutoVZUp[0];
				AutoVelZDown = cfg.AutoVZDown[0];
				
				//飞行速度限幅
				if( AutoVelZUp > cfg.maxAutoVelUp[0] )
					AutoVelZUp = cfg.maxAutoVelUp[0];

				if( AutoVelZDown > cfg.maxAutoVelDown[0] )
					AutoVelZDown = cfg.maxAutoVelDown[0];
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_set_ZAutoSpeed( double SpUp, double SpDown, double TIMEOUT )
		{
			if( !isvalid(SpUp) || !isvalid(SpDown) )
				return false;
			
			vector3<double> pos;
			get_Position_Ctrl(&pos);
			if( LockCtrl(TIMEOUT) )
			{
				if( SpUp > 0 )
					AutoVelZUp = SpUp;
				if( SpDown > 0 )
					AutoVelZDown = SpDown;
				
				//飞行速度限幅
				if( AutoVelZUp > cfg.maxAutoVelUp[0] )
					AutoVelZUp = cfg.maxAutoVelUp[0];

				if( AutoVelZDown > cfg.maxAutoVelDown[0] )
					AutoVelZDown = cfg.maxAutoVelDown[0];
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		//移动Z目标位置
		bool Position_Control_move_TargetPositionZRelative( double posz, double TIMEOUT )
		{
			if( !isvalid(posz) )
				return false;
			//必须为位置或者自动模式
			if( Altitude_ControlMode!=Position_ControlMode_Position &&
					Is_2DAutoMode(Altitude_ControlMode)==false &&
					Is_3DAutoMode(Altitude_ControlMode)==false )
				return false;
			
			bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
			if( !isMSafe )
			{	//用户控制访问更新控制时间
				last_ZCtrlTime = TIME::now();
			}
			target_position.z += posz;
			
			return true;
		}
		
		bool Position_Control_set_TargetPositionZ( double posz, double vel, double TIMEOUT )
		{
			if( !isvalid(posz) || !isvalid(vel) )
				return false;
			
			vector3<double> pos;
			get_Position_Ctrl(&pos);
			if( LockCtrl(TIMEOUT) )
			{
				if( Altitude_Control_Enabled == false )
				{
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_ZCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				target_position.z = posz;
				if( Is_2DAutoMode(Altitude_ControlMode)==false && Altitude_ControlMode!=Position_ControlMode_Position )
					Target_tracker[2].x1 = pos.z;
				
				//设定飞行速度
				//飞行速度限幅
				if( target_position.z > pos.z )
				{
					if( vel > 10 )
						AutoVelZUp = vel;
					else if( vel < 0 )
						AutoVelZUp = cfg.AutoVZUp[0];
					
					if( AutoVelZUp > cfg.maxAutoVelUp[0] )
						AutoVelZUp = cfg.maxAutoVelUp[0];
				}
				else
				{
					if( vel > 10 )
						AutoVelZDown = vel;
					else if( vel < 0 )
						AutoVelZDown = cfg.AutoVZDown[0];
					
					if( AutoVelZDown > cfg.maxAutoVelDown[0] )
						AutoVelZDown = cfg.maxAutoVelDown[0];
				}
				
				//切换模式
				Altitude_ControlMode = Position_ControlMode_RouteLine;
			
				//更新控制时间
				if(!isMSafe)
					last_ZCtrlTime = TIME::now();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_set_TargetPositionZGlobal( double posz, double vel, double TIMEOUT )
		{
			if( !isvalid(posz) || !isvalid(vel) )
				return false;
			
			//获取最优全球定位传感器信息
			PosSensorHealthInf1 global_inf;
			if( get_OptimalGlobal_Z( &global_inf ) == false )
				return false;
			posz -= global_inf.HOffset;
			return Position_Control_set_TargetPositionZ( posz, vel, TIMEOUT );
		}
		bool Position_Control_set_TargetPositionZRelative( double posz, double vel, double TIMEOUT )
		{
			if( !isvalid(posz) || !isvalid(vel) )
				return false;
			vector3<double> pos;
			if( get_Position_Ctrl( &pos, TIMEOUT ) == false )
				return false;
			return Position_Control_set_TargetPositionZ( pos.z + posz, vel, TIMEOUT );
		}
		bool Position_Control_set_TargetPositionZRA( double posz, double vel, double TIMEOUT )
		{
			//获取起飞位置Z坐标
			double homeZ;
			getHomeLocalZ(&homeZ);
			return Position_Control_set_TargetPositionZ( homeZ + posz, vel, TIMEOUT );
		}
		
		bool Position_Control_set_TargetVelocityZ( double velz, double TIMEOUT )
		{
			if( !isvalid(velz) )
				return false;
			if( LockCtrl(TIMEOUT) )
			{
				if( Altitude_Control_Enabled == false )
				{	//控制器未打开
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_ZCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				target_velocity.z = velz;
				Altitude_ControlMode = Position_ControlMode_Velocity;
			
				//更新控制时间
				if(!isMSafe)
					last_ZCtrlTime = TIME::now();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_get_TargetVelocityZ( double* velz, double TIMEOUT )
		{
			if( velz == 0 )
				return false;
			if( LockCtrl(TIMEOUT) )
			{
				if( Altitude_Control_Enabled == false )
				{	//控制器未打开
					UnlockCtrl();
					return false;
				}
				if( Altitude_ControlMode != Position_ControlMode_Velocity )
				{
					UnlockCtrl();
					return false;
				}
				*velz = target_velocity.z;
				Altitude_ControlMode = Position_ControlMode_Velocity;
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_set_ZLock( double TIMEOUT )
		{
			vector3<double> pos;
			if( get_Position_Ctrl( &pos, TIMEOUT ) == false )
				return false;
			if( LockCtrl(TIMEOUT) )
			{
				if( Altitude_Control_Enabled == false )
				{	//控制器未打开
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_ZCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				if( Altitude_ControlMode != Position_ControlMode_Position )
					Altitude_ControlMode = Position_ControlMode_Locking;
				Target_tracker[2].x1 = pos.z;
				
				//更新控制时间
				if(!isMSafe)
					last_ZCtrlTime = TIME::now();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		//起飞到当前高度上方的height高度
		static double TakeoffHeight;
		bool Position_Control_Takeoff_HeightRelative( double height, double TIMEOUT )
		{
			if( !isvalid(height) )
				return false;
			if( height < 10 )
					return false;
			bool inFlight;	get_is_inFlight(&inFlight);
			if( inFlight == true )
				return false;
			
			if( LockCtrl(TIMEOUT) )
			{
				if( Altitude_Control_Enabled == false )
				{	//控制器未打开
					UnlockCtrl();
					return false;
				}	
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_ZCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				
				Altitude_ControlMode = Position_ControlMode_Takeoff;
				TakeoffHeight = height;
			
				//更新控制时间
				if(!isMSafe)
					last_ZCtrlTime = TIME::now();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_Takeoff_HeightGlobal( double height, double TIMEOUT )
		{
			//获取最优全球定位传感器信息
			PosSensorHealthInf1 global_inf;
			if( get_OptimalGlobal_Z( &global_inf ) == false )
				return false;
			height -= global_inf.HOffset;
			height -= global_inf.PositionENU.z;
			return Position_Control_Takeoff_HeightRelative( height, TIMEOUT );
		}
		bool Position_Control_Takeoff_Height( double height, double TIMEOUT )
		{
			vector3<double> pos;
			get_Position_Ctrl(&pos);
			height = height - pos.z;
			return Position_Control_Takeoff_HeightRelative( height, TIMEOUT );
		}
	/*高度*/
		
	/*水平位置*/
		static double pos_vel = -1;
		static double XYLock_maxAcc = -1;
		static double XYLock_CAcc = 0;
		static vector2<double> Expected_VelXY;
		static double frontDir = -10;
		static vector3<double> frontVec;
		bool is_Position_Control_Enabled( bool* ena, double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				*ena = Position_Control_Enabled;
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_Enable( double TIMEOUT )
		{
			if( get_Position_MSStatus() != MS_Ready )
				return false;
			
			vector3<double> velocity;
			get_VelocityENU_Ctrl(&velocity);
			if( LockCtrl(TIMEOUT) )
			{
				if( Position_Control_Enabled == true )
				{	//控制器已打开
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_XYCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				Altitude_Control_Enable();
				if( Altitude_Control_Enabled == false )
				{
					UnlockCtrl();
					return false;
				}
				XYLock_CAcc = 0;
				Expected_VelXY.x = velocity.x;
				Expected_VelXY.y = velocity.y;
				HorizontalPosition_ControlMode = Position_ControlMode_Locking;
				Position_Control_Enabled = true;
				
				//更新控制时间
				if(!isMSafe)
					last_XYCtrlTime = TIME::now();
				
				//读参数	
				Target_tracker_xy.P1 = 10;
				Target_tracker_xy.P2 = 10;
				Target_tracker_xy.P3 = 20;
				Target_tracker_xy.r2 = cfg.maxVelXY[0];
				Target_tracker_xy.r3 = cfg.maxAccXY[0];
				Position_Control_reset_XYAutoSpeed();
				Position_Control_reset_XYZAutoSpeed();
				Position_Control_reset_TurnRaius();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_Disable( double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				if( Position_Control_Enabled == false )
				{
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( ForceMSafeCtrl && !isMSafe )
				{	//屏蔽用户控制
					UnlockCtrl();
					return false;
				}
				HorizontalPosition_ControlMode = Position_ControlMode_Null;
				Position_Control_Enabled = false;		
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		bool get_Position_ControlMode( Position_ControlMode* mode, double* fDir, vector3<double>* fVec, double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && (Is_2DAutoMode(HorizontalPosition_ControlMode) || Is_3DAutoMode(HorizontalPosition_ControlMode)) )
				{	//用户控制访问且为自动模式时更新控制时间
					last_XYCtrlTime = TIME::now();
				}
				if(mode)
					*mode = HorizontalPosition_ControlMode;
				if(fDir)
					*fDir = frontDir;
				if(fVec)
					*fVec = frontVec;
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		bool Position_Control_set_TargetVelocityXY_AngleLimit( double velx, double vely, double maxAngle, double TIMEOUT )
		{
			if( !isvalid(velx) || !isvalid(vely) || !isvalid(maxAngle) )
				return false;
			
			if( LockCtrl(TIMEOUT) )
			{
				if( Position_Control_Enabled == false )
				{	//控制器未打开
					UnlockCtrl();
					return false;
				}	
				
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_XYCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				
				constrain_vector( velx, vely, (double)cfg.maxVelXY[0] );
				target_velocity.x = velx;
				target_velocity.y = vely;	
				HorizontalPosition_ControlMode = Position_ControlMode_Velocity;
				VelCtrlMaxRoll = maxAngle;
				if( VelCtrlMaxRoll>=0 && VelCtrlMaxRoll<0.05 )
					VelCtrlMaxRoll = 0.05;
				VelCtrlMaxPitch = -1;

				//更新控制时间
				if(!isMSafe)
					last_XYCtrlTime = TIME::now();
			
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( double velx, double vely, double maxRoll, double maxPitch, double TIMEOUT )
		{
			if( !isvalid(velx) || !isvalid(vely) || !isvalid(maxPitch) || !isvalid(maxRoll) )
				return false;
			
			if( LockCtrl(TIMEOUT) )
			{
				if( Position_Control_Enabled == false )
				{	//控制器未打开
					UnlockCtrl();
					return false;
				}	
				
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_XYCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				
				constrain_vector( velx, vely, (double)cfg.maxVelXY[0] );
				
				double yaw;	double yaw_declination;
				get_YawDeclination(&yaw_declination);
				Attitude_Control_get_TargetTrackYaw(&yaw);
				yaw += yaw_declination;
				double sin_Yaw, cos_Yaw;
				fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
				double velx_ENU = BodyHeading2ENU_x( velx , vely , sin_Yaw , cos_Yaw );
				double vely_ENU = BodyHeading2ENU_y( velx , vely , sin_Yaw , cos_Yaw );
				
				target_velocity.x = velx_ENU;
				target_velocity.y = vely_ENU;	
				HorizontalPosition_ControlMode = Position_ControlMode_Velocity;
				VelCtrlMaxRoll = maxRoll;
				if( VelCtrlMaxRoll>=0 && VelCtrlMaxRoll<0.05 )
					VelCtrlMaxRoll = 0.05;
				VelCtrlMaxPitch = maxPitch;
				if( VelCtrlMaxPitch>=0 && VelCtrlMaxPitch < 0.05 )
					VelCtrlMaxPitch = 0.05;		

				//更新控制时间
				if(!isMSafe)
					last_XYCtrlTime = TIME::now();
			
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		bool Position_Control_set_XYLock( double maxAcc, double TIMEOUT )
		{
			if( !isvalid(maxAcc) )
				return false;
			
			if( LockCtrl(TIMEOUT) )
			{
				if( Position_Control_Enabled == false )
				{
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_XYCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				if( HorizontalPosition_ControlMode != Position_ControlMode_Position )
				{
					XYLock_maxAcc = maxAcc;
					if( HorizontalPosition_ControlMode != Position_ControlMode_Locking )
					{
						XYLock_CAcc = 0;
						HorizontalPosition_ControlMode = Position_ControlMode_Locking;
					}
				}
				
				//更新控制时间
				if(!isMSafe)
					last_XYCtrlTime = TIME::now();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_set_XYLockFast( double maxAcc, double TIMEOUT )
		{
			if( !isvalid(maxAcc) )
				return false;
			
			if( LockCtrl(TIMEOUT) )
			{
				if( Position_Control_Enabled == false )
				{
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_XYCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				if( HorizontalPosition_ControlMode != Position_ControlMode_Position )
				{
					XYLock_maxAcc = maxAcc;
					if( HorizontalPosition_ControlMode != Position_ControlMode_Locking )
					{
						XYLock_CAcc = cfg.maxAccXY[0];
						Expected_VelXY.zero();
						HorizontalPosition_ControlMode = Position_ControlMode_Locking;
					}
				}
				
				//更新控制时间
				if(!isMSafe)
					last_XYCtrlTime = TIME::now();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		/*OffBoard模式*/
			/*XY*/
				bool Position_Control_set_TargetPosVelAccXY_OffBoard( double posx, double posy, double velx, double vely, double accx, double accy, double TIMEOUT )
				{
					if( !isvalid(posx) || !isvalid(posy) ||
							!isvalid(velx) || !isvalid(vely) ||
							!isvalid(accx) || !isvalid(accy) )
						return false;
					if( LockCtrl(TIMEOUT) )
					{
						if( Position_Control_Enabled == false )
						{
							UnlockCtrl();
							return false;
						}
						bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
						if( !isMSafe && ForceMSafeCtrl )
						{	//屏蔽用户控制
							last_XYCtrlTime = TIME::now();
							UnlockCtrl();
							return false;
						}
						
						//设定目标位置
						target_position.x = posx;
						target_position.y = posy;
						
						//设定目标速度
						target_velocity.x = velx;
						target_velocity.y = vely;
						
						//设定目标加速度
						target_acc.x = accx;
						target_acc.y = accy;
						
						HorizontalPosition_ControlMode = Position_ControlMode_OffBoard;
						OffBoardXY_target_mode = 3;
						
						//更新控制时间
						if(!isMSafe)
							last_XYCtrlTime = TIME::now();
						
						UnlockCtrl();
						return true;
					}
					return false;
				}
				bool Position_Control_set_TargetGlobalPosVelAccXY_OffBoard( double Lat, double Lon, double velx, double vely, double accx, double accy, double TIMEOUT )
				{
					if( !isvalid(Lat) || !isvalid(Lon) ||
							!isvalid(velx) || !isvalid(vely) ||
							!isvalid(accx) || !isvalid(accy) )
						return false;
					
					//获取最优全球定位传感器信息
					PosSensorHealthInf3 global_inf;
					if( get_OptimalGlobal_XYZ( &global_inf ) == false )
						return false;
					//获取指定经纬度平面坐标
					double x, y;
					map_projection_project( &global_inf.mp, Lat, Lon, &x, &y );
					x -= global_inf.HOffset.x;
					y -= global_inf.HOffset.y;
					return Position_Control_set_TargetPosVelAccXY_OffBoard( x, y, velx, vely, accx, accy, TIMEOUT );
				}
				bool Position_Control_set_TargetPosRelativeVelAccXY_OffBoard( double posx, double posy, double velx, double vely, double accx, double accy, double TIMEOUT )
				{
					if( !isvalid(posx) || !isvalid(posy) ||
							!isvalid(velx) || !isvalid(vely) ||
							!isvalid(accx) || !isvalid(accy) )
						return false;
					
					if( LockCtrl(TIMEOUT) )
					{
						vector3<double> position;
						if( Is_PosCtrlMode(HorizontalPosition_ControlMode) )
							position = target_position;
						else
							get_Position_Ctrl(&position);
						bool res = Position_Control_set_TargetPosVelAccXY_OffBoard( position.x + posx, position.y + posy, velx, vely, accx, accy, TIMEOUT );
						
						UnlockCtrl();
						return res;
					}
					return false;
				}
				
				bool Position_Control_set_TargetVelAccXY_OffBoard( double velx, double vely, double accx, double accy, double TIMEOUT )
				{
					if( !isvalid(velx) || !isvalid(vely) ||
							!isvalid(accx) || !isvalid(accy) )
						return false;
					if( LockCtrl(TIMEOUT) )
					{
						if( Position_Control_Enabled == false )
						{
							UnlockCtrl();
							return false;
						}
						bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
						if( !isMSafe && ForceMSafeCtrl )
						{	//屏蔽用户控制
							last_XYCtrlTime = TIME::now();
							UnlockCtrl();
							return false;
						}

						//设定目标速度
						target_velocity.x = velx;
						target_velocity.y = vely;
						
						//设定目标加速度
						target_acc.x = accx;
						target_acc.y = accy;
						
						HorizontalPosition_ControlMode = Position_ControlMode_OffBoard;
						OffBoardXY_target_mode = 2;
						
						//更新控制时间
						if(!isMSafe)
							last_XYCtrlTime = TIME::now();
						
						UnlockCtrl();
						return true;
					}
					return false;
				}
				
				bool Position_Control_set_TargetPosVelAccXY_SLAM_OffBoard( uint8_t slamSensorInd, double posx, double posy, double velx, double vely, double accx, double accy, double TIMEOUT )
				{
					if( !isvalid(posx) || !isvalid(posy) || !isvalid(velx) || !isvalid(vely) || !isvalid(accx) || !isvalid(accy) )
						return false;
					
					PosSensorHealthInf2 sensor_inf;
					if( get_PosSensorHealth_XY( &sensor_inf, slamSensorInd ) && sensor_inf.slam_sensor )
					{
						double x, y;
						double yaw = sensor_inf.mp.lat0_rad + sensor_inf.mp.lon0_rad;
						double sin_Yaw, cos_Yaw;
						fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
						x = BodyHeading2ENU_x( posx , posy , sin_Yaw , cos_Yaw );
						y = BodyHeading2ENU_y( posx , posy , sin_Yaw , cos_Yaw );
						x -= sensor_inf.HOffset.x;
						y -= sensor_inf.HOffset.y;
						
						double vx, vy;
						vx = BodyHeading2ENU_x( velx , vely , sin_Yaw , cos_Yaw );
						vy = BodyHeading2ENU_x( velx , vely , sin_Yaw , cos_Yaw );
						
						double ax, ay;
						ax = BodyHeading2ENU_x( accx , accy , sin_Yaw , cos_Yaw );
						ay = BodyHeading2ENU_x( accx , accy , sin_Yaw , cos_Yaw );
						
						return Position_Control_set_TargetPosVelAccXY_OffBoard( x, y, vx, vy, ax, ay, TIMEOUT );
					}
					else	//无传感器信息 或 不是slam传感器
						return false;
				}
				bool Position_Control_set_TargetVelAccXY_SLAM_OffBoard( uint8_t slamSensorInd, double velx, double vely, double accx, double accy, double TIMEOUT )
				{
					if( !isvalid(velx) || !isvalid(vely) || !isvalid(accx) || !isvalid(accy) )
						return false;
					
					PosSensorHealthInf2 sensor_inf;
					if( get_PosSensorHealth_XY( &sensor_inf, slamSensorInd ) && sensor_inf.slam_sensor )
					{
						double x, y;
						double yaw = sensor_inf.mp.lat0_rad + sensor_inf.mp.lon0_rad;
						double sin_Yaw, cos_Yaw;
						fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
						
						double vx, vy;
						vx = BodyHeading2ENU_x( velx , vely , sin_Yaw , cos_Yaw );
						vy = BodyHeading2ENU_x( velx , vely , sin_Yaw , cos_Yaw );
						
						double ax, ay;
						ax = BodyHeading2ENU_x( accx , accy , sin_Yaw , cos_Yaw );
						ay = BodyHeading2ENU_x( accx , accy , sin_Yaw , cos_Yaw );
						
						return Position_Control_set_TargetVelAccXY_OffBoard( vx, vy, ax, ay, TIMEOUT );
					}
					else	//无传感器信息 或 不是slam传感器
						return false;
				}
			/*XY*/
				
			/*Z*/
				bool Position_Control_set_TargetPosVelAccZ_OffBoard( double posz, double velz, double accz, double TIMEOUT )
				{
					if( !isvalid(posz) ||
							!isvalid(velz) ||
							!isvalid(accz))
						return false;
					if( LockCtrl(TIMEOUT) )
					{
						if( Position_Control_Enabled == false )
						{
							UnlockCtrl();
							return false;
						}
						bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
						if( !isMSafe && ForceMSafeCtrl )
						{	//屏蔽用户控制
							last_ZCtrlTime = TIME::now();
							UnlockCtrl();
							return false;
						}
						
						//设定目标位置
						target_position.z = posz;
						
						//设定目标速度
						target_velocity.z = velz;
						
						//设定目标加速度
						target_acc.z = accz;
						
						Altitude_ControlMode = Position_ControlMode_OffBoard;
						OffBoardZ_target_mode = 3;
						
						//更新控制时间
						if(!isMSafe)
							last_ZCtrlTime = TIME::now();
						
						UnlockCtrl();
						return true;
					}
					return false;
				}
				bool Position_Control_set_TargetPosGlobalVelAccZ_OffBoard( double posz, double velz, double accz, double TIMEOUT )
				{
					if( !isvalid(posz) || !isvalid(velz) || !isvalid(accz) )
						return false;
					
					//获取最优全球定位传感器信息
					PosSensorHealthInf1 global_inf;
					if( get_OptimalGlobal_Z( &global_inf ) == false )
						return false;
					posz -= global_inf.HOffset;
					return Position_Control_set_TargetPosVelAccZ_OffBoard( posz, velz, accz, TIMEOUT );
				}
				bool Position_Control_set_TargetPosRelativeVelAccZ_OffBoard( double posz, double velz, double accz, double TIMEOUT )
				{
					if( !isvalid(posz) || !isvalid(velz) || !isvalid(accz) )
						return false;
					
					vector3<double> pos;
					if( get_Position_Ctrl( &pos, TIMEOUT ) == false )
						return false;
					if( LockCtrl(TIMEOUT) )
					{
						if( Is_PosCtrlMode(Altitude_ControlMode) )
							pos.z = target_position.z;
						bool res = Position_Control_set_TargetPosVelAccZ_OffBoard( pos.z + posz, velz, accz, TIMEOUT );
						
						UnlockCtrl();
						return res;
					}
					return false;
				}
				bool Position_Control_set_TargetPosZRAVelAccZ_OffBoard( double posz, double velz, double accz, double TIMEOUT )
				{
					//获取起飞位置Z坐标
					double homeZ;
					getHomeLocalZ(&homeZ);
					return Position_Control_set_TargetPosVelAccZ_OffBoard( homeZ + posz, velz, accz, TIMEOUT );
				}
				
				bool Position_Control_set_TargetVelAccZ_OffBoard( double velz, double accz, double TIMEOUT )
				{
					if( !isvalid(velz) ||
							!isvalid(accz))
						return false;
					if( LockCtrl(TIMEOUT) )
					{
						if( Position_Control_Enabled == false )
						{
							UnlockCtrl();
							return false;
						}
						bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
						if( !isMSafe && ForceMSafeCtrl )
						{	//屏蔽用户控制
							last_ZCtrlTime = TIME::now();
							UnlockCtrl();
							return false;
						}
						
						//设定目标速度
						target_velocity.z = velz;
						
						//设定目标加速度
						target_acc.z = accz;
						
						Altitude_ControlMode = Position_ControlMode_OffBoard;
						OffBoardZ_target_mode = 2;
						
						//更新控制时间
						if(!isMSafe)
							last_ZCtrlTime = TIME::now();
						
						UnlockCtrl();
						return true;
					}
					return false;
				}
				
				bool Position_Control_set_TargetPosVelAccZ_SLAM_OffBoard( uint8_t slamSensorInd, double posz, double velz, double accz, double TIMEOUT )
				{
					if( !isvalid(posz) || !isvalid(velz) || !isvalid(accz) )
						return false;
					
					PosSensorHealthInf1 sensor_inf;
					if( get_PosSensorHealth_Z( &sensor_inf, slamSensorInd ) && sensor_inf.slam_sensor )
					{
						double z = posz - sensor_inf.HOffset;
						
						return Position_Control_set_TargetPosVelAccZ_OffBoard( z, velz, accz, TIMEOUT );
					}
					else	//无传感器信息 或 不是slam传感器
						return false;
				}
				bool Position_Control_set_TargetVelAccZ_SLAM_OffBoard( uint8_t slamSensorInd, double velz, double accz, double TIMEOUT )
				{
					if( !isvalid(velz) || !isvalid(accz) )
						return false;
					
					PosSensorHealthInf1 sensor_inf;
					if( get_PosSensorHealth_Z( &sensor_inf, slamSensorInd ) && sensor_inf.slam_sensor )
					{
						return Position_Control_set_TargetVelAccZ_OffBoard( velz, accz, TIMEOUT );
					}
					else	//无传感器信息 或 不是slam传感器
						return false;
				}
			/*Z*/
				
			/*XYZ*/
				bool Position_Control_set_TargetPosVelAccXYZ_OffBoard( double posx, double posy, double posz, double velx, double vely, double velz, double accx, double accy, double accz, double TIMEOUT )
				{
					if( !isvalid(posx) || !isvalid(posy) || !isvalid(posz) ||
							!isvalid(velx) || !isvalid(vely) || !isvalid(velz) ||
							!isvalid(accx) || !isvalid(accy) || !isvalid(accz) )
						return false;
					if( LockCtrl(TIMEOUT) )
					{
						if( Position_Control_Enabled == false )
						{
							UnlockCtrl();
							return false;
						}
						bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
						if( !isMSafe && ForceMSafeCtrl )
						{	//屏蔽用户控制
							last_XYCtrlTime = TIME::now();
							UnlockCtrl();
							return false;
						}
						
						//设定目标位置
						target_position.x = posx;
						target_position.y = posy;
						target_position.z = posz;
						
						//设定目标速度
						target_velocity.x = velx;
						target_velocity.y = vely;
						target_velocity.z = velz;
						
						//设定目标加速度
						target_acc.x = accx;
						target_acc.y = accy;
						target_acc.z = accz;
						
						HorizontalPosition_ControlMode = Position_ControlMode_OffBoard3D;
						OffBoardXY_target_mode = 3;
						
						//更新控制时间
						if(!isMSafe)
							last_XYCtrlTime = TIME::now();
						
						UnlockCtrl();
						return true;
					}
					return false;
				}
				bool Position_Control_set_TargetGlobalPosVelAccXYZ_OffBoard( double Lat, double Lon, double posz, double velx, double vely, double velz, double accx, double accy, double accz, double TIMEOUT )
				{
					if( !isvalid(Lat) || !isvalid(Lon) || !isvalid(posz) ||
							!isvalid(velx) || !isvalid(vely) || !isvalid(velz) ||
							!isvalid(accx) || !isvalid(accy) || !isvalid(accz) )
						return false;
					
					//获取最优全球定位传感器信息
					PosSensorHealthInf3 global_inf;
					if( get_OptimalGlobal_XYZ( &global_inf ) == false )
						return false;
					//获取指定经纬度平面坐标
					double x, y;
					map_projection_project( &global_inf.mp, Lat, Lon, &x, &y );
					x -= global_inf.HOffset.x;
					y -= global_inf.HOffset.y;
					posz -= global_inf.HOffset.z;
					return Position_Control_set_TargetPosVelAccXYZ_OffBoard( x, y, posz, velx, vely, velz, accx, accy, accz, TIMEOUT );
				}
				bool Position_Control_set_TargetGlobalPosVelAccXYZLocal_OffBoard( double Lat, double Lon, double posz, double velx, double vely, double velz, double accx, double accy, double accz, double TIMEOUT )
				{
					if( !isvalid(Lat) || !isvalid(Lon) || !isvalid(posz) ||
							!isvalid(velx) || !isvalid(vely) || !isvalid(velz) ||
							!isvalid(accx) || !isvalid(accy) || !isvalid(accz) )
						return false;
					
					//获取最优全球定位传感器信息
					PosSensorHealthInf3 global_inf;
					if( get_OptimalGlobal_XYZ( &global_inf ) == false )
						return false;
					//获取指定经纬度平面坐标
					double x, y;
					map_projection_project( &global_inf.mp, Lat, Lon, &x, &y );
					x -= global_inf.HOffset.x;
					y -= global_inf.HOffset.y;
					return Position_Control_set_TargetPosVelAccXYZ_OffBoard( x, y, posz, velx, vely, velz, accx, accy, accz, TIMEOUT );
				}
				bool Position_Control_set_TargetPosRelativeVelAccXYZ_OffBoard( double posx, double posy, double posz, double velx, double vely, double velz, double accx, double accy, double accz, double TIMEOUT )
				{
					if( !isvalid(posx) || !isvalid(posy) || !isvalid(posz) ||
							!isvalid(velx) || !isvalid(vely) || !isvalid(velz) ||
							!isvalid(accx) || !isvalid(accy) || !isvalid(accz) )
						return false;
					
					if( LockCtrl(TIMEOUT) )
					{
						vector3<double> position;
						if( Is_PosCtrlMode(HorizontalPosition_ControlMode) && Is_PosCtrlMode(Altitude_ControlMode) )
							position = target_position;
						else
							get_Position_Ctrl(&position);
						bool res = Position_Control_set_TargetPosVelAccXYZ_OffBoard( position.x + posx, position.y + posy, position.z + posz, velx, vely, velz, accx, accy, accz, TIMEOUT );
						
						UnlockCtrl();
						return res;
					}
					return false;
				}
				
				bool Position_Control_set_TargetVelAccXYZ_OffBoard( double velx, double vely, double velz, double accx, double accy, double accz, double TIMEOUT )
				{
					if( !isvalid(velx) || !isvalid(vely) || !isvalid(velz) ||
							!isvalid(accx) || !isvalid(accy) || !isvalid(accz) )
						return false;
					if( LockCtrl(TIMEOUT) )
					{
						if( Position_Control_Enabled == false )
						{
							UnlockCtrl();
							return false;
						}
						bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
						if( !isMSafe && ForceMSafeCtrl )
						{	//屏蔽用户控制
							last_XYCtrlTime = TIME::now();
							UnlockCtrl();
							return false;
						}

						//设定目标速度
						target_velocity.x = velx;
						target_velocity.y = vely;
						target_velocity.z = velz;
						
						//设定目标加速度
						target_acc.x = accx;
						target_acc.y = accy;
						target_acc.z = accz;
						
						HorizontalPosition_ControlMode = Position_ControlMode_OffBoard3D;
						OffBoardXY_target_mode = 2;
						
						//更新控制时间
						if(!isMSafe)
							last_XYCtrlTime = TIME::now();
						
						UnlockCtrl();
						return true;
					}
					return false;
				}
				
				bool Position_Control_set_TargetPosVelAccXYZ_SLAM_OffBoard( uint8_t slamSensorInd, double posx, double posy, double posz, double velx, double vely, double velz, double accx, double accy, double accz, double TIMEOUT )
				{
					if( !isvalid(posx) || !isvalid(posy) || !isvalid(velx) || !isvalid(vely) || !isvalid(accx) || !isvalid(accy) )
						return false;
					
					PosSensorHealthInf3 sensor_inf;
					if( get_PosSensorHealth_XYZ( &sensor_inf, slamSensorInd ) && sensor_inf.slam_sensor )
					{
						double x, y;
						double yaw = sensor_inf.mp.lat0_rad + sensor_inf.mp.lon0_rad;
						double sin_Yaw, cos_Yaw;
						fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
						x = BodyHeading2ENU_x( posx , posy , sin_Yaw , cos_Yaw );
						y = BodyHeading2ENU_y( posx , posy , sin_Yaw , cos_Yaw );
						x -= sensor_inf.HOffset.x;
						y -= sensor_inf.HOffset.y;
						double z = posz - sensor_inf.HOffset.z;
						
						double vx, vy;
						vx = BodyHeading2ENU_x( velx , vely , sin_Yaw , cos_Yaw );
						vy = BodyHeading2ENU_x( velx , vely , sin_Yaw , cos_Yaw );
						
						double ax, ay;
						ax = BodyHeading2ENU_x( accx , accy , sin_Yaw , cos_Yaw );
						ay = BodyHeading2ENU_x( accx , accy , sin_Yaw , cos_Yaw );
						
						return Position_Control_set_TargetPosVelAccXYZ_OffBoard( x, y, z, vx, vy, velz, ax, ay, accz, TIMEOUT );
					}
					else	//无传感器信息 或 不是slam传感器
						return false;
				}
				bool Position_Control_set_TargetVelAccXYZ_SLAM_OffBoard( uint8_t slamSensorInd, double velx, double vely, double velz, double accx, double accy, double accz, double TIMEOUT )
				{
					if( !isvalid(velx) || !isvalid(vely) || !isvalid(accx) || !isvalid(accy) )
						return false;
					
					PosSensorHealthInf3 sensor_inf;
					if( get_PosSensorHealth_XYZ( &sensor_inf, slamSensorInd ) && sensor_inf.slam_sensor )
					{
						double x, y;
						double yaw = sensor_inf.mp.lat0_rad + sensor_inf.mp.lon0_rad;
						double sin_Yaw, cos_Yaw;
						fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
						
						double vx, vy;
						vx = BodyHeading2ENU_x( velx , vely , sin_Yaw , cos_Yaw );
						vy = BodyHeading2ENU_x( velx , vely , sin_Yaw , cos_Yaw );
						
						double ax, ay;
						ax = BodyHeading2ENU_x( accx , accy , sin_Yaw , cos_Yaw );
						ay = BodyHeading2ENU_x( accx , accy , sin_Yaw , cos_Yaw );
						
						return Position_Control_set_TargetVelAccXYZ_OffBoard( vx, vy, velz, ax, ay, accz, TIMEOUT );
					}
					else	//无传感器信息 或 不是slam传感器
						return false;
				}
			/*XYZ*/
		/*OffBoard模式*/
		
		/*手动绕圈模式*/
			static double ManualCircleVel = 0;
			static double ManualCircleR = 500;
			static vector2<double> ManualCircleOrigin;
			bool Position_Control_do_ManualCircleRelative( double dCircleVel, double dCircleR, double dCircleO, double TIMEOUT )
			{
				if( !isvalid(dCircleVel) || !isvalid(dCircleR) || !isvalid(dCircleO) )
					return false;
				
				if( LockCtrl(TIMEOUT) )
				{
					if( Position_Control_Enabled == false )
					{	//控制器未打开
						UnlockCtrl();
						return false;
					}	
					
					bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
					if( !isMSafe && ForceMSafeCtrl )
					{	//屏蔽用户控制
						last_XYCtrlTime = TIME::now();
						UnlockCtrl();
						return false;
					}
					
					if( HorizontalPosition_ControlMode!=Position_ControlMode_Position && 
							HorizontalPosition_ControlMode!=Position_ControlMode_ManualCircle )				
					{	//未锁定位置先锁定
						UnlockCtrl();
						Position_Control_set_XYLock();
						return false;
					}		
					
					Quaternion attitude;
					if( get_AirframeY_quat( &attitude, TIMEOUT ) == false )
					{
						UnlockCtrl();
						return false;
					}
					
					vector3<double> position;
					if( get_Position_Ctrl( &position, TIMEOUT ) == false )
					{
						UnlockCtrl();
						return false;
					}
					
					if( HorizontalPosition_ControlMode!=Position_ControlMode_ManualCircle )
					{	//初始进入模式
						ManualCircleVel = 0;
						ManualCircleR = 500;
						
						double yaw = attitude.getYaw();
						double sin_Yaw, cos_Yaw;
						fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
						ManualCircleOrigin.x = position.x + BodyHeading2ENU_x( ManualCircleR , 0 , sin_Yaw , cos_Yaw );
						ManualCircleOrigin.y = position.y + BodyHeading2ENU_y( ManualCircleR , 0 , sin_Yaw , cos_Yaw );
						
						HorizontalPosition_ControlMode = Position_ControlMode_ManualCircle;
					}									
					
					if( !is_zero(dCircleO) && ManualCircleR+dCircleO>=20 )
					{	//移动圆心
						vector2<double> RVec = vector2<double>(position.x,position.y) - ManualCircleOrigin;
						double RVec_length = safe_sqrt(RVec.get_square());					
						vector2<double> RVec_normed;
						if( RVec_length > 0.1 )
							RVec_normed = RVec * (1.0/RVec_length);
						ManualCircleOrigin -= RVec_normed*dCircleO;
						ManualCircleR += dCircleO;
					}
					//限制最大速度
					ManualCircleVel += dCircleVel;
					ManualCircleR += dCircleR;
					if( ManualCircleR < 20 )
						ManualCircleR = 20;
					double max_vel = safe_sqrt(0.7*cfg.maxAccXY[0]*ManualCircleR);
					if( max_vel > cfg.maxVelXY[0] )
						max_vel = cfg.maxVelXY[0];
					if( fabs(ManualCircleVel) > max_vel )
						ManualCircleVel = sign(ManualCircleVel)*max_vel;

					//更新控制时间
					if(!isMSafe)
						last_XYCtrlTime = TIME::now();
				
					UnlockCtrl();
					return true;
				}
				return false;
			}
		/*手动绕圈模式*/
			
		static double AutoVelXY = 500;
		static double AutoVelXYZ = 500;
		/*直线飞行*/
			//直线方程系数
			//A=(x1,y1,z1)=target_position目标点
			//B=(x2,y2,z2)=起点
			static vector3<double> route_line_A_B;	//(B-A)
			static double route_line_m;	// 1/(B-A)^2
			static double line_track_desired = 0;
			static double line_track_desired_vel = 0;
			static double line_track_desired_maxv = 500;
			static double line_track_desired_maxacc = 300;
			static uint16_t line_track_delay_counter = 0;
			#define reset_line_track_state(maxv,maxacc,desired_vel) { line_track_delay_counter = 0;\
																																line_track_desired = 0;\
																																line_track_desired_vel = desired_vel;\
																																line_track_desired_maxv = maxv;\
																																line_track_desired_maxacc = maxacc; }
			
			//直线飞行障碍限制
			static double max_line_track_desired = -1;
			static double max_line_track_desired_vel = -1;
			bool Position_Control_set_RouteLineAvoidanceRelative( double pos, double vel, double TIMEOUT )
			{
				if( !isvalid(pos) || !isvalid(vel) )
					return false;
				if( HorizontalPosition_ControlMode!=Position_ControlMode_RouteLine3D && HorizontalPosition_ControlMode!=Position_ControlMode_RouteLine )
					return false;

				vector3<double> position;
				get_Position_Ctrl(&position);
				if( LockCtrl(TIMEOUT) )
				{
					if( pos < 0 )
						max_line_track_desired = -1;
					else {
						
						//计算垂足
						vector3<double> A_C = position - target_position;
						double k = (A_C * route_line_A_B) * route_line_m;
						vector3<double> foot_point = (route_line_A_B * k) + target_position;
						
						//计算偏差
						vector3<double> B = target_position + route_line_A_B;
						vector3<double> dis = foot_point - B;
						double distance = safe_sqrt(dis.get_square());
						
						//移动目标点
						if( pos>0 || max_line_track_desired>distance+pos )
							max_line_track_desired = distance + pos;
					}
					
					if( vel < 0 )
						max_line_track_desired_vel = -1;
					else {
						max_line_track_desired_vel = vel;
					}
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
		
			bool Position_Control_set_TargetPositionXY( double posx, double posy, double vel, MissionPrestoreMode prestore, bool* prestored, double TIMEOUT )
			{
				if( !isvalid(posx) || !isvalid(posy) || !isvalid(vel) )
					return false;
				
				if( LockCtrl(TIMEOUT) )
				{
					if( Position_Control_Enabled == false )
					{
						UnlockCtrl();
						return false;
					}
					bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
					if( !isMSafe && ForceMSafeCtrl )
					{	//屏蔽用户控制
						last_XYCtrlTime = TIME::now();
						UnlockCtrl();
						return false;
					}
					
					if( prestore )
					{	//航点预存
						bool res = prestoreMissionLine( vector2<double>( posx, posy ) );
						if(prestored)
							*prestored = res;
						if( res || prestore==MissionPrestoreMode_Prestore )
						{
							UnlockCtrl();
							return res;
						}
					}
					else
						reset_prestoreMission();
					
					vector3<double> position;
					if( HorizontalPosition_ControlMode==Position_ControlMode_Position )
						position = target_position;
					else
						get_Position_Ctrl(&position);
					target_position.x = posx;
					target_position.y = posy;
					
					//calculate vector B-A
					route_line_A_B = position - target_position;
					route_line_A_B.z = 0;
					double route_line_A_B_sq = route_line_A_B.get_square();
					if( route_line_A_B_sq > 0.1 )
					{
						//calculate 1/(B-A)^2
						route_line_m = 1.0 / route_line_A_B_sq;
						
						//设置速度
						if( vel > 10 )
							AutoVelXY = vel;
						else if( vel < 0 )
							AutoVelXY = cfg.AutoVXY[0];
						
						//速度限幅
						if( AutoVelXY > cfg.maxAutoVelXY[0] )
							AutoVelXY = cfg.maxAutoVelXY[0];
						
						double desired_vel = 0;
						if( HorizontalPosition_ControlMode != Position_ControlMode_RouteLine )
						{
							//desired_vel = line_track_desired_vel;
							//切换模式
							HorizontalPosition_ControlMode = Position_ControlMode_RouteLine;
						}
						reset_line_track_state(AutoVelXY,cfg.maxAutoAccXY[0],desired_vel)
					}
					else
						HorizontalPosition_ControlMode = Position_ControlMode_Position;
				
					//复位航线避障
					max_line_track_desired = -1;
					max_line_track_desired_vel = -1;
					
					//更新控制时间
					if(!isMSafe)
						last_XYCtrlTime = TIME::now();
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			
			bool Position_Control_set_TargetPositionXYZ( double posx, double posy, double posz, double vel, MissionPrestoreMode prestore, bool* prestored, double TIMEOUT )
			{
				if( !isvalid(posx) || !isvalid(posy) || !isvalid(posz) || !isvalid(vel) )
					return false;
				if( LockCtrl(TIMEOUT) )
				{
					if( Position_Control_Enabled == false )
					{
						UnlockCtrl();
						return false;
					}
					bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
					if( !isMSafe && ForceMSafeCtrl )
					{	//屏蔽用户控制
						last_XYCtrlTime = last_ZCtrlTime = TIME::now();
						UnlockCtrl();
						return false;
					}
					
					if( prestore )
					{	//航点预存
						bool res = prestoreMissionLine( vector3<double>( posx, posy, posz ) );
						if(prestored)
							*prestored = res;
						if( res || prestore==MissionPrestoreMode_Prestore )
						{
							UnlockCtrl();
							return res;
						}
					}
					else
						reset_prestoreMission();
					
					vector3<double> position;
					if( HorizontalPosition_ControlMode==Position_ControlMode_Position && Altitude_ControlMode==Position_ControlMode_Position )
						position = target_position;
					else
						get_Position_Ctrl(&position);
					target_position.x = posx;
					target_position.y = posy;
					target_position.z = posz;
					
					//calculate vector B-A
					route_line_A_B = position - target_position;
					double route_line_A_B_sq = route_line_A_B.get_square();
					if( route_line_A_B_sq > 0.1 )
					{
						//calculate 1/(B-A)^2
						route_line_m = 1.0 / route_line_A_B_sq;
						
						//设置速度
						if( vel > 10 )
							AutoVelXYZ = vel;
						else if( vel < 0 )
							AutoVelXYZ = cfg.AutoVXYZ[0];
						
						HorizontalPosition_ControlMode = Altitude_ControlMode = Position_ControlMode_RouteLine3D;
						reset_line_track_state(AutoVelXYZ,cfg.maxAutoAccXY[0],0)
					}
					else
						HorizontalPosition_ControlMode = Altitude_ControlMode = Position_ControlMode_Position;
				
					//复位航线避障
					max_line_track_desired = -1;
					max_line_track_desired_vel = -1;
					
					//更新控制时间
					if(!isMSafe)
						last_XYCtrlTime = last_ZCtrlTime = TIME::now();
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			
			bool Position_Control_set_TargetPositionXYRelative( double posx, double posy, double vel, MissionPrestoreMode prestore, bool* prestored, double TIMEOUT )
			{
				if( !isvalid(posx) || !isvalid(posy) || !isvalid(vel) )
					return false;
				
				if( LockCtrl(TIMEOUT) )
				{
					if( Position_Control_Enabled == false )
					{
						UnlockCtrl();
						return false;
					}
					bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
					if( !isMSafe && ForceMSafeCtrl )
					{	//屏蔽用户控制
						last_XYCtrlTime = TIME::now();
						UnlockCtrl();
						return false;
					}
					
					vector3<double> position;
					if( HorizontalPosition_ControlMode==Position_ControlMode_Position )
						position = target_position;
					else
						get_Position_Ctrl(&position);
					target_position.x = position.x + posx;
					target_position.y = position.y + posy;
					
					if( prestore )
					{	//航点预存
						bool res = prestoreMissionLine( vector2<double>( position.x + posx, position.y + posy ) );
						if(prestored)
							*prestored = res;
						if( res || prestore==MissionPrestoreMode_Prestore )
						{
							UnlockCtrl();
							return res;
						}
					}
					else
						reset_prestoreMission();
					
					//calculate vector B-A
					route_line_A_B = position - target_position;
					route_line_A_B.z = 0;
					double route_line_A_B_sq = route_line_A_B.get_square();
					if( route_line_A_B_sq > 0.1 )
					{
						//calculate 1/(B-A)^2
						route_line_m = 1.0 / route_line_A_B_sq;
						
						//设置速度
						if( vel > 10 )
							AutoVelXY = vel;
						else if( vel < 0 )
							AutoVelXY = cfg.AutoVXY[0];
						
						//速度限幅
						if( AutoVelXY > cfg.maxAutoVelXY[0] )
							AutoVelXY = cfg.maxAutoVelXY[0];
						
						double desired_vel = 0;
						if( HorizontalPosition_ControlMode != Position_ControlMode_RouteLine )
						{
							//desired_vel = line_track_desired_vel;
							//切换模式
							HorizontalPosition_ControlMode = Position_ControlMode_RouteLine;
						}
						reset_line_track_state(AutoVelXY,cfg.maxAutoAccXY[0],desired_vel)
					}
					else
						HorizontalPosition_ControlMode = Position_ControlMode_Position;
				
					//复位航线避障
					max_line_track_desired = -1;
					max_line_track_desired_vel = -1;
					
					//更新控制时间
					if(!isMSafe)
						last_XYCtrlTime = TIME::now();
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			bool Position_Control_set_TargetPositionXYZRelative( double posx, double posy, double posz, double vel, MissionPrestoreMode prestore, bool* prestored, double TIMEOUT )
			{
				if( !isvalid(posx) || !isvalid(posy) || !isvalid(posz) || !isvalid(vel) )
					return false;
				
				if( LockCtrl(TIMEOUT) )
				{
					if( Position_Control_Enabled == false )
					{
						UnlockCtrl();
						return false;
					}
					bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
					if( !isMSafe && ForceMSafeCtrl )
					{	//屏蔽用户控制
						last_XYCtrlTime = last_ZCtrlTime = TIME::now();
						UnlockCtrl();
						return false;
					}
					
					vector3<double> position;
					if( HorizontalPosition_ControlMode==Position_ControlMode_Position && Altitude_ControlMode==Position_ControlMode_Position )
						position = target_position;
					else
						get_Position_Ctrl(&position);			

					if( prestore )
					{	//航点预存
						bool res = prestoreMissionLine( vector3<double>( position.x + posx, position.y + posy, position.z + posz ) );
						if(prestored)
							*prestored = res;
						if( res || prestore==MissionPrestoreMode_Prestore )
						{
							UnlockCtrl();
							return res;
						}
					}
					else
						reset_prestoreMission();
					
					target_position.x = position.x + posx;
					target_position.y = position.y + posy;
					target_position.z = position.z + posz;
					
					//calculate vector B-A
					route_line_A_B = position - target_position;
					double route_line_A_B_sq = route_line_A_B.get_square();
					if( route_line_A_B_sq > 0.1 )
					{
						//calculate 1/(B-A)^2
						route_line_m = 1.0 / route_line_A_B_sq;
						
						//设置速度
						if( vel > 10 )
							AutoVelXYZ = vel;
						else if( vel < 0 )
							AutoVelXYZ = cfg.AutoVXYZ[0];
						
						double desired_vel = 0;
						if( HorizontalPosition_ControlMode!=Position_ControlMode_RouteLine3D || Altitude_ControlMode!=Position_ControlMode_RouteLine3D )
						{
							//desired_vel = line_track_desired_vel;
							//切换模式
							HorizontalPosition_ControlMode = Altitude_ControlMode = Position_ControlMode_RouteLine3D;
						}
						reset_line_track_state(AutoVelXYZ,cfg.maxAutoAccXY[0],desired_vel)
					}
					else
						HorizontalPosition_ControlMode = Altitude_ControlMode = Position_ControlMode_Position;
				
					//复位航线避障
					max_line_track_desired = -1;
					max_line_track_desired_vel = -1;
					
					//更新控制时间
					if(!isMSafe)
						last_XYCtrlTime = last_ZCtrlTime = TIME::now();
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			
			//移动XY目标位置
			bool Position_Control_move_TargetPositionXYRelative( double posx, double posy, double TIMEOUT )
			{
				if( !isvalid(posx) || !isvalid(posy) )
					return false;
				
				if( LockCtrl(TIMEOUT) )
				{				
					//必须为位置或者自动模式
					if( HorizontalPosition_ControlMode!=Position_ControlMode_Position &&
							Is_2DAutoMode(HorizontalPosition_ControlMode)==false &&
							Is_3DAutoMode(HorizontalPosition_ControlMode)==false )
					{
						UnlockCtrl();
						return false;
					}
					
					bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
					if( !isMSafe && ForceMSafeCtrl )
					{	//屏蔽用户控制
						last_XYCtrlTime = TIME::now();
						UnlockCtrl();
						return false;
					}	
					
					target_position.x += posx;
					target_position.y += posy;
					
					if( !isMSafe )
					{	//用户控制访问更新控制时间
						last_XYCtrlTime = TIME::now();
					}
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			bool Position_Control_move_TargetPositionXYRelativeBodyheading( double posx, double posy, double TIMEOUT )
			{
				Quaternion att;
				get_Attitude_quat( &att );
				double yaw = att.getYaw();
				double sin_Yaw, cos_Yaw;
				fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
				double posx_enu = BodyHeading2ENU_x( posx , posy , sin_Yaw , cos_Yaw );
				double posy_enu = BodyHeading2ENU_y( posx , posy , sin_Yaw , cos_Yaw );
				return Position_Control_move_TargetPositionXYRelative( posx_enu, posy_enu, TIMEOUT );
			}
			
			bool Position_Control_set_TargetPositionXYRelativeBodyheading( double posx, double posy, double vel, MissionPrestoreMode prestore, bool* prestored, double TIMEOUT )
			{
				Quaternion att;
				get_Attitude_quat( &att );
				double yaw = att.getYaw();
				double sin_Yaw, cos_Yaw;
				fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
				double posx_enu = BodyHeading2ENU_x( posx , posy , sin_Yaw , cos_Yaw );
				double posy_enu = BodyHeading2ENU_y( posx , posy , sin_Yaw , cos_Yaw );
				return Position_Control_set_TargetPositionXYRelative( posx_enu, posy_enu, vel, prestore, prestored, TIMEOUT );
			}
			bool Position_Control_set_TargetPositionXYZRelativeBodyheading( double posx, double posy, double posz, double vel, MissionPrestoreMode prestore, bool* prestored, double TIMEOUT )
			{
				Quaternion att;
				get_Attitude_quat( &att );
				double yaw = att.getYaw();
				double sin_Yaw, cos_Yaw;
				fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
				double posx_enu = BodyHeading2ENU_x( posx , posy , sin_Yaw , cos_Yaw );
				double posy_enu = BodyHeading2ENU_y( posx , posy , sin_Yaw , cos_Yaw );
				return Position_Control_set_TargetPositionXYZRelative( posx_enu, posy_enu, posz, vel, prestore, prestored, TIMEOUT );
			}
			
			bool Position_Control_set_TargetPositionXY_LatLon( double Lat, double Lon, double vel, MissionPrestoreMode prestore, bool* prestored, double TIMEOUT )
			{
				if( !isvalid(Lat) || !isvalid(Lon) || !isvalid(vel) )
					return false;
				
				//获取最优全球定位传感器信息
				PosSensorHealthInf2 global_inf;
				if( get_OptimalGlobal_XY( &global_inf ) == false )
					return false;
				//获取指定经纬度平面坐标
				double x, y;
				map_projection_project( &global_inf.mp, Lat, Lon, &x, &y );
				x -= global_inf.HOffset.x;
				y -= global_inf.HOffset.y;
				return Position_Control_set_TargetPositionXY( x, y, vel, prestore, prestored, TIMEOUT );
			}
			bool Position_Control_set_TargetPositionXY_SLAM( uint8_t slamSensorInd, double posx, double posy, double vel, MissionPrestoreMode prestore, bool* prestored, double TIMEOUT )
			{
				if( !isvalid(posx) || !isvalid(posy) || !isvalid(vel) )
					return false;
				
				PosSensorHealthInf2 sensor_inf;
				if( get_PosSensorHealth_XY( &sensor_inf, slamSensorInd ) && sensor_inf.slam_sensor )
				{
					double x, y;
					double yaw = sensor_inf.mp.lat0_rad + sensor_inf.mp.lon0_rad;
					double sin_Yaw, cos_Yaw;
					fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
					x = BodyHeading2ENU_x( posx , posy , sin_Yaw , cos_Yaw );
					y = BodyHeading2ENU_y( posx , posy , sin_Yaw , cos_Yaw );
					x -= sensor_inf.HOffset.x;
					y -= sensor_inf.HOffset.y;
					return Position_Control_set_TargetPositionXY( x, y, vel, prestore, prestored, TIMEOUT );
				}
				else	//无传感器信息 或 不是slam传感器
					return false;
			}
			bool Position_Control_set_TargetPositionXYZ_LatLon( double Lat, double Lon, double posz, double vel, MissionPrestoreMode prestore, bool* prestored, double TIMEOUT )
			{
				if( !isvalid(Lat) || !isvalid(Lon) || !isvalid(posz) || !isvalid(vel) )
					return false;
				
				//获取最优全球定位传感器信息
				PosSensorHealthInf3 global_inf;
				if( get_OptimalGlobal_XYZ( &global_inf ) == false )
					return false;
				//获取指定经纬度平面坐标
				double x, y;
				map_projection_project( &global_inf.mp, Lat, Lon, &x, &y );
				x -= global_inf.HOffset.x;
				y -= global_inf.HOffset.y;
				posz -= global_inf.HOffset.z;
				return Position_Control_set_TargetPositionXYZ( x, y, posz, vel, prestore, prestored, TIMEOUT );
			}
			bool Position_Control_set_TargetPositionXYZ_SLAM( uint8_t slamSensorInd, double posx, double posy, double posz, double vel, MissionPrestoreMode prestore, bool* prestored, double TIMEOUT )
			{
				if( !isvalid(posx) || !isvalid(posy) || !isvalid(posz) || !isvalid(vel) )
					return false;
				
				PosSensorHealthInf3 sensor_inf;
				if( get_PosSensorHealth_XYZ( &sensor_inf, slamSensorInd ) && sensor_inf.slam_sensor )
				{
					double x, y;
					double yaw = sensor_inf.mp.lat0_rad + sensor_inf.mp.lon0_rad;
					double sin_Yaw, cos_Yaw;
					fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
					x = BodyHeading2ENU_x( posx , posy , sin_Yaw , cos_Yaw );
					y = BodyHeading2ENU_y( posx , posy , sin_Yaw , cos_Yaw );
					x -= sensor_inf.HOffset.x;
					y -= sensor_inf.HOffset.y;
					posz -= sensor_inf.HOffset.z;
					return Position_Control_set_TargetPositionXYZ( x, y, posz, vel, prestore, prestored, TIMEOUT );
				}
				else	//无传感器信息 或 不是slam传感器
					return false;
			}
			bool Position_Control_set_TargetPositionXYZRA_LatLon( double Lat, double Lon, double posz, double vel, MissionPrestoreMode prestore, bool* prestored, double TIMEOUT )
			{
				if( !isvalid(Lat) || !isvalid(Lon) || !isvalid(posz) || !isvalid(vel) )
					return false;
				
				//获取最优全球定位传感器信息
				PosSensorHealthInf2 global_inf;
				if( get_OptimalGlobal_XY( &global_inf ) == false )
					return false;
				//获取指定经纬度平面坐标
				double x, y;
				map_projection_project( &global_inf.mp, Lat, Lon, &x, &y );
				x -= global_inf.HOffset.x;
				y -= global_inf.HOffset.y;
				//获取起飞位置Z坐标
				double homeZ;
				getHomeLocalZ(&homeZ);
				return Position_Control_set_TargetPositionXYZ( x, y, homeZ + posz, vel, prestore, prestored, TIMEOUT );
			}
			bool Position_Control_set_TargetPositionXYZRelative_LatLon( double Lat, double Lon, double posz, double vel, MissionPrestoreMode prestore, bool* prestored, double TIMEOUT )
			{
				if( !isvalid(Lat) || !isvalid(Lon) || !isvalid(posz) || !isvalid(vel) )
					return false;
				
				//获取最优全球定位传感器信息
				PosSensorHealthInf2 global_inf;
				if( get_OptimalGlobal_XY( &global_inf ) == false )
					return false;
				//获取指定经纬度平面坐标
				double x, y, z;
				map_projection_project( &global_inf.mp, Lat, Lon, &x, &y );
				x -= global_inf.HOffset.x;
				y -= global_inf.HOffset.y;
				vector3<double> position;
				if( HorizontalPosition_ControlMode==Position_ControlMode_Position )
					position = target_position;
				else
					get_Position_Ctrl(&position);
				z = position.z + posz;
				return Position_Control_set_TargetPositionXYZ( x, y, z, vel, prestore, prestored, TIMEOUT );
			}
			
			bool Position_Control_get_LineFlightDistance( double* distance, double TIMEOUT )
			{
				if( distance == 0 )
					return false;
				if( HorizontalPosition_ControlMode!=Position_ControlMode_RouteLine3D && HorizontalPosition_ControlMode!=Position_ControlMode_RouteLine )
					return false;
				
				vector3<double> position;
				get_Position_Ctrl(&position);
				if( LockCtrl(TIMEOUT) )
				{
					//计算偏差
					vector3<double> B = target_position + route_line_A_B;
					vector3<double> B_C = position - B;
					if( route_line_A_B.get_square() > 0.1 )
					{
						vector3<double> route_n = route_line_A_B * (-1.0/safe_sqrt(route_line_A_B.get_square()));
						*distance = route_n * B_C;
					}
					else
						*distance = safe_sqrt( B_C.get_square() );
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			bool Position_Control_get_LineFlightABDistance( vector3<double>* AB, double* distance, double TIMEOUT )
 			{
				if( AB==0 && distance==0 )
					return false;
				if( HorizontalPosition_ControlMode!=Position_ControlMode_RouteLine3D && HorizontalPosition_ControlMode!=Position_ControlMode_RouteLine )
					return false;
				
				vector3<double> position;
				get_Position_Ctrl(&position);
				if( LockCtrl(TIMEOUT) )
				{
					if( AB != 0 )
						*AB = route_line_A_B;
					if( distance != 0 )
					{
						//计算垂足
						vector3<double> A_C = position - target_position;
						double k = (A_C * route_line_A_B) * route_line_m;
						vector3<double> foot_point = (route_line_A_B * k) + target_position;
						
						//计算偏差
						vector3<double> B = target_position + route_line_A_B;
						vector3<double> dis = foot_point - B;
						*distance = safe_sqrt(dis.get_square());
					}
					UnlockCtrl();
					return true;
				}
				return false;
			}
		/*直线飞行*/
			
		/*设定自动飞行速度*/
			bool Position_Control_get_XYAutoSpeed( double* AtVelXY, double TIMEOUT )
			{
				if( LockCtrl(TIMEOUT) )
				{
					*AtVelXY = AutoVelXY;
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			bool Position_Control_reset_XYAutoSpeed( double TIMEOUT )
			{
				if( LockCtrl(TIMEOUT) )
				{
					AutoVelXY = cfg.AutoVXY[0];
					if( AutoVelXY > cfg.AutoVXY[0] )
						AutoVelXY = cfg.AutoVXY[0];
					line_track_desired_maxv = AutoVelXY;
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			bool Position_Control_set_XYAutoSpeed( double AtVelXY, double TIMEOUT )
			{
				if( !isvalid(AtVelXY) )
					return false;
				
				if( LockCtrl(TIMEOUT) )
				{
					if( AtVelXY > 5 )
						AutoVelXY = AtVelXY;
					else if( AtVelXY < 0 )
						AutoVelXY = cfg.AutoVXY[0];
					
					if( AutoVelXY > cfg.maxAutoVelXY[0] )
						AutoVelXY = cfg.maxAutoVelXY[0];
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			
			bool Position_Control_get_XYZAutoSpeed( double* AtVelXYZ, double TIMEOUT )
			{
				if( LockCtrl(TIMEOUT) )
				{
					*AtVelXYZ = AutoVelXYZ;
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			bool Position_Control_reset_XYZAutoSpeed( double TIMEOUT )
			{		
				if( LockCtrl(TIMEOUT) )
				{
					AutoVelXYZ = cfg.AutoVXYZ[0];
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			bool Position_Control_set_XYZAutoSpeed( double AtVelXYZ, double TIMEOUT )
			{
				if( !isvalid(AtVelXYZ) )
					return false;
				
				if( LockCtrl(TIMEOUT) )
				{
					if( AtVelXYZ > 5 )
						AutoVelXYZ = AtVelXYZ;
					else if( AtVelXYZ < 0 )
						AutoVelXYZ = cfg.AutoVXYZ[0];
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
		/*设定自动飞行速度*/
	/*水平位置*/
			
	/*航线预存*/
		//航线结束方式
		enum MissionEndMode
		{
			MissionEndMode_null = 0,
			//任务结束-圆形
			MissionEndMode_Arc,
		};
		static MissionEndMode missionEndMode = MissionEndMode_null;
		static double MissionEnd_TotalRoute = 0;
		static double MissionEnd_advanceS = 0;
		static vector3<double> MissionEnd_P1;
		static vector3<double> MissionEnd_P2;
		static double MissionEnd_v1 = 0;
		static double MissionEnd_v2 = 0;
		static inline void resetMissionEndMode()
		{
			missionEndMode = MissionEndMode_null;
		}
		
		static inline bool setMissionEndArc( double startSpeed, double endSpeed, double advanceS, vector3<double> circleO, vector3<double> angle )
		{
			if( !isvalid(endSpeed) || 
				!isvalid(circleO.x) || !isvalid(circleO.y) || !isvalid(circleO.z) ||
				!isvalid(angle.x) || !isvalid(angle.y) || !isvalid(angle.z) )
				return false;
			
			MissionEnd_P1 = circleO;
			MissionEnd_P2 = angle;
			MissionEnd_v1 = startSpeed;
			MissionEnd_v2 = endSpeed;
			MissionEnd_advanceS = advanceS;
			missionEndMode = MissionEndMode_Arc;
			
			return true;
		}
		
		//预存航线信息
		enum PrestoreMission
		{
			PrestoreMission_null = 0,
			//直线
			PrestoreMission_Line,
		};
		static PrestoreMission prestoreMission = PrestoreMission_null;
		static vector3<double> prestoreMission_P1;
		static void reset_prestoreMission()
		{
			prestoreMission = PrestoreMission_null;
			resetMissionEndMode();
		}
		bool get_prestoreMissionState()
		{
			return prestoreMission != PrestoreMission_null;
		}
		//预存三维直线航线
		static bool prestoreMissionLine( vector3<double> pD )
		{
			if( TRadius <= 0 )
			{
				reset_prestoreMission();
				return false;
			}
			
			switch(HorizontalPosition_ControlMode)
			{
				case Position_ControlMode_RouteLine3D:
				{	//当前为航线
					break;
				}
				
				default:
					reset_prestoreMission();
					return false;
			}
			
			//求转弯角sin cos
			vector3<double> CO = route_line_A_B;
			vector3<double> pC = target_position;
			vector3<double> CD = pD - pC;
			
			//求夹角OCD
			double sinAngle, cosAngle;
			vector3<double> angleOCD = vector3<double>::get_included_angle( CO, CD );
			double angleOCD_length = safe_sqrt(angleOCD.get_square());
			fast_sin_cos( angleOCD_length, &sinAngle, &cosAngle );
			
			//求向量OC CD长度
			double OClength = safe_sqrt(CO.get_square());
			double CDlength = safe_sqrt(CD.get_square());
			if( !is_zero(1+cosAngle) && !is_zero(OClength) && !is_zero(CDlength) && !is_zero(angleOCD_length) )
			{	//转弯角不为直线
				double Cr = TRadius;
				//Cr不能大于当前航线长度
				if( sq(Cr) > CO.get_square() )
					Cr = safe_sqrt( CO.get_square() );
				//Cr不能大于下一条航线长度的一半
				if( sq(Cr) > CD.get_square()*0.25 )
					Cr = safe_sqrt( CD.get_square() ) * 0.5;
				double r = Cr*sinAngle/(1+cosAngle);
				
				//根据Cr和r求圆心坐标
				double invOClength = 1.0 / OClength;
				double invCDlength = 1.0 / CDlength;
				vector3<double> nCO = CO * invOClength;
				vector3<double> nCD = CD * invCDlength;
				vector3<double> CG = nCO + nCD;
				double CGlength = safe_sqrt(CG.get_square());
				if(!is_zero(CGlength))
				{
					//求向量CG
					CG *= safe_sqrt(sq(Cr)+sq(r)) * (1.0/CGlength);
					//获得圆心坐标
					vector3<double> circleO = pC + CG;
					
					//计算协调转弯速度
					double circleVel = safe_sqrt(cfg.maxAutoAccXY[0] * r);
					//速度限幅
					if( circleVel > AutoVelXYZ )
						circleVel = AutoVelXYZ;
					double maxCDVel = safe_sqrt(cfg.maxAutoAccXY[0]*CDlength);
					if( circleVel > maxCDVel )
						circleVel = maxCDVel;
					//OC速度限幅
					double xy_length = safe_sqrt( sq(CO.x) + sq(CO.y) );
					double xy_scale = xy_length * invOClength;
					double z_scale = fabs(CO.z) * invOClength;
					if( circleVel*xy_scale > cfg.maxAutoVelXY[0] )
						circleVel = cfg.maxAutoVelXY[0] / xy_scale;
					double z_vel = circleVel*z_scale;
					if( CO.z < 0 )
					{	//向上飞
						if( z_vel > cfg.maxAutoVelUp[0] )
							circleVel *= cfg.maxAutoVelUp[0] / z_vel;
					}
					else
					{	//向下飞
						if( z_vel > cfg.maxAutoVelDown[0] )
							circleVel *= cfg.maxAutoVelDown[0] / z_vel;
					}
					//CD速度限幅
					xy_length = safe_sqrt( sq(CD.x) + sq(CD.y) );
					xy_scale = xy_length * invCDlength;
					z_scale = fabs(CD.z) * invCDlength;
					if( circleVel*xy_scale > cfg.maxAutoVelXY[0] )
						circleVel = cfg.maxAutoVelXY[0] / xy_scale;
					z_vel = circleVel*z_scale;
					if( CD.z > 0 )
					{	//向上飞
						if( z_vel > cfg.maxAutoVelUp[0] )
							circleVel *= cfg.maxAutoVelUp[0] / z_vel;
					}
					else
					{	//向下飞
						if( z_vel > cfg.maxAutoVelDown[0] )
							circleVel *= cfg.maxAutoVelDown[0] / z_vel;
					}
					//计算转角
					vector3<double> rAngle = angleOCD * ( -(Pi-angleOCD_length) / angleOCD_length );
					
					//设置协调转弯
					prestoreMission = PrestoreMission_Line;
					prestoreMission_P1 = pD;
					setMissionEndArc( circleVel, circleVel, Cr, circleO, rAngle );
				}
				else
					reset_prestoreMission();
			}
			else
			{	//转角为直线
				reset_prestoreMission();
			}
			return prestoreMission!=PrestoreMission_null;
		}
		//预存二维直线航线
		static bool prestoreMissionLine( vector2<double> pD )
		{
			if( TRadius <= 0 )
			{
				reset_prestoreMission();
				return false;
			}
			
			switch(HorizontalPosition_ControlMode)
			{
				case Position_ControlMode_RouteLine:
				{	//当前为航线
					break;
				}
				
				default:
					reset_prestoreMission();
					return false;
			}
			
			//求转弯角sin cos
			vector3<double> CO = route_line_A_B;	CO.z = 0;
			vector3<double> pC = target_position;	pC.z = 0;
			vector3<double> CD = vector3<double>(pD.x, pD.y, 0) - pC;
			
			//求夹角OCD
			double sinAngle, cosAngle;
			vector3<double> angleOCD = vector3<double>::get_included_angle( CO, CD );
			double angleOCD_length = safe_sqrt(angleOCD.get_square());
			fast_sin_cos( angleOCD_length, &sinAngle, &cosAngle );
			
			//求向量OC CD长度
			double OClength = safe_sqrt(CO.get_square());
			double CDlength = safe_sqrt(CD.get_square());
			if( !is_zero(1+cosAngle) && !is_zero(OClength) && !is_zero(CDlength) && !is_zero(angleOCD_length) )
			{	//转弯角不为直线
				double Cr = TRadius;
				//Cr不能大于当前航线长度
				if( sq(Cr) > CO.get_square() )
					Cr = safe_sqrt( CO.get_square() );
				//Cr不能大于下一条航线长度的一半
				if( sq(Cr) > CD.get_square()*0.25 )
					Cr = safe_sqrt( CD.get_square() ) * 0.5;
				double r = Cr*sinAngle/(1+cosAngle);
				
				//根据Cr和r求圆心坐标
				double invOClength = 1.0 / OClength;
				double invCDlength = 1.0 / CDlength;
				vector3<double> nCO = CO * invOClength;
				vector3<double> nCD = CD * invCDlength;
				vector3<double> CG = nCO + nCD;
				double CGlength = safe_sqrt(CG.get_square());
				if(!is_zero(CGlength))
				{
					//求向量CG
					CG *= safe_sqrt(sq(Cr)+sq(r)) * (1.0/CGlength);
					//获得圆心坐标
					vector3<double> circleO = pC + CG;
					
					//计算协调转弯速度
					double circleVel = safe_sqrt(cfg.maxAutoAccXY[0] * r);
					//速度限幅
					if( circleVel > AutoVelXY )
						circleVel = AutoVelXY;
					double maxCDVel = safe_sqrt(cfg.maxAutoAccXY[0]*CDlength);
					if( circleVel > maxCDVel )
						circleVel = maxCDVel;
					//计算转角
					vector3<double> rAngle = angleOCD * ( -(Pi-angleOCD_length) / angleOCD_length );
					
					//设置协调转弯
					prestoreMission = PrestoreMission_Line;
					prestoreMission_P1 = vector3<double>(pD.x, pD.y, 0);
					setMissionEndArc( circleVel, circleVel, Cr, circleO, rAngle );
				}
				else
					reset_prestoreMission();
			}
			else
			{	//转角为直线
				reset_prestoreMission();
			}
			return prestoreMission!=PrestoreMission_null;
		}
	/*航线预存*/
			
/*控制接口*/
		
/*滤波器*/
	static Filter_Butter4_LP ThrOut_Filters[3];
/*滤波器*/
			
#define DESIRED_VEL_CTRL_DEC 250
			
void ctrl_Position()
{	
	bool Attitude_Control_Enabled;	is_Attitude_Control_Enabled(&Attitude_Control_Enabled);
	if( Attitude_Control_Enabled == false )
	{
		Altitude_Control_Enabled = false;
		Position_Control_Enabled = false;
		return;
	}
	
	double h = 1.0/CtrlRateHz;
	
	double e_1_n;
	double e_1;
	double e_2_n;
	double e_2;
	
	bool inFlight;	get_is_inFlight(&inFlight);
	vector3<double> Position;	get_Position_Ctrl(&Position);
	vector3<double> VelocityENU;	get_VelocityENU_Ctrl(&VelocityENU);
	vector3<double> AccelerationENU;	get_AccelerationENU_Ctrl(&AccelerationENU);
	double AccelerationENU_ESO_Z;
	get_es_AccZ(&AccelerationENU_ESO_Z);
	
	//位置速度滤波
	double Ps = cfg.P1[0];
	double Pv = cfg.P2[0];
	double Pa = cfg.P3[0];
	#define T21P 0.5
	
	static vector3<double> TAcc;
	vector3<double> TargetVelocity;
	vector3<double> TargetVelocity_1;
	vector3<double> TargetVelocity_2;
	
	//XY或Z其中一个为非3D模式则退出3D模式
	if( Is_3DAutoMode(HorizontalPosition_ControlMode) && Is_3DAutoMode(Altitude_ControlMode)==false )
		HorizontalPosition_ControlMode = Position_ControlMode_Locking;
	else if( Is_3DAutoMode(HorizontalPosition_ControlMode)==false && Is_3DAutoMode(Altitude_ControlMode) )
		Altitude_ControlMode = Position_ControlMode_Locking;
	
	vector3<double> posErr;
	
	if( Position_Control_Enabled )
	{	//水平位置控制
		if( get_Position_MSStatus() != MS_Ready )
		{
			Position_Control_Enabled = false;
			goto PosCtrl_Finish;
		}
		
		switch( HorizontalPosition_ControlMode )
		{
			case Position_ControlMode_ManualCircle:
			{	//手动绕圈模式
				if( inFlight )
				{
					//计算半径方向偏差
					vector2<double> RVec = vector2<double>(Position.x,Position.y) - ManualCircleOrigin;
					double RVec_length = safe_sqrt(RVec.get_square());					
					vector2<double> RVec_normed;
					if( RVec_length > 0.1 )
						RVec_normed = RVec * (1.0/RVec_length);
					double RErrVec_lengthErr = ManualCircleR - RVec_length;
					vector2<double> RErrVec = RVec_normed * RErrVec_lengthErr;
					
					//计算e1导数
					vector2<double> e1_1( VelocityENU.x, VelocityENU.y );
					double e1r_1 = - (e1_1 * RVec_normed);
					vector2<double> e1_2( TAcc.x, TAcc.y );
					double e1r_2 = - (e1_2 * RVec_normed);
					vector2<double> circleVel = e1_1 + RVec_normed*e1r_1;
					/*半径方向*/
						vector3<double> esAngularRate;
						get_EsAngularRate(&esAngularRate);
						smooth_kp_d2 d1r = smooth_kp_2( RErrVec_lengthErr, e1r_1, e1r_2 , Ps, AutoVelXY+100 );
						vector2<double> T2r = RVec_normed*d1r.d0;
						double CircleAcc = 0;
						if( RVec_length > 0.1 )
							CircleAcc = circleVel.get_square() * (1.0/RVec_length);
						vector2<double> T2r_1 = RVec_normed*d1r.d1 - RVec_normed*CircleAcc;
						vector2<double> T2r_2 = RVec_normed*d1r.d2;
					/*半径方向*/
						
					//计算绕圆周速度
					vector2<double> TargetCircleVel(-RVec_normed.y,RVec_normed.x);
					TargetCircleVel *= ManualCircleVel;
						
					//控制偏航
					Quaternion quat;
					get_AirframeY_quat(&quat);
					double currentYaw = quat.getYaw();
					double yaw_err = atan2(-RVec.y,-RVec.x) - currentYaw;
					yaw_err = Mod( yaw_err, 2*Pi );
					if(yaw_err > Pi)
						yaw_err -= 2*Pi;
					while(yaw_err < -Pi)
						yaw_err += 2*Pi;
					double target_yaw_rate = 0;
					if( RVec_length > 0.1 )
						target_yaw_rate = (RVec_normed.x*VelocityENU.y - RVec_normed.y*VelocityENU.x) * (1.0/RVec_length);
					Attitude_Control_set_Target_YawRate( yaw_err*1.0 + target_yaw_rate );
						
					TargetVelocity.x = T2r.x+TargetCircleVel.x;	TargetVelocity.y = T2r.y+TargetCircleVel.y;
					TargetVelocity_1.x = T2r_1.x;	TargetVelocity_1.y = T2r_1.y;
					TargetVelocity_2.x = T2r_2.x;	TargetVelocity_2.y = T2r_2.y;
				}
				else
				{	//没起飞前在绕圈模式
					//回到position模式
					HorizontalPosition_ControlMode = Position_ControlMode_Position;
				}
				break;
			}
			
			case Position_ControlMode_Position:
			{	//悬停模式
				if( inFlight )
				{
					vector2<double> e1;
					e1.x = target_position.x - Position.x;
					e1.y = target_position.y - Position.y;
					vector2<double> e1_1;
					e1_1.x = - VelocityENU.x;
					e1_1.y = - VelocityENU.y;
					vector2<double> e1_2;
					e1_2.x = - TAcc.x;
					e1_2.y = - TAcc.y;
					double e1_length = safe_sqrt(e1.get_square());
					e_1_n = e1.x*e1_1.x + e1.y*e1_1.y;
					if( !is_zero(e1_length) )
						e_1 = e_1_n / e1_length;
					else
						e_1 = 0;
					e_2_n = ( e1.x*e1_2.x + e1.y*e1_2.y + e1_1.x*e1_1.x + e1_1.y*e1_1.y )*e1_length - e_1*e_1_n;
					if( !is_zero(e1_length*e1_length) )
						e_2 = e_2_n / (e1_length*e1_length);
					else
						e_2 = 0;
					
					double _pos_vel = 200;
					if( e1_length < 100 )
						pos_vel = -1;
					if( pos_vel > 0 )
						_pos_vel = pos_vel;					
					smooth_kp_d2 d1 = smooth_kp_2( e1_length, e_1, e_2 , Ps, _pos_vel );
					vector2<double> T2;
					vector2<double> T2_1;
					vector2<double> T2_2;
					if( !is_zero(e1_length*e1_length*e1_length) )
					{
						vector2<double> n = e1 * (1.0/e1_length);
						vector2<double> n_1 = (e1_1*e1_length - e1*e_1) / (e1_length*e1_length);
						vector2<double> n_2 = ( (e1_2*e1_length-e1*e_2)*e1_length - (e1_1*e1_length-e1*e_1)*(2*e_1) ) / (e1_length*e1_length*e1_length);
						T2 = n*d1.d0;
						T2_1 = n*d1.d1 + n_1*d1.d0;
						T2_2 = n*d1.d2 + n_1*(2*d1.d1) + n_2*d1.d0;
					}
					TargetVelocity.x = T2.x;	TargetVelocity.y = T2.y;
					TargetVelocity_1.x = T21P*T2_1.x;	TargetVelocity_1.y = T21P*T2_1.y;
					TargetVelocity_2.x = T2_2.x;	TargetVelocity_2.y = T2_2.y;
				}
				else
				{	//没起飞前在位置控制模式
					//重置期望位置
					target_position.x = Position.x;
					target_position.y = Position.y;
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					goto PosCtrl_Finish;
				}
				break;
			}		
			case Position_ControlMode_Velocity:
			{	//速度控制模式
				if( !inFlight )
				{
					//没起飞时重置期望速度
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					goto PosCtrl_Finish;
				}
				else
				{
					TargetVelocity.x = target_velocity.x;
					TargetVelocity.y = target_velocity.y;
					if( getAttCtrlCfg()->Mode[0] & ATTCTRL_MODE_TURNCOMP_ENA_BIT )
					{
						double w;
						Attitude_Control_get_YawTrackVel(&w);
						if( sq(TargetVelocity.x)+sq(TargetVelocity.y) > sq(VelocityENU.x)+sq(VelocityENU.y) )
						{
							TargetVelocity_1.x = -VelocityENU.y*w;
							TargetVelocity_1.y =  VelocityENU.x*w;
						}
						else
						{
							TargetVelocity_1.x = -TargetVelocity.y*w;
							TargetVelocity_1.y =  TargetVelocity.x*w;
						}
					}
					Pv = cfg.P2_VelXY[0];
				}
				break;
			}
			
			case Position_ControlMode_OffBoard:
			{	//OffBoard模式
				if( inFlight )
				{
					if( OffBoardXY_target_mode == 3 )
					{	//控制位置+速度+加速度
						vector2<double> e1;
						e1.x = target_position.x - Position.x;
						e1.y = target_position.y - Position.y;
						vector2<double> e1_1;
						e1_1.x = target_velocity.x - VelocityENU.x;
						e1_1.y = target_velocity.y - VelocityENU.y;
						vector2<double> e1_2;
						e1_2.x = target_acc.x - TAcc.x;
						e1_2.y = target_acc.y - TAcc.y;
						double e1_length = safe_sqrt(e1.get_square());
						e_1_n = e1.x*e1_1.x + e1.y*e1_1.y;
						if( !is_zero(e1_length) )
							e_1 = e_1_n / e1_length;
						else
							e_1 = 0;
						e_2_n = ( e1.x*e1_2.x + e1.y*e1_2.y + e1_1.x*e1_1.x + e1_1.y*e1_1.y )*e1_length - e_1*e_1_n;
						if( !is_zero(e1_length*e1_length) )
							e_2 = e_2_n / (e1_length*e1_length);
						else
							e_2 = 0;
						
						//记录位置误差
						posErr.x = e1.x;
						posErr.y = e1.y;
						
						double _pos_vel = cfg.maxVelXY[0];
//						if( e1_length < 100 )
//							pos_vel = -1;
//						if( pos_vel > 0 )
//							_pos_vel = pos_vel;					
						smooth_kp_d2 d1 = smooth_kp_2( e1_length, e_1, e_2 , Ps, _pos_vel );
						vector2<double> T2;
						vector2<double> T2_1;
						vector2<double> T2_2;
						if( !is_zero(e1_length*e1_length*e1_length) )
						{
							vector2<double> n = e1 * (1.0/e1_length);
							vector2<double> n_1 = (e1_1*e1_length - e1*e_1) / (e1_length*e1_length);
							vector2<double> n_2 = ( (e1_2*e1_length-e1*e_2)*e1_length - (e1_1*e1_length-e1*e_1)*(2*e_1) ) / (e1_length*e1_length*e1_length);
							T2 = n*d1.d0;
							T2_1 = n*d1.d1 + n_1*d1.d0;
							T2_2 = n*d1.d2 + n_1*(2*d1.d1) + n_2*d1.d0;
						}
						TargetVelocity.x = T2.x + target_velocity.x;	TargetVelocity.y = T2.y + target_velocity.y;
						TargetVelocity_1.x = T21P*T2_1.x + target_acc.x;	TargetVelocity_1.y = T21P*T2_1.y + target_acc.y;
						TargetVelocity_2.x = T2_2.x;	TargetVelocity_2.y = T2_2.y;
					}
					else
					{	//速度+加速度
						TargetVelocity.x = target_velocity.x;	TargetVelocity.y = target_velocity.y;
						TargetVelocity_1.x = T21P*target_acc.x;	TargetVelocity_1.y = T21P*target_acc.y;
					}
				}
				else
				{	//没起飞前在位置控制模式
					//重置期望位置
					target_position.x = Position.x;
					target_position.y = Position.y;
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					goto PosCtrl_Finish;
				}
				break;
			}
			case Position_ControlMode_OffBoard3D:
			{	//OffBoard模式
				if( inFlight )
				{
					if( OffBoardXY_target_mode == 3 )
					{	//控制位置+速度+加速度
						vector3<double> e1 = target_position - Position;
						vector3<double> e1_1 = target_velocity - VelocityENU;
						//vector2<double> e1_2 = target_acc - TAcc;
						double e1_length = safe_sqrt(e1.get_square());
						e_1_n = e1.x*e1_1.x + e1.y*e1_1.y + e1.z*e1_1.z;
						if( !is_zero(e1_length) )
							e_1 = e_1_n / e1_length;
						else
							e_1 = 0;
						
						//记录位置误差
						posErr = e1;
						
						double _pos_vel = cfg.maxVelXY[0];
//						if( e1_length < 100 )
//							pos_vel = -1;
//						if( pos_vel > 0 )
//							_pos_vel = pos_vel;					
						smooth_kp_d2 d1 = smooth_kp_2( e1_length, e_1, e_2 , Ps, _pos_vel );
						vector3<double> T2;
						vector3<double> T2_1;
						vector3<double> T2_2;
						if( !is_zero(e1_length*e1_length*e1_length) )
						{
							vector3<double> n = e1 * (1.0/e1_length);
							vector3<double> n_1 = (e1_1*e1_length - e1*e_1) / (e1_length*e1_length);
							T2 = n*d1.d0;
							T2_1 = n*d1.d1 + n_1*d1.d0;
						}
						TargetVelocity = T2 + target_velocity;
						TargetVelocity_1 = T2_1*T21P + target_acc;
					}
					else
					{	//速度+加速度
						TargetVelocity.x = target_velocity.x;	TargetVelocity.y = target_velocity.y;
						TargetVelocity_1.x = T21P*target_acc.x;	TargetVelocity_1.y = T21P*target_acc.y;
					}
				}
				else
				{	//没起飞前在位置控制模式
					//重置期望位置
					target_position.x = Position.x;
					target_position.y = Position.y;
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					goto PosCtrl_Finish;
				}
				break;
			}
			
			case Position_ControlMode_RouteLine:
			{
				if( inFlight )
				{
					//速度限幅
					if( AutoVelXY > cfg.maxAutoVelXY[0] )
						AutoVelXY = cfg.maxAutoVelXY[0];
					line_track_desired_maxv = AutoVelXY;
					
					//计算垂足	
					vector2<double> A( target_position.x, target_position.y );					
					vector2<double> C( Position.x, Position.y );
					vector2<double> A_C = C - A;
					vector2<double> A_B( route_line_A_B.x, route_line_A_B.y );
					double k = (A_C * A_B) * route_line_m;
					vector2<double> foot_point = (A_B * k) + A;
					
					//是否进行MissionEnd动作
					bool needMissionEnd = false;
					double stopVel = 0;
					double advanceS = 0;
					if( missionEndMode )
					{
						needMissionEnd = true;
						stopVel = MissionEnd_v1;
						if( stopVel > line_track_desired_maxv )
							stopVel = line_track_desired_maxv;
						advanceS = MissionEnd_advanceS;
					}
					
					//计算跟踪A点位置
					vector2<double> A_desired = A;
					double A_B_length = safe_sqrt(A_B.get_square());		
					double line_track_desired_acc = 0;	
					if( A_B_length > 0.1 )
					{
						double stopVelDiff = line_track_desired_vel - stopVel;
						if( stopVelDiff < 0 )
							stopVelDiff = 0;
						if( ( (0.5*sq(stopVelDiff) + stopVel*stopVelDiff)/line_track_desired_maxacc >= A_B_length - line_track_desired - advanceS ) ||
								( max_line_track_desired>0 && 0.5*sq(line_track_desired_vel)/line_track_desired_maxacc>=max_line_track_desired-line_track_desired )
							)
						{	//需要减速
							line_track_desired_vel -= line_track_desired_maxacc*h;
							if( line_track_desired_vel < 0 )
								line_track_desired_vel = 0;
							else
								line_track_desired_acc = -line_track_desired_maxacc;
						}
						else
						{	//前馈加速到最高速度
							double max_v = line_track_desired_maxv;
							if( line_track_desired_vel < max_v )
								line_track_desired_vel += line_track_desired_maxacc*h;
							if( line_track_desired_vel > max_v )
								line_track_desired_vel = max_v;
							else
								line_track_desired_acc = +line_track_desired_maxacc;
						}
						//根据障碍限制目标速度
						if( max_line_track_desired_vel>0 && line_track_desired_vel>max_line_track_desired_vel )
							line_track_desired_vel = max_line_track_desired_vel;
						
						vector2<double> B = A + A_B;
						A_desired = B - A_B*(line_track_desired/A_B_length);
						
						frontDir = atan2( -route_line_A_B.y, -route_line_A_B.x );
						frontVec.set_vector( -route_line_A_B.x, -route_line_A_B.y, 0 );
					}
					else
					{
						frontDir = -10;
						frontVec.zero();
					}
					
					//计算偏差
					vector2<double> e1r = A_desired - foot_point;
					vector2<double> e1d = foot_point - C;
					double e1r_length = safe_sqrt(e1r.get_square());
					double e1d_length = safe_sqrt(e1d.get_square());
					
					//计算route方向单位向量
					vector2<double> route_n;
					if( e1r_length > 0.001 )
						route_n = e1r * (1.0/e1r_length);
					
					//计算d方向单位向量
					vector2<double> d_n;
					if( e1d_length > 0.001 )
						d_n = e1d * (1.0/e1d_length);
					
					double route_sign = 1.0;
					if( route_n*vector2<double>(route_line_A_B.x,route_line_A_B.y) > 0 )
						route_sign = -1.0;
					
					//移动目标位置
					if( e1r_length < 10*100 )
					{
						line_track_desired += line_track_desired_vel*h;
						if( line_track_desired > A_B_length )
							line_track_desired = A_B_length;
						//根据障碍限制目标位置
						if( max_line_track_desired>0 && line_track_desired>max_line_track_desired )
							line_track_desired = max_line_track_desired;
					}
					
					//计算e1导数
					vector2<double> e1_1( VelocityENU.x, VelocityENU.y );
					double e1r_1 = route_sign*line_track_desired_vel - (e1_1 * route_n);
					double e1d_1 = -(e1_1 * d_n);
					//e1二阶导
					vector2<double> e1_2( TAcc.x, TAcc.y );
					double e1r_2 = route_sign*line_track_desired_acc - (e1_2 * route_n);
					double e1d_2 = -(e1_2 * d_n);
					
					pos_vel = cfg.maxAutoVelXY[0];
					/*route方向*/
						double desired_vel_ctrl = line_track_desired_vel - DESIRED_VEL_CTRL_DEC;
						if( desired_vel_ctrl < 0 )
							desired_vel_ctrl = 0;
					
						smooth_kp_d2 d1r = smooth_kp_2( e1r_length, e1r_1, e1r_2 , Ps, AutoVelXY+100 );
						vector2<double> T2r = route_n * ( d1r.d0 + desired_vel_ctrl );
						vector2<double> T2r_1 = route_n * d1r.d1;
						vector2<double> T2r_2 = route_n * d1r.d2;
					/*route方向*/
					
					/*d方向*/
						smooth_kp_d2 d1d = smooth_kp_2( e1d_length, e1d_1, e1d_2 , Ps, pos_vel );
						vector2<double> T2d = d_n * d1d.d0;
						vector2<double> T2d_1 = d_n * d1d.d1;
						vector2<double> T2d_2 = d_n * d1d.d2;
					/*d方向*/
						
					TargetVelocity.x = T2r.x+T2d.x;	TargetVelocity.y = T2r.y+T2d.y;
					TargetVelocity_1.x = T21P*T2r_1.x+T2d_1.x;	TargetVelocity_1.y = T21P*T2r_1.y+T2d_1.y;
					TargetVelocity_2.x = T2r_2.x+T2d_2.x;	TargetVelocity_2.y = T2r_2.y+T2d_2.y;
						
					//判断到点
					double wp_range = 0;
					if( wp_range < 0.1 )
						wp_range = 0.1;
					if( A_B_length - line_track_desired <= wp_range+advanceS ) 
					{
						if( !needMissionEnd )
						{
							//计算到点延时时间
							double wp_delay = 1;
							
							//到点延时
							++line_track_delay_counter;
							if( line_track_delay_counter >= wp_delay*CtrlRateHz )
								HorizontalPosition_ControlMode = Position_ControlMode_Position;
						}
						else
						{
							switch(missionEndMode)
							{
								case MissionEndMode_Arc:
								{	//圆弧结束
									target_position.x = A_desired.x;
									target_position.y = A_desired.y;
									vector3<double> routeDist = 
										MissionEnd_P2 % (vector3<double>(target_position.x, target_position.y, 0) - MissionEnd_P1);
									MissionEnd_TotalRoute = safe_sqrt(routeDist.get_square());
									line_track_desired = 0;
									HorizontalPosition_ControlMode = Position_ControlMode_MissionEndArc;
									break;
								}
								
								default:
								case MissionEndMode_null:
								{
									//计算到点延时时间
									double wp_delay = 1;
									
									//到点延时
									++line_track_delay_counter;
									if( line_track_delay_counter >= wp_delay*CtrlRateHz )
										HorizontalPosition_ControlMode = Position_ControlMode_Position;
									break;
								}
							}
						}
						
						
						
						//计算到点延时时间
						double wp_delay = 1;
						
						//到点延时
						++line_track_delay_counter;
						if( line_track_delay_counter >= wp_delay*CtrlRateHz )
							HorizontalPosition_ControlMode = Position_ControlMode_Position;
					}
					else
						line_track_delay_counter = 0;
				}
				else
				{
					//没起飞时重置期望速度
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					goto PosCtrl_Finish;
				}
				break;
			}
			case Position_ControlMode_MissionEndArc:
			{
				if( inFlight )
				{
					//求当前前进速度
					bool trackComplete = false;
					double trackPercent;
					double old_line_track_desired = line_track_desired;
					if( MissionEnd_TotalRoute > 0.01 )
					{
						//移动目标位置
						line_track_desired += line_track_desired_vel*h;
						trackPercent = line_track_desired / MissionEnd_TotalRoute;
						line_track_desired_vel = trackPercent * ( MissionEnd_v2 - MissionEnd_v1 ) + MissionEnd_v1;
						if( line_track_desired > MissionEnd_TotalRoute )
						{
							line_track_desired = MissionEnd_TotalRoute;
							trackComplete = true;
						}
					}
					else
					{
						line_track_desired_vel = MissionEnd_v2;
						line_track_desired = MissionEnd_TotalRoute;
						trackPercent = 1;
						trackComplete = true;
					}
					//求旋转后的位置
					Quaternion quat;
					quat.rotate_delta_angle(MissionEnd_P2*trackPercent);
					vector3<double> rotateP = quat.rotate(target_position - MissionEnd_P1);
					vector3<double> new_target_position = rotateP + MissionEnd_P1;
					
					//计算速度
					vector3<double> routeVel = MissionEnd_P2 % rotateP;
					double routeVelLength = safe_sqrt(routeVel.get_square());
					if( routeVelLength > 0.1 )
					{
						frontDir = atan2( routeVel.y, routeVel.x );
						frontVec = routeVel;
						
						double desired_vel_ctrl = line_track_desired_vel - DESIRED_VEL_CTRL_DEC;
						if( desired_vel_ctrl < 0 )
							desired_vel_ctrl = 0;
						routeVel *= desired_vel_ctrl * (1.0/routeVelLength);
					}
					else
					{
						routeVel.zero();
						frontDir = -10;
						frontVec.zero();
					}
					
					//进行位置控制
					vector2<double> e1;
					e1.x = new_target_position.x - Position.x;
					e1.y = new_target_position.y - Position.y;
					vector2<double> e1_1;
					e1_1.x = - VelocityENU.x;
					e1_1.y = - VelocityENU.y;
					vector2<double> e1_2;
					e1_2.x = - TAcc.x;
					e1_2.y = - TAcc.y;
					double e1_length = safe_sqrt(e1.get_square());
					e_1_n = e1.x*e1_1.x + e1.y*e1_1.y;
					if( !is_zero(e1_length) )
						e_1 = e_1_n / e1_length;
					else
						e_1 = 0;
					e_2_n = ( e1.x*e1_2.x + e1.y*e1_2.y + e1_1.x*e1_1.x + e1_1.y*e1_1.y )*e1_length - e_1*e_1_n;
					if( !is_zero(e1_length*e1_length) )
						e_2 = e_2_n / (e1_length*e1_length);
					else
						e_2 = 0;
					
					//误差过大恢复目标位置
					if( e1_length > 10*100 )
						line_track_desired = old_line_track_desired;
					
					double _pos_vel = 200;
					if( e1_length < 100 )
						pos_vel = -1;
					if( pos_vel > 0 )
						_pos_vel = pos_vel;					
					smooth_kp_d2 d1 = smooth_kp_2( e1_length, e_1, e_2 , Ps, _pos_vel );
					vector2<double> T2;
					vector2<double> T2_1;
					vector2<double> T2_2;
					if( !is_zero(e1_length*e1_length*e1_length) )
					{
						vector2<double> n = e1 * (1.0/e1_length);
						vector2<double> n_1 = (e1_1*e1_length - e1*e_1) / (e1_length*e1_length);
						T2 = n*d1.d0;
						T2_1 = n*d1.d1 + n_1*d1.d0;
					}
					TargetVelocity.x = T2.x + routeVel.x;
					TargetVelocity.y = T2.y + routeVel.y;
					TargetVelocity_1.x = T21P*T2_1.x;
					TargetVelocity_1.y = T21P*T2_1.y;
					
					//追踪完成
					if(trackComplete)
					{
						resetMissionEndMode();
						target_position = new_target_position;
						switch(prestoreMission)
						{
							case PrestoreMission_Line:
							{	//加载直线飞行
								vector3<double> position = target_position;
								target_position.x = prestoreMission_P1.x;
								target_position.y = prestoreMission_P1.y;
								
								//calculate vector B-A
								route_line_A_B = position - target_position;
								route_line_A_B.z = 0;
								double route_line_A_B_sq = route_line_A_B.get_square();
								if( route_line_A_B_sq > 0.1 )
								{
									//calculate 1/(B-A)^2
									route_line_m = 1.0 / route_line_A_B_sq;
									
									HorizontalPosition_ControlMode = Position_ControlMode_RouteLine;
									reset_line_track_state(AutoVelXY,cfg.maxAutoAccXY[0],MissionEnd_v2)
								}
								else
									HorizontalPosition_ControlMode = Position_ControlMode_Position;
							
								//复位航线避障
								max_line_track_desired = -1;
								max_line_track_desired_vel = -1;
								
								break;
							}
							
							default:
							{
								HorizontalPosition_ControlMode = Position_ControlMode_Position;
								break;
							}
						}
						reset_prestoreMission();
					}
				}
				else
				{	//没起飞前在位置控制模式
					//重置期望位置
					target_position.x = Position.x;
					target_position.y = Position.y;
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					goto PosCtrl_Finish;
				}
				break;
			}
			
			case Position_ControlMode_RouteLine3D:
			{
				if( inFlight )
				{
					//速度限幅
					double xyz_length = safe_sqrt( route_line_A_B.get_square() );
					double v_scale = 1.0;
					if( xyz_length > 1 )
					{
						double xy_length = safe_sqrt( route_line_A_B.x*route_line_A_B.x + route_line_A_B.y*route_line_A_B.y );
						
						double inv_xyz = 1.0 / xyz_length;
						double xy_scale = xy_length * inv_xyz;
						double z_scale = fabs(route_line_A_B.z) * inv_xyz;
						
						if( AutoVelXYZ*xy_scale > cfg.maxAutoVelXY[0] )
							v_scale = cfg.maxAutoVelXY[0] / (AutoVelXYZ*xy_scale);
						if( route_line_A_B.z < 0 )
						{	//向上飞
							double z_vel = AutoVelXYZ*z_scale;
							if( z_vel > cfg.maxAutoVelUp[0] )
							{
								double new_scale = cfg.maxAutoVelUp[0] / z_vel;
								if( new_scale < v_scale )
									v_scale = new_scale;
							}
						}
						else
						{	//向下飞
							double z_vel = AutoVelXYZ*z_scale;
							if( z_vel > cfg.maxAutoVelDown[0] )
							{
								double new_scale = cfg.maxAutoVelDown[0] / z_vel;
								if( new_scale < v_scale )
									v_scale = new_scale;
							}
						}
					}
					else
					{
						if( AutoVelXYZ > cfg.maxAutoVelXY[0] )
							v_scale = cfg.maxAutoVelXY[0] / AutoVelXYZ;
					}
					line_track_desired_maxv = AutoVelXYZ * v_scale;
					
					//计算垂足
					vector3<double> A_C = Position - target_position;
					double k = (A_C * route_line_A_B) * route_line_m;
					vector3<double> foot_point = (route_line_A_B * k) + target_position;
					
					//是否进行MissionEnd动作
					bool needMissionEnd = false;
					double stopVel = 0;
					double advanceS = 0;
					if( missionEndMode )
					{
						needMissionEnd = true;
						stopVel = MissionEnd_v1;
						if( stopVel > line_track_desired_maxv )
							stopVel = line_track_desired_maxv;
						advanceS = MissionEnd_advanceS;
					}
					
					//计算跟踪A点位置
					vector3<double> A_desired = target_position;
					double A_B_length = safe_sqrt(route_line_A_B.get_square());	
					double line_track_desired_acc = 0;		
					if( A_B_length > 0.1 )
					{
						double stopVelDiff = line_track_desired_vel - stopVel;
						if( stopVelDiff < 0 )
							stopVelDiff = 0;
						if( ( (0.5*sq(stopVelDiff) + stopVel*stopVelDiff)/line_track_desired_maxacc >= A_B_length - line_track_desired - advanceS ) ||
								( max_line_track_desired>0 && 0.5*sq(line_track_desired_vel)/line_track_desired_maxacc>=max_line_track_desired-line_track_desired )
							)
						{	//需要减速
							line_track_desired_vel -= line_track_desired_maxacc*h;
							if( line_track_desired_vel < 0 )
								line_track_desired_vel = 0;
							else
								line_track_desired_acc = -line_track_desired_maxacc;
						}
						else
						{	//前馈加速到最高速度
							double max_v = line_track_desired_maxv;
							if( line_track_desired_vel < max_v )
								line_track_desired_vel += line_track_desired_maxacc*h;
							if( line_track_desired_vel > max_v )
								line_track_desired_vel = max_v;
							else
								line_track_desired_acc = +line_track_desired_maxacc;
						}
						//根据障碍限制目标速度
						if( max_line_track_desired_vel>0 && line_track_desired_vel>max_line_track_desired_vel )
							line_track_desired_vel = max_line_track_desired_vel;
						
						
						vector3<double> B = target_position + route_line_A_B;
						A_desired = B - route_line_A_B*(line_track_desired/A_B_length);
						
						frontDir = atan2( -route_line_A_B.y, -route_line_A_B.x );
						frontVec.set_vector( -route_line_A_B.x, -route_line_A_B.y, -route_line_A_B.z );
					}
					else
					{
						frontDir = -10;
						frontVec.zero();
					}
					
					//计算偏差
					vector3<double> e1r = A_desired - foot_point;
					vector3<double> e1d = foot_point - Position;
					double e1r_length = safe_sqrt(e1r.get_square());
					double e1d_length = safe_sqrt(e1d.get_square());
					
					//计算route方向单位向量
					vector3<double> route_n;
					if( e1r_length > 0.001 )
						route_n = e1r * (1.0/e1r_length);

					double route_sign = 1.0;
					if( route_n*route_line_A_B > 0 )
						route_sign = -1.0;
					
					//移动目标位置
					if( e1r_length < 10*100 )
					{
						line_track_desired += line_track_desired_vel*h;
						if( line_track_desired > A_B_length )
							line_track_desired = A_B_length;
						//根据障碍限制目标位置
						if( max_line_track_desired>0 && line_track_desired>max_line_track_desired )
							line_track_desired = max_line_track_desired;
					}
					
					//计算e1导数
					double e1r_1_length = -(VelocityENU * route_n);	
					vector3<double> e1r_1 = route_n * e1r_1_length;					
					vector3<double> e1d_1 = -(VelocityENU + e1r_1);
					e1r_1_length += route_sign*line_track_desired_vel;
					//e1二阶导
					vector3<double> e1_2( TAcc.x, TAcc.y, AccelerationENU_ESO_Z );
					double e1r_2_length = -(e1_2 * route_n);
					vector3<double> e1r_2 = route_n * e1r_2_length;					
					vector3<double> e1d_2 = -(e1_2 + e1r_2);
					e1r_2_length += route_sign*line_track_desired_acc;
					
					pos_vel = cfg.maxAutoVelXY[0];
					if( pos_vel < 200 )
						pos_vel = 200;
					/*route方向*/
						double desired_vel_ctrl = line_track_desired_vel - DESIRED_VEL_CTRL_DEC;
						if( desired_vel_ctrl < 0 )
							desired_vel_ctrl = 0;
					
						smooth_kp_d2 d1r = smooth_kp_2( e1r_length, e1r_1_length, e1r_2_length , Ps, line_track_desired_maxv+100 );
						vector3<double> T2r = route_n * ( d1r.d0 + desired_vel_ctrl );
						vector3<double> T2r_1 = route_n * d1r.d1;
						vector3<double> T2r_2 = route_n * d1r.d2;
					
						T2r_1.x *= T21P;
						T2r_1.y *= T21P;
					/*route方向*/

					/*d方向*/
						e_1_n = e1d.x*e1d_1.x + e1d.y*e1d_1.y + e1d.z*e1d_1.z;
						if( !is_zero(e1d_length) )
							e_1 = e_1_n / e1d_length;
						else
							e_1 = 0;
						e_2_n = ( e1d.x*e1d_2.x + e1d.y*e1d_2.y + e1d.z*e1d_2.z + e1d_1.x*e1d_1.x + e1d_1.y*e1d_1.y + e1d_1.z*e1d_1.z )*e1d_length - e_1*e_1_n;
						if( !is_zero(e1d_length*e1d_length) )
							e_2 = e_2_n / (e1d_length*e1d_length);
						else
							e_2 = 0;
						smooth_kp_d2 d1d = smooth_kp_2( e1d_length, e_1, e_2 , Ps, pos_vel );
						vector3<double> T2d;
						vector3<double> T2d_1;
						vector3<double> T2d_2;
						if( !is_zero(e1d_length*e1d_length*e1d_length) )
						{
							vector3<double> n = e1d * (1.0/e1d_length);
							vector3<double> n_1 = (e1d_1*e1d_length - e1d*e_1) / (e1d_length*e1d_length);
							vector3<double> n_2 = ( (e1d_2*e1d_length-e1d*e_2)*e1d_length - (e1d_1*e1d_length-e1d*e_1)*(2*e_1) ) / (e1d_length*e1d_length*e1d_length);
							T2d = n*d1d.d0;
							T2d_1 = n*d1d.d1 + n_1*d1d.d0;
							T2d_2 = n*d1d.d2 + n_1*(2*d1d.d1) + n_2*d1d.d0;
						}
						
						T2d_1.x *= T21P;
						T2d_1.y *= T21P;
					/*d方向*/
						
					TargetVelocity = T2r + T2d;
					TargetVelocity_1 = T2r_1 + T2d_1;
					TargetVelocity_2 = T2r_2 + T2d_2;
						
					//判断到点
					pos_vel = line_track_desired_maxv;
					double wp_range = 0;
					if( wp_range < 0.1 )
						wp_range = 0.1;
					if( A_B_length - line_track_desired <= wp_range+advanceS ) 
					{
						if( !needMissionEnd )
						{
							//计算到点延时时间
							double wp_delay = 1;
							
							//到点延时
							++line_track_delay_counter;
							if( line_track_delay_counter >= wp_delay*CtrlRateHz )
								HorizontalPosition_ControlMode = Altitude_ControlMode = Position_ControlMode_Position;
						}
						else
						{
							switch(missionEndMode)
							{
								case MissionEndMode_Arc:
								{	//圆弧结束
									target_position = A_desired;
									vector3<double> routeDist = MissionEnd_P2 % (target_position - MissionEnd_P1);
									MissionEnd_TotalRoute = safe_sqrt(routeDist.get_square());
									line_track_desired = 0;
									HorizontalPosition_ControlMode = Altitude_ControlMode = Position_ControlMode_MissionEndArc3D;
									break;
								}
								
								default:
								case MissionEndMode_null:
								{
									//计算到点延时时间
									double wp_delay = 1;
									
									//到点延时
									++line_track_delay_counter;
									if( line_track_delay_counter >= wp_delay*CtrlRateHz )
										HorizontalPosition_ControlMode = Altitude_ControlMode = Position_ControlMode_Position;
									break;
								}
							}
						}
					}
					else
						line_track_delay_counter = 0;
				}
				else
				{
					//没起飞时重置期望速度
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					goto PosCtrl_Finish;
				}
				break;
			}
			case Position_ControlMode_MissionEndArc3D:
			{
				if( inFlight )
				{
					//求当前前进速度
					bool trackComplete = false;
					double trackPercent;
					double old_line_track_desired = line_track_desired;
					if( MissionEnd_TotalRoute > 0.01 )
					{
						//移动目标位置
						line_track_desired += line_track_desired_vel*h;
						trackPercent = line_track_desired / MissionEnd_TotalRoute;
						line_track_desired_vel = trackPercent * ( MissionEnd_v2 - MissionEnd_v1 ) + MissionEnd_v1;
						if( line_track_desired > MissionEnd_TotalRoute )
						{
							line_track_desired = MissionEnd_TotalRoute;
							trackComplete = true;
						}
					}
					else
					{
						line_track_desired_vel = MissionEnd_v2;
						line_track_desired = MissionEnd_TotalRoute;
						trackPercent = 1;
						trackComplete = true;
					}
					//求旋转后的位置
					Quaternion quat;
					quat.rotate_delta_angle(MissionEnd_P2*trackPercent);
					vector3<double> rotateP = quat.rotate(target_position - MissionEnd_P1);
					vector3<double> new_target_position = rotateP + MissionEnd_P1;
					
					//计算速度
					vector3<double> routeVel = MissionEnd_P2 % rotateP;
					double routeVelLength = safe_sqrt(routeVel.get_square());
					if( routeVelLength > 0.1 )
					{
						frontDir = atan2( routeVel.y, routeVel.x );
						frontVec = routeVel;
						
						double desired_vel_ctrl = line_track_desired_vel - DESIRED_VEL_CTRL_DEC;
						if( desired_vel_ctrl < 0 )
							desired_vel_ctrl = 0;
						routeVel *= desired_vel_ctrl * (1.0/routeVelLength);
					}
					else
					{
						routeVel.zero();
						frontDir = -10;
						frontVec.zero();
					}
					
					//进行位置控制
					vector3<double> e1 = new_target_position - Position;
					vector3<double> e1_1 = -VelocityENU;
					double e1_length = safe_sqrt(e1.get_square());
					e_1_n = e1.x*e1_1.x + e1.y*e1_1.y + e1.z*e1_1.z;
					if( !is_zero(e1_length) )
						e_1 = e_1_n / e1_length;
					else
						e_1 = 0;
					
					//误差过大恢复目标位置
					if( e1_length > 10*100 )
						line_track_desired = old_line_track_desired;
					
					double _pos_vel = cfg.maxAutoVelXY[0];			
					smooth_kp_d2 d1 = smooth_kp_2( e1_length, e_1, e_2 , Ps, _pos_vel );
					vector3<double> T2;
					vector3<double> T2_1;
					vector3<double> T2_2;
					if( !is_zero(e1_length*e1_length*e1_length) )
					{
						vector3<double> n = e1 * (1.0/e1_length);
						vector3<double> n_1 = (e1_1*e1_length - e1*e_1) / (e1_length*e1_length);
						T2 = n*d1.d0;
						T2_1 = n*d1.d1 + n_1*d1.d0;
					}
					TargetVelocity = T2 + routeVel;
					TargetVelocity_1 = T2_1*T21P;
					
					//追踪完成
					if(trackComplete)
					{
						resetMissionEndMode();
						target_position = new_target_position;
						switch(prestoreMission)
						{
							case PrestoreMission_Line:
							{	//加载直线飞行
								vector3<double> position = target_position;
								target_position = prestoreMission_P1;
								
								//calculate vector B-A
								route_line_A_B = position - target_position;
								double route_line_A_B_sq = route_line_A_B.get_square();
								if( route_line_A_B_sq > 0.1 )
								{
									//calculate 1/(B-A)^2
									route_line_m = 1.0 / route_line_A_B_sq;

									HorizontalPosition_ControlMode = Altitude_ControlMode = Position_ControlMode_RouteLine3D;
									reset_line_track_state(AutoVelXYZ,cfg.maxAutoAccXY[0],MissionEnd_v2)
								}
								else
									HorizontalPosition_ControlMode = Altitude_ControlMode = Position_ControlMode_Position;
							
								//复位航线避障
								max_line_track_desired = -1;
								max_line_track_desired_vel = -1;
								
								break;
							}
							
							default:
							{
								HorizontalPosition_ControlMode = Altitude_ControlMode = Position_ControlMode_Position;
								break;
							}
						}
						reset_prestoreMission();
					}
				}
				else
				{	//没起飞前在位置控制模式
					//重置期望位置
					target_position.x = Position.x;
					target_position.y = Position.y;
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					goto PosCtrl_Finish;
				}
				break;
			}
			
			case Position_ControlMode_Locking:
			default:
			{	//刹车锁位置
				static uint16_t lock_counter = 0;
				if( inFlight )
				{
					double ExpectVelLength = safe_sqrt(Expected_VelXY.get_square());
					if( ExpectVelLength > XYLock_CAcc*h )
					{
						lock_counter = 0;
						double inv_ExpectVelLength = 1.0 / ExpectVelLength;
						vector2<double> dec = Expected_VelXY*(inv_ExpectVelLength * XYLock_CAcc*h);
						Expected_VelXY -= dec;
						TargetVelocity.x = Expected_VelXY.x;
						TargetVelocity.y = Expected_VelXY.y;
					}
					else
					{
						TargetVelocity.x = 0;
						TargetVelocity.y = 0;
						double breakT = 1.2;
						if( cfg.breakT[0] > 0 )
							breakT = cfg.breakT[0];
						if( ++lock_counter >= CtrlRateHz*breakT )
						{
							lock_counter = 0;
							TargetVelocity.zero();
							target_position.x = Position.x + 1.0/Pv*VelocityENU.x;
							target_position.y = Position.y + 1.0/Pv*VelocityENU.y;
							HorizontalPosition_ControlMode = Position_ControlMode_Position;
						}
					}
				}
				else
				{
					lock_counter = 0;
					target_position.x = Position.x;
					target_position.y = Position.y;
					HorizontalPosition_ControlMode = Position_ControlMode_Position;
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					goto PosCtrl_Finish;
				}
				break;
			}
		}	
		
		//计算期望加速度
		vector2<double> e2;
		e2.x = TargetVelocity.x - VelocityENU.x;
		e2.y = TargetVelocity.y - VelocityENU.y;
		vector2<double> e2_1;
		e2_1.x = TargetVelocity_1.x - TAcc.x;
		e2_1.y = TargetVelocity_1.y - TAcc.y;
		double e2_length = safe_sqrt(e2.get_square());
		e_1_n = e2.x*e2_1.x + e2.y*e2_1.y;
		if( !is_zero(e2_length) )
			e_1 = e_1_n / e2_length;
		else
			e_1 = 0;
		smooth_kp_d1 d2;
		if( HorizontalPosition_ControlMode==Position_ControlMode_Locking )
		{
			XYLock_CAcc += h * cfg.maxBreakJerk[0];
			if( XYLock_maxAcc>50 )
				XYLock_CAcc = constrain( XYLock_CAcc, 10.0, XYLock_maxAcc );
			else
				XYLock_CAcc = constrain( XYLock_CAcc, 10.0, (double)cfg.maxAccXY[0] );
			d2 = smooth_kp_1( e2_length, e_1 , Pv, cfg.maxAccXY[0] );
		}
		else
			d2 = smooth_kp_1( e2_length, e_1 , Pv, cfg.maxAccXY[0] );
		vector2<double> T3;
		vector2<double> T3_1;
		if( !is_zero(e2_length*e2_length) )
		{
			vector2<double> n = e2 * (1.0/e2_length);
			vector2<double> n_1 = (e2_1*e2_length - e2*e_1) / (e2_length*e2_length);
			T3 = n*d2.d0;
			T3_1 = n*d2.d1 + n_1*d2.d0;
		}
		T3 += vector2<double>( TargetVelocity_1.x, TargetVelocity_1.y );
		T3_1 += vector2<double>( TargetVelocity_2.x, TargetVelocity_2.y );
		
		static TD3_Lite TD3filter[2];
		TD3filter[0].track3(T3.x, h,  getAttCtrlCfg()->TD4_P1[0], getAttCtrlCfg()->TD4_P2[0], getAttCtrlCfg()->TD4_P3[0]);
		TD3filter[1].track3(T3.y, h,  getAttCtrlCfg()->TD4_P1[0], getAttCtrlCfg()->TD4_P2[0], getAttCtrlCfg()->TD4_P3[0]);
		
		if( HorizontalPosition_ControlMode!=Position_ControlMode_Locking )
		{
			double velLen = sq(VelocityENU.x) + sq(VelocityENU.y);
			if(velLen < 0.000001 ) {
				Expected_VelXY.zero();
			}
			else {
				velLen = safe_sqrt(velLen);
				double invVel = 1.0 / velLen;
				
				double reqAcc =  -(VelocityENU.x*TD3filter[0].get_x1() + VelocityENU.y*TD3filter[1].get_x1())*invVel;
				if( reqAcc < 0 ) {
					reqAcc = 0;
					Expected_VelXY.x = VelocityENU.x;
					Expected_VelXY.y = VelocityENU.y;
				}
				else
				{
					double reqVelErr = reqAcc / Pv;
					if( velLen <= reqVelErr ) {
						reqAcc = 0;
						Expected_VelXY.zero();
					}
					else {
						double reqVelLen = (velLen - reqVelErr)*invVel;
						Expected_VelXY.x = VelocityENU.x*reqVelLen;
						Expected_VelXY.y = VelocityENU.y*reqVelLen;
					}
				}
			}
		}
		
		vector2<double> e3;
		e3.x = T3.x - TAcc.x;
		e3.y = T3.y - TAcc.y;
		double e3_length = safe_sqrt(e3.get_square());
		double d3;
		if( Is_AutoMode(HorizontalPosition_ControlMode) )
			d3 = smooth_kp_0( e3_length , Pa, cfg.maxAutoJerkXY[0] );
		else
			d3 = smooth_kp_0( e3_length , Pa, cfg.maxJerkXY[0] );
		vector2<double> T4;
		if( !is_zero(e3_length) )
		{
			vector2<double> n = e3 * (1.0/e3_length);
			T4 = n*d3;
		}
		if( Is_AutoMode(HorizontalPosition_ControlMode) )
			T4.constrain( cfg.maxAutoJerkXY[0] );
		else
			T4.constrain( cfg.maxJerkXY[0] );
		T4 += T3_1;
		
		TAcc.x += T4.x*h;
		TAcc.y += T4.y*h;	
		
		//去除风力扰动
		vector3<double> WindDisturbance;
		get_WindDisturbance( &WindDisturbance );
		double windDisturbance_length = safe_sqrt( sq(WindDisturbance.x) + sq(WindDisturbance.y) );
		if( cfg.maxWindComp[0] >= 0 )
		{
			if( windDisturbance_length>0.001 && windDisturbance_length>cfg.maxWindComp[0] )
			{
				double scale = cfg.maxWindComp[0] / windDisturbance_length;
				WindDisturbance.x *= scale;
				WindDisturbance.y *= scale;
			}
		}
		vector2<double> target_acceleration;
//		target_acceleration.x = TAcc.x - WindDisturbance.x;
//		target_acceleration.y = TAcc.y - WindDisturbance.y;
		target_acceleration.x = T3.x;// - WindDisturbance.x;
		target_acceleration.y = T3.y;// - WindDisturbance.y;
		
		//旋转至Bodyheading
		Quaternion attitude;
		get_AirframeY_quat(&attitude);
		double yaw;
		attitude.get_RP_quat(&yaw);
		
		static double yaw_filted = 0;
		if( windDisturbance_length < 300 )
			yaw_filted = yaw;
		else
		{
			double trackVel;
			Attitude_Control_get_YawTrackVel(&trackVel);
			yaw_filted += trackVel*h;
			double kp = 1000 / windDisturbance_length;
			
			double err = yaw - yaw_filted;
			while( err > Pi )
				err -= 2*Pi;
			while( err < -Pi )
				err += 2*Pi;
			yaw_filted += kp*h * err;
			if( yaw_filted > Pi )
				yaw_filted -= 2*Pi;
			if( yaw_filted < -Pi )
				yaw_filted += 2*Pi;
		}
		
		double sin_Yaw, cos_Yaw;
		fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
		double sin_Yaw_filted, cos_Yaw_filted;
		fast_sin_cos( yaw_filted, &sin_Yaw_filted, &cos_Yaw_filted );
		double target_acceleration_x_bodyheading = ENU2BodyHeading_x( target_acceleration.x , target_acceleration.y , sin_Yaw_filted , cos_Yaw_filted );
		double target_acceleration_y_bodyheading = ENU2BodyHeading_y( target_acceleration.x , target_acceleration.y , sin_Yaw_filted , cos_Yaw_filted );
//		target_acceleration_x_bodyheading = ThrOut_Filters[0].run(target_acceleration_x_bodyheading);
//		target_acceleration_y_bodyheading = ThrOut_Filters[1].run(target_acceleration_y_bodyheading);
		
		//计算风力补偿角度
		double WindDisturbance_Bodyheading_x = ENU2BodyHeading_x( WindDisturbance.x , WindDisturbance.y , sin_Yaw_filted , cos_Yaw_filted );
		double WindDisturbance_Bodyheading_y = ENU2BodyHeading_y( WindDisturbance.x , WindDisturbance.y , sin_Yaw_filted , cos_Yaw_filted );
		
		target_acceleration_x_bodyheading -= WindDisturbance_Bodyheading_x;
		target_acceleration_y_bodyheading -= WindDisturbance_Bodyheading_y;
		
		//计算角度
		double AccUp = GravityAcc;// + AccelerationENU_ESO_Z;
		
		//计算目标角度
		double target_Roll = atan2( -target_acceleration_y_bodyheading , AccUp );
		double target_Pitch = atan2( target_acceleration_x_bodyheading , AccUp );
		if( HorizontalPosition_ControlMode==Position_ControlMode_Velocity )
		{	//角度限幅
			//机体系加速度前馈
			double targetVelocity_Body_x_1 = ENU2BodyHeading_x( TargetVelocity_1.x , TargetVelocity_1.y , sin_Yaw , cos_Yaw );
			double targetVelocity_Body_y_1 = ENU2BodyHeading_y( TargetVelocity_1.x , TargetVelocity_1.y , sin_Yaw , cos_Yaw );
			double AntiDisturbancePitch = atan2( -(WindDisturbance_Bodyheading_x - targetVelocity_Body_x_1) , AccUp );
			double AntiDisturbanceRoll = atan2( (WindDisturbance_Bodyheading_y - targetVelocity_Body_y_1) , AccUp );
			if( VelCtrlMaxRoll>0 && VelCtrlMaxPitch>0 )
			{
				target_Roll = constrain( target_Roll , AntiDisturbanceRoll - VelCtrlMaxRoll, AntiDisturbanceRoll + VelCtrlMaxRoll );
				target_Pitch = constrain( target_Pitch , AntiDisturbancePitch - VelCtrlMaxPitch, AntiDisturbancePitch + VelCtrlMaxPitch );
			}
			else if( VelCtrlMaxRoll>0 )
			{
				vector2<double> Tangle( target_Roll - AntiDisturbanceRoll, target_Pitch - AntiDisturbancePitch );
				Tangle.constrain(VelCtrlMaxRoll);
				target_Roll = AntiDisturbanceRoll + Tangle.x;
				target_Pitch = AntiDisturbancePitch + Tangle.y;
			}
		}
		
		//设定目标角度
		Attitude_Control_set_Target_RollPitch( target_Roll, target_Pitch );
		
		//获取真实目标角度修正TAcc
		Attitude_Control_get_Target_RollPitch( &target_Roll, &target_Pitch );
		target_acceleration_x_bodyheading = tan(target_Pitch)*GravityAcc;
		target_acceleration_y_bodyheading = -tan(target_Roll)*GravityAcc;
		target_acceleration.x = BodyHeading2ENU_x( target_acceleration_x_bodyheading, target_acceleration_y_bodyheading , sin_Yaw, cos_Yaw );
		target_acceleration.y = BodyHeading2ENU_y( target_acceleration_x_bodyheading, target_acceleration_y_bodyheading , sin_Yaw, cos_Yaw );
		TAcc.x = target_acceleration.x + WindDisturbance.x;
		TAcc.y = target_acceleration.y + WindDisturbance.y;
	}//水平位置控制
	else
	{
		ThrOut_Filters[0].reset(0);
		ThrOut_Filters[1].reset(0);
	}
	
PosCtrl_Finish:	
	if( Altitude_Control_Enabled )
	{//高度控制
			
		//设置控制量限幅
		Target_tracker[2].r2p = cfg.maxVelUp[0];
		Target_tracker[2].r2n = cfg.maxVelDown[0];
		Target_tracker[2].r3p = cfg.maxAccUp[0];
		Target_tracker[2].r3n = cfg.maxAccDown[0];
		Target_tracker[2].r4p = cfg.maxJerkUp[0];
		Target_tracker[2].r4n = cfg.maxJerkDown[0];
		
		if( !Is_3DMode(Altitude_ControlMode) )
		{
			switch( Altitude_ControlMode )
			{
				case Position_ControlMode_Position:
				{	//控制位置
					if( inFlight )
					{
						Target_tracker[2].r2p = 0.3*cfg.maxVelUp[0];
						Target_tracker[2].r2n = 0.3*cfg.maxVelDown[0];
						Target_tracker[2].track4( target_position.z , 1.0 / CtrlRateHz );
					}
					else
					{
						//没起飞前在位置控制模式
						//不要起飞
						Target_tracker[2].reset();
						target_position.z = Target_tracker[2].x1 = Position.z;
						Attitude_Control_set_Throttle( getAttCtrlCfg()->idleThrottlePct );
						goto AltCtrl_Finish;
					}
					break;
				}
				case Position_ControlMode_OffBoard:
				{	//控制位置
					bool OffBoardUp = false;
					if( OffBoardZ_target_mode==3 )
					{						
						if( target_position.z>=Position.z+10 && target_velocity.z>=0 && target_acc.z>=0 )
							OffBoardUp = true;
					}
					else
					{
						if( target_velocity.z>=10 && target_acc.z>=0 )
							OffBoardUp = true;
					}
					if( inFlight || OffBoardUp )
					{
						if( OffBoardZ_target_mode == 3 )
						{	//位置+速度+加速度
							Target_tracker[2].r2p = cfg.maxVelUp[0];
							Target_tracker[2].r2n = cfg.maxVelDown[0];
							Target_tracker[2].track4( target_position.z,target_velocity.z,target_acc.z,0,0, 1.0/CtrlRateHz );
							
							//记录位置误差
							posErr.z = Target_tracker[2].x1 - Position.z;
						}
						else
						{	//速度+加速度
							double TVel;
							if( target_velocity.z > cfg.maxVelUp[0] )
								TVel = cfg.maxVelUp[0];
							else if( target_velocity.z < -cfg.maxVelDown[0] )
								TVel = -cfg.maxVelDown[0];
							else
								TVel = target_velocity.z;
							Target_tracker[2].track3( TVel,target_acc.z,0,0 , 1.0/CtrlRateHz );
						}
					}
					else
					{	//没起飞前在位置控制模式
						//不要起飞
						Target_tracker[2].reset();
						target_position.z = Target_tracker[2].x1 = Position.z;
						Attitude_Control_set_Throttle( getAttCtrlCfg()->idleThrottlePct );
						goto AltCtrl_Finish;
					}
					break;
				}
				case Position_ControlMode_Velocity:
				{	//控制速度
					if( inFlight )
					{
						double TVel;
						if( target_velocity.z > cfg.maxVelUp[0] )
							TVel = cfg.maxVelUp[0];
						else if( target_velocity.z < -cfg.maxVelDown[0] )
							TVel = -cfg.maxVelDown[0];
						else
							TVel = target_velocity.z;
						Target_tracker[2].track3( TVel , 1.0 / CtrlRateHz );
					}
					else
					{
						//没起飞且期望速度为负
						//不要起飞
						Target_tracker[2].reset();
						Target_tracker[2].x1 = Position.z;
						Target_tracker[2].x2 = 35;
						if( target_velocity.z <= 0 )
						{
							Attitude_Control_set_Throttle( getAttCtrlCfg()->idleThrottlePct );
							goto AltCtrl_Finish;
						}
					}
					break;
				}
				
				case Position_ControlMode_Takeoff:
				{	//起飞
					//设置控制量最大值
					Target_tracker[2].r3p = cfg.maxAutoAccUp[0];
					Target_tracker[2].r3n = cfg.maxAutoAccDown[0];
					Target_tracker[2].r4p = cfg.maxAutoJerkUp[0];
					Target_tracker[2].r4n = cfg.maxAutoJerkDown[0];
					
					if( inFlight )
					{
						//已起飞
						//控制达到目标高度
						double homeZ;
						getHomeLocalZ(&homeZ);
						if( Position.z - homeZ < 100 )
							Target_tracker[2].r2n = Target_tracker[2].r2p = 50;
						else
							Target_tracker[2].r2n = Target_tracker[2].r2p = AutoVelZUp;
						Target_tracker[2].track4( target_position.z , 1.0 / CtrlRateHz );
						if( fabs( Target_tracker[2].x1 - target_position.z ) < 0.7 && \
								fabs( target_position.z - Position.z ) < 50 && \
								in_symmetry_range( Target_tracker[2].x2 , 0.7 ) && \
								in_symmetry_range( Target_tracker[2].x3 , 0.7 )	)
							Altitude_ControlMode = Position_ControlMode_Position;
					}
					else
					{
						//未起飞
						//等待起飞
						target_position.z = Position.z + TakeoffHeight;
						Target_tracker[2].reset();
						Target_tracker[2].x1 = Position.z;
						Target_tracker[2].x2 = 35;
					}
					break;
				}
				case Position_ControlMode_RouteLine:
				{	//飞到指定高度
					
					//设置控制量最大值
					Target_tracker[2].r3p = cfg.maxAutoAccUp[0];
					Target_tracker[2].r3n = cfg.maxAutoAccDown[0];
					Target_tracker[2].r4p = cfg.maxAutoJerkUp[0];
					Target_tracker[2].r4n = cfg.maxAutoJerkDown[0];
					
					if( inFlight )
					{
						//已起飞
						//控制达到目标高度
						Target_tracker[2].r2p = AutoVelZUp;
						Target_tracker[2].r2n = AutoVelZDown;
						Target_tracker[2].track4( target_position.z , 1.0f / CtrlRateHz );
						if( fabs( Target_tracker[2].x1 - target_position.z ) < 0.7 && \
								fabs( target_position.z - Position.z ) < 50 && \
								in_symmetry_range( Target_tracker[2].x2 , 0.7 ) && \
								in_symmetry_range( Target_tracker[2].x3 , 0.7 )	)
							Altitude_ControlMode = Position_ControlMode_Position;
					}
					else
					{
						//未起飞
						//不要起飞
						Target_tracker[2].reset();
						Target_tracker[2].x1 = Position.z;
						Attitude_Control_set_Throttle( getAttCtrlCfg()->idleThrottlePct );
						goto AltCtrl_Finish;
					}
					break;
				}
				
				case Position_ControlMode_Locking:
				default:
				{	//锁位置（减速到0然后锁住高度）
					if( inFlight )
					{
						Target_tracker[2].track3( 0 , 1.0 / CtrlRateHz );
						if( in_symmetry_range( VelocityENU.z , 10.0 ) && \
								in_symmetry_range( Target_tracker[2].x2 , 0.3 ) && \
								in_symmetry_range( Target_tracker[2].x3 , 0.3 ) && \
								in_symmetry_range( Target_tracker[2].x4 , 0.3 )	)
						{
							target_position.z = Target_tracker[2].x1 = Position.z;
							Altitude_ControlMode = Position_ControlMode_Position;
						}
					}
					else
					{
						Altitude_ControlMode = Position_ControlMode_Position;
						Attitude_Control_set_Throttle( getAttCtrlCfg()->idleThrottlePct );
						goto AltCtrl_Finish;
					}
					break;
				}
			}
		}
		
		if( inFlight )
		{
			//计算期望速度
			double target_velocity_z;
			//期望垂直速度的导数
			double Tvz_1, Tvz_2;
			if( Is_3DAutoMode(Altitude_ControlMode) )
			{
				target_velocity_z = TargetVelocity.z;
				Tvz_1 = TargetVelocity_1.z;
				Tvz_2 = TargetVelocity_2.z;
				Target_tracker[2].reset();
				Target_tracker[2].x1 = target_position.z;
			}
			else
			{
				if( Target_tracker[2].get_tracking_mode() == 4 )
				{
					double max_fb_vel = ( Target_tracker[2].x1 - Position.z ) > 0 ? cfg.maxAutoVelUp[0] : cfg.maxAutoVelDown[0];
					smooth_kp_d2 TvFb = smooth_kp_2(
							Target_tracker[2].x1 - Position.z,
							Target_tracker[2].x2 - VelocityENU.z, 
							Target_tracker[2].x3 - AccelerationENU_ESO_Z, 
							Ps, max_fb_vel );
					target_velocity_z = TvFb.d0 + Target_tracker[2].x2;
					Tvz_1 = TvFb.d1 + Target_tracker[2].x3;
					Tvz_2 = TvFb.d2 + Target_tracker[2].x4;
				}
				else
				{
					target_velocity_z = Target_tracker[2].x2;
					Tvz_1 = Target_tracker[2].x3;
					Tvz_2 = Target_tracker[2].x4;
				}
			}
			
			//计算期望加速度
			double max_fb_acc = ( target_velocity_z - VelocityENU.z ) > 0 ? cfg.maxAutoAccUp[0] : cfg.maxAutoAccDown[0];
			smooth_kp_d1 TaFb = smooth_kp_1(
					target_velocity_z - VelocityENU.z,
					Tvz_1 - AccelerationENU_ESO_Z, 
					2, max_fb_acc );
			double target_acceleration_z = TaFb.d0 + Tvz_1;
			double target_acceleration_z_1 = TaFb.d1 + Tvz_2;
			
			//长期速度误差时加速修正
			double velErr = target_velocity_z - VelocityENU.z;
			static float velErr_lastT = 0;
			if( velErr>7 && velErr_lastT>=0 )
				velErr_lastT += h;
			else if( velErr<-7 && velErr_lastT<=0 )
				velErr_lastT -= h;
			else
				velErr_lastT = 0;
			float targetAccZ_Inc = constrain( 80 * velErr_lastT*velErr_lastT*velErr_lastT, -500.0f, 500.0f );
			target_acceleration_z += targetAccZ_Inc;
			
			//target_acceleration_z = TargetVelocityFilter[2].run( target_acceleration_z );
			//加速度误差
			double acceleration_z_error = target_acceleration_z - AccelerationENU.z;
			double AccZErrP = cfg.ZSense[0]*10;
			double accZErrD;
			if( cfg.ZSenseOrder[0] == 4 )
			{
				acceleration_z_error = AccZ_Err_filter.track4( acceleration_z_error, h, AccZErrP,AccZErrP,AccZErrP,AccZErrP );
				accZErrD = AccZ_Err_filter.get_x2();
				AccZ_Err_filter.x1 -= h*( Pa*acceleration_z_error + 0*accZErrD );
				AccZ_Err_filter.x2 -= Pa*acceleration_z_error + 0*accZErrD;
			}
			else
			{
				acceleration_z_error = AccZ_Err_filter.track3( acceleration_z_error, h, AccZErrP,AccZErrP,AccZErrP );
				accZErrD = AccZ_Err_filter.get_x3();
				AccZ_Err_filter.x2 -= h*( Pa*acceleration_z_error + 0*accZErrD );
				AccZ_Err_filter.x3 -= Pa*acceleration_z_error + 0*accZErrD;
			}
			
			//获取倾角cosin
			Quaternion quat;
			get_Airframe_quat(&quat);
			double lean_cosin = quat.get_lean_angle_cosin();
			
			//获取悬停油门 - 电机起转油门
			double hover_throttle;
			get_hover_throttle(&hover_throttle);
			
			//计算输出油门
			double force, T, b;
			get_throttle_force(&force);
			get_ESO_height_T(&T);
			get_throttle_b(&b);
			if( force < 1 )
				force = 1;
			double errD_kp = 0;
			if( cfg.ZSenseD[0]>=0 && cfg.ZSenseD[0]<=10 )
				errD_kp = cfg.ZSenseD[0]*0.1;
			
			double leanKp = lean_cosin;
			if( leanKp < 0.3 )
				leanKp = 0.3;
			double throttle = ( force + T * ( Pa*leanKp*acceleration_z_error + errD_kp*accZErrD + target_acceleration_z_1 ) )/b;
			//double throttle = ( target_acceleration_z + Pa*T*acceleration_z_error )/b + hover_throttle;
			//倾角补偿
			if( lean_cosin > 0.1 )				
				throttle /= lean_cosin;
			else	//倾角太大
				throttle = 50;
			
			if( Attitude_Control_Enabled )
			{
				double logbuf[10];
				logbuf[0] = throttle;
				logbuf[1] = hover_throttle;
				logbuf[2] = force;
				logbuf[3] = target_acceleration_z;
				logbuf[4] = AccelerationENU_ESO_Z;
				SDLog_Msg_DebugVect( "thr", logbuf, 5 );
			}
			
			//油门限幅
			if( throttle > 100 )
				throttle = 100;
			if( inFlight )
			{
				if( throttle < 0 )
					throttle = 0;
			}
			
			//输出
			Attitude_Control_set_Throttle( throttle );
		}
		else
		{
			//没起飞
			//均匀增加油门起飞
			double throttle;
			get_OutputThrottle(&throttle);
			ThrOut_Filters[2].reset(throttle);
			AccZ_Err_filter.reset();
			throttle = constrain( throttle + h * 35, (double)getAttCtrlCfg()->minThrottlePct, 100.0 );
			Attitude_Control_set_Throttle( throttle );
		}
		
	}//高度控制
AltCtrl_Finish:
	update_history_PositionErr(posErr);
}

void init_Ctrl_Position()
{
	//初始化期望TD4滤波器
	Target_tracker[0] = TD4_SL( 15 , 15 , 15 , 15 );
	Target_tracker[1] = TD4_SL( 15 , 15 , 15 , 15 );
	Target_tracker[2] = TD4_SL( 1 , 2 , 50 , 60 );	
	Target_tracker[2].r3p = 900;
	Target_tracker[2].r3n = 600;
	
	//初始化期望速度低通滤波器
	TargetVelocityFilter[0].set_cutoff_frequency( CtrlRateHz , 10 );
	TargetVelocityFilter[1].set_cutoff_frequency( CtrlRateHz , 10 );
	TargetVelocityFilter[2].set_cutoff_frequency( CtrlRateHz , 10 );
	
	AccZ_Err_filter.reset();
	
	/*初始化参数*/
		PosCtrlCfg initial_cfg;
		//默认XY航线速度
		initial_cfg.AutoVXY[0] = 1000;
		//默认Z向上航线速度
		initial_cfg.AutoVZUp[0] = 450;
		//默认Z向下航线速度
		initial_cfg.AutoVZDown[0] = 350;
		//默认XYZ航线速度
		initial_cfg.AutoVXYZ[0] = 1000;	
		
		//高度前馈滤波器
		initial_cfg.Z_TD4P1[0] = 1;
		initial_cfg.Z_TD4P2[0] = 5;
		initial_cfg.Z_TD4P3[0] = 50;
		initial_cfg.Z_TD4P4[0] = 60;
		
		//位置反馈增益
		initial_cfg.P1[0] = 1;
		//速度反馈增益
		initial_cfg.P2[0] = 2;
		//加速度反馈增益
		initial_cfg.P3[0] = 10;
		//水平速度控制速度反馈增益
		initial_cfg.P2_VelXY[0] = 4;
		//水平速度前馈
		initial_cfg.VelXYFF[0] = 0.3;
		//高度加速度扰动补偿滤波器阶数
		initial_cfg.ZSenseOrder[0] = 2;
		//垂直加速度反馈滤波器截止频率
		initial_cfg.ZSense[0] = 4;
		//垂直加速度扰动补偿滤波器
		initial_cfg.ZSenseD[0] = 0;
		//最大风力补偿
		initial_cfg.maxWindComp[0] = -1;
		//刹车延迟时间
		initial_cfg.breakT[0] = 1;
		
		//最大水平速度
		initial_cfg.maxVelXY[0] = 1500;
		//最大水平加速度
		initial_cfg.maxAccXY[0] = 600;
		//最大水平加加速度
		initial_cfg.maxJerkXY[0] = 3000;
		//自动模式最大水平速度
		initial_cfg.maxAutoVelXY[0] = 1200;
		//自动模式最大水平加速度
		initial_cfg.maxAutoAccXY[0] = 200;
		//自动模式最大水平加加速度
		initial_cfg.maxAutoJerkXY[0] = 600;
		
		//最大向上速度
		initial_cfg.maxVelUp[0] = 500;
		//最大向下速度
		initial_cfg.maxVelDown[0] = 400;
		//最大向上加速度
		initial_cfg.maxAccUp[0] = 800;
		//最大向下加速度
		initial_cfg.maxAccDown[0] = 600;
		//最大向上加加速度
		initial_cfg.maxJerkUp[0] = 5000;
		//最大向下加加速度
		initial_cfg.maxJerkDown[0] = 3000;		
		//自动模式最大向上速度
		initial_cfg.maxAutoVelUp[0] = 500;
		//自动模式最大向下速度
		initial_cfg.maxAutoVelDown[0] = 350;
		//自动模式最大向上加速度
		initial_cfg.maxAutoAccUp[0] = 250;
		//自动模式最大向下加速度
		initial_cfg.maxAutoAccDown[0] = 200;
		//自动模式最大向上加加速度
		initial_cfg.maxAutoJerkUp[0] = 800;
		//自动模式最大向下加加速度
		initial_cfg.maxAutoJerkDown[0] = 800;
		
		//最大刹车加速度比例
		initial_cfg.maxBreakAccPc[0] = 1.0f;
		//最大刹车加加速度
		initial_cfg.maxBreakJerk[0] = 2000;
		
		//到达目标点范围
		initial_cfg.TRadius[0] = -1;
		//降落速度
		initial_cfg.LandVel[0] = 50;
		//协调转弯速度
		initial_cfg.SMVel[0] = 250;
	/*初始化参数*/

	SName param_names[] = {
		//默认XY航线速度
		"PC_AutoVXY" ,	
		//默认Z向上航线速度
		"PC_AutoVZUp" ,	
		//默认Z向下航线速度
		"PC_AutoVZDn" ,	
		//默认XYZ航线速度
		"PC_AutoVXYZ" ,	
		
		//高度前馈滤波器
		"PC_Z_TD4P1" ,
		"PC_Z_TD4P2" ,
		"PC_Z_TD4P3" ,
		"PC_Z_TD4P4" ,
		
		//位置反馈增益
		"PC_P1" ,
		//速度反馈增益
		"PC_P2" ,
		//加速度反馈增益
		"PC_P3" ,
		//水平速度控制速度反馈增益
		"PC_P2VelXY" ,
		//水平速度前馈
		"PC_VelXYFF" ,
		//垂直加速度反馈滤波器阶数
		"PC_ZSenseOrder" ,
		//垂直加速度反馈滤波器截止频率
		"PC_ZSense" ,
		//垂直加速度扰动补偿滤波器
		"PC_ZSenseD" ,
		//最大风力补偿
		"PC_maxWindComp" ,
		//刹车延迟时间
		"PC_breakT" ,
		
		//最大水平速度
		"PC_maxVelXY" ,
		//最大水平加速度
		"PC_maxAccXY" ,
		//最大水平加加速度
		"PC_maxJerkXY" ,
		//自动模式最大水平速度
		"PC_maxAutoVelXY" ,
		//自动模式最大水平加速度
		"PC_maxAutoAccXY" ,
		//自动模式最大水平加加速度
		"PC_maxAutoJerkXY" ,
		
		//最大向上速度
		"PC_maxVelUp" ,
		//最大向下速度
		"PC_maxVelDn" ,
		//最大向上加速度
		"PC_maxAccUp" ,
		//最大向下加速度
		"PC_maxAccDn" ,
		//最大向上加加速度
		"PC_maxJerkUp" ,
		//最大向下加加速度
		"PC_maxJerkDn" ,
		//自动模式最大向上速度
		"PC_maxAutoVelUp" ,
		//自动模式最大向下速度
		"PC_maxAutoVelDn" ,
		//自动模式最大向上加速度
		"PC_maxAutoAccUp" ,
		//自动模式最大向下加速度
		"PC_maxAutoAccDn" ,
		//自动模式最大向上加加速度
		"PC_maxAutoJerkUp" ,
		//自动模式最大向下加加速度
		"PC_maxAutoJerkDn" ,
		
		//最大刹车加速度比例
		"PC_maxBreakAccPc",
		//最大刹车加加速度
		"PC_maxBreakJerk",
		
		//到达目标点范围
		"PC_TRadius" ,
		//降落速度
		"PC_LandVel" ,
		//协调转弯速度
		"PC_SMVel" ,
	};

	MAV_PARAM_TYPE param_types[] = {
		//默认XY航线速度
		MAV_PARAM_TYPE_REAL32 ,	
		//默认Z向上航线速度
		MAV_PARAM_TYPE_REAL32 ,	
		//默认Z向下航线速度
		MAV_PARAM_TYPE_REAL32 ,	
		//默认XYZ航线速度
		MAV_PARAM_TYPE_REAL32 ,	
		//高度前馈滤波器
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		
		//位置反馈增益
		MAV_PARAM_TYPE_REAL32 ,
		//速度反馈增益
		MAV_PARAM_TYPE_REAL32 ,
		//加速度反馈增益
		MAV_PARAM_TYPE_REAL32 ,
		//水平速度控制速度反馈增益
		MAV_PARAM_TYPE_REAL32 ,
		//水平速度前馈
		MAV_PARAM_TYPE_REAL32 ,
		//垂直加速度反馈滤波器阶数
		MAV_PARAM_TYPE_UINT8 ,
		//垂直加速度反馈滤波器截止频率
		MAV_PARAM_TYPE_REAL32 ,
		//垂直加速度扰动补偿滤波器
		MAV_PARAM_TYPE_REAL32 ,
		//最大风力补偿
		MAV_PARAM_TYPE_REAL32 ,
		//刹车延迟时间
		MAV_PARAM_TYPE_REAL32 ,
		
		//最大水平速度
		MAV_PARAM_TYPE_REAL32 ,
		//最大水平加速度
		MAV_PARAM_TYPE_REAL32 ,
		//最大水平加加速度
		MAV_PARAM_TYPE_REAL32 ,
		//自动模式最大水平速度
		MAV_PARAM_TYPE_REAL32 ,
		//自动模式最大水平加速度
		MAV_PARAM_TYPE_REAL32 ,
		//自动模式最大水平加加速度
		MAV_PARAM_TYPE_REAL32 ,
		
		//最大向上速度
		MAV_PARAM_TYPE_REAL32 ,
		//最大向下速度
		MAV_PARAM_TYPE_REAL32 ,
		//最大向上加速度
		MAV_PARAM_TYPE_REAL32 ,
		//最大向下加速度
		MAV_PARAM_TYPE_REAL32 ,
		//最大向上加加速度
		MAV_PARAM_TYPE_REAL32 ,
		//最大向下加加速度
		MAV_PARAM_TYPE_REAL32 ,	
		//自动模式最大向上速度
		MAV_PARAM_TYPE_REAL32 ,
		//自动模式最大向下速度
		MAV_PARAM_TYPE_REAL32 ,
		//自动模式最大向上加速度
		MAV_PARAM_TYPE_REAL32 ,
		//自动模式最大向下加速度
		MAV_PARAM_TYPE_REAL32 ,
		//自动模式最大向上加加速度
		MAV_PARAM_TYPE_REAL32 ,
		//自动模式最大向下加加速度
		MAV_PARAM_TYPE_REAL32 ,
		
		//最大刹车加速度比例
		MAV_PARAM_TYPE_REAL32,
		//最大刹车加加速度
		MAV_PARAM_TYPE_REAL32,
		
		//到达目标点范围
		MAV_PARAM_TYPE_REAL32 ,
		//降落速度
		MAV_PARAM_TYPE_REAL32 ,
		//协调转弯速度
		MAV_PARAM_TYPE_REAL32 ,
	};

	ThrOut_Filters[0].set_cutoff_frequency( CtrlRateHz, 20 );
	ThrOut_Filters[1].set_cutoff_frequency( CtrlRateHz, 20 );
	ThrOut_Filters[2].set_cutoff_frequency( CtrlRateHz, 20 );
	
	ParamGroupRegister( "PosCtrl", 7, sizeof(cfg)/8, param_types, param_names, (uint64_t*)&initial_cfg );
	
	/*初始化避障参数*/
		//机架轴距(cm)
		avCfg.wheelbase[0] = 45;
		//避障策略
		avCfg.AvoidanceMode[0] = AvModeFlag_Ena | AvModeFlag_DownsideAv_Ena | 
															AvModeFlag_UpsideAcav_Ena;
		//避障距离（离障碍物距离cm）
		avCfg.AvoidanceDist[0] = 250;
		//围栏开关
		avCfg.fenceEnable[0] = FenceEnable_CpxFenceFlag;
		//围栏类型
		avCfg.fenceType[0] = 0;
		//围栏动作
		avCfg.fenceAction[0] = 0;
		//圆形围栏半径
		avCfg.fenceRadius[0] = 50;
		//最大最小高度
		avCfg.fenceMinAlt[0] = -10;
		avCfg.fenceMaxAlt[0] = 100;
		
		SName Avparam_names[] = {
			//机架轴距(cm)
			"Av_WheelBase" ,	
			//避障策略
			"Av_Mode" ,	
			//避障距离（离障碍物距离cm）
			"Av_Distance" ,	
			
			//围栏开关
			"FENCE_ENABLE" ,
			//围栏类型
			"FENCE_TYPE" ,
			//围栏动作
			"FENCE_ACTION" ,
			//圆形围栏半径
			"FENCE_RADIUS" ,
			//最大最小高度
			"FENCE_ALT_MIN" ,
			"FENCE_ALT_MAX" ,
		};

		MAV_PARAM_TYPE Avparam_types[] = {
			//机架轴距(cm)
			MAV_PARAM_TYPE_REAL32 ,	
			//避障策略
			MAV_PARAM_TYPE_UINT32 ,	
			//避障距离（离障碍物距离cm）
			MAV_PARAM_TYPE_REAL32 ,

			//围栏开关
			MAV_PARAM_TYPE_UINT32 ,
			//围栏类型
			MAV_PARAM_TYPE_UINT32 ,
			//围栏动作
			MAV_PARAM_TYPE_UINT32 ,
			//圆形围栏半径
			MAV_PARAM_TYPE_REAL32 ,
			//最大最小高度
			MAV_PARAM_TYPE_REAL32 ,
			MAV_PARAM_TYPE_REAL32 ,			
		};
		ParamGroupRegister( "Avoidance", 7, sizeof(avCfg)/8, Avparam_types, Avparam_names, (uint64_t*)&avCfg );
	/*初始化避障参数*/
}