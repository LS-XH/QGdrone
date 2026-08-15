#include "Basic.hpp"
#include "FreeRTOS.h"
#include "drv_USB.hpp"
#include "drv_Uart1.hpp"
#include "Sensors.hpp"
#include "MeasurementSystem.hpp"
#include "mavlink.h"
#include "Commulink.hpp"
#include "StorageSystem.hpp"
#include "ControlSystem.hpp"
#include "Avoidance.hpp"

float debug_test[30];

static void Debug_task(void* pvParameters)
{
	while(1)
	{
		uint8_t port_id = 0;
		const Port* port = get_CommuPort(port_id);
		if( port==0 || port->write==0 )
		{
			os_delay(1);
			continue;
		}
		mavlink_message_t msg_sd;
		
		vector3<double> tvel(1,0,0);
		double dis;
		get_AvLineDistanceFlu(&dis,tvel,200);
		
//		if( mavlink_lock_chan(1,0.01) )
//		{
//			mavlink_msg_vfr_hud_pack_chan(
//				1 ,	//system id
//				MAV_COMP_ID_AUTOPILOT1 ,	//component id
//				port_id , 	//chan
//				&msg_sd ,
//				0,0,0,0,0,0
//			);
//			mavlink_msg_to_send_buffer(Write_Uart1, 
//																 Lock_Uart1,
//																 Unlock_Uart1,
//																 &msg_sd, 0, -1);
//			mavlink_unlock_chan(1);
//		}
		extern int aaw[16];
		IMU_Sensor imu;
		GetMagnetometer( 2, &imu, 0.1 );
		if( mavlink_lock_chan(port_id,0.01) )
		{
			vector3<double> vec;
			IMU_Sensor sensor;
			Position_Sensor_Data pos;
			get_AccelerationNC_filted(&vec);
			GetPositionSensorData(1,&pos);
			GetMagnetometer( 2, &sensor );
			mavlink_msg_debug_vect_pack_chan( 
				get_CommulinkSysId() ,	//system id
				get_CommulinkCompId() ,	//component id
				port_id , 	//chan
				&msg_sd ,
				"1" ,	//name
				TIME::get_System_Run_Time() * 1e6 , 	//boot ms
				debug_test[0] ,
				debug_test[1] ,
				debug_test[2] );
			mavlink_msg_to_send_buffer(port->write, 
																 port->lock,
																 port->unlock,
																 &msg_sd, 0, -1);
					
			
			Position_Sensor_Data pos2;
			GetPositionSensorData(default_rtk_sensor_index,&pos);
			GetPositionSensorData(internal_baro_sensor_index,&pos2);
			mavlink_msg_debug_vect_pack_chan( 
				get_CommulinkSysId() ,	//system id
				get_CommulinkCompId() ,	//component id
				port_id , 	//chan
				&msg_sd ,
				"2" ,	//name
				TIME::get_System_Run_Time() * 1e6 , 	//boot ms
				sensor.data_raw.x ,
				sensor.data_raw.y ,
				sensor.data_raw.z );
			mavlink_msg_to_send_buffer(port->write, 
																 port->lock,
																 port->unlock,
																 &msg_sd, 0, -1);
																 
			get_AccelerationENU_Ctrl(&vec);
			mavlink_msg_debug_vect_pack_chan( 
				get_CommulinkSysId() ,	//system id
				get_CommulinkCompId() ,	//component id
				port_id , 	//chan
				&msg_sd ,
				"3" ,	//name
				TIME::get_System_Run_Time() * 1e6 , 	//boot ms
				debug_test[13] ,
				debug_test[14] ,
				debug_test[15] );
			mavlink_msg_to_send_buffer(port->write, 
																 port->lock,
																 port->unlock,
																 &msg_sd, 0, -1);
								
GetPositionSensorData(15,&pos);								
		mavlink_msg_debug_vect_pack_chan( 
				get_CommulinkSysId() ,	//system id
				get_CommulinkCompId() ,	//component id
				port_id , 	//chan
				&msg_sd ,
				"4" ,	//name
				TIME::get_System_Run_Time() * 1e6 , 	//boot ms
				debug_test[13],
				debug_test[14] ,
				pos.position.y*0.01 );
			mavlink_msg_to_send_buffer(port->write, 
																 port->lock,
																 port->unlock,
																 &msg_sd, 0, -1);
//																 
//			GetMagnetometer( 2, &sensor );
//			mavlink_msg_debug_vect_pack_chan( 
//				1 ,	//system id
//				MAV_COMP_ID_AUTOPILOT1 ,	//component id
//				port_id , 	//chan
//				&msg_sd ,
//				"Mag" ,	//name
//				TIME::get_System_Run_Time() * 1e6 , 	//boot ms
//				debug_test[21] ,
//				debug_test[22] ,
//				debug_test[24] );
//			mavlink_msg_to_send_buffer(port->write, 
//																 port->lock,
//																 port->unlock,
//																 &msg_sd, 0, -1);
			mavlink_unlock_chan(port_id);
		}
		
		bool unLocked;
		is_Attitude_Control_Enabled(&unLocked);
		if( unLocked )
		{
			//uint8_t mt_count = get_MainMotorCount();
			double logbuf[8];
			logbuf[0] = TIM4->CCR3;
			logbuf[1] = TIM4->CCR4;
			logbuf[2] = TIM8->CCR1;
			logbuf[3] = TIM8->CCR2;
			logbuf[4] = TIM3->CCR1;
			logbuf[5] = TIM3->CCR2;
			logbuf[6] = TIM15->CCR1;
			logbuf[7] = TIM15->CCR2;
			SDLog_Msg_DebugVect( "dianji", logbuf, 8 );
			
			logbuf[0] = debug_test[0];
			logbuf[1] = debug_test[1];
			logbuf[2] = debug_test[2];
			SDLog_Msg_DebugVect( "lingP", logbuf, 3 );
			
			
//			//时间
//			RTC_TimeStruct rtc = Get_RTC_Time();
//			//姿态角
//			Quaternion airframe_quat;
//			get_AirframeY_quat(&airframe_quat);
//			airframe_quat.Enu2Ned();
//			double heading = airframe_quat.getYaw();
//			if( heading < 0 )
//				heading = 2*Pi + heading;
//			//经纬度
//			double lat=0, lon=0;
//			double gpsAlt = 0;
//			Position_Sensor gps_sensor;
//			if( GetPositionSensor( default_gps_sensor_index, &gps_sensor ) )
//			{
//				lat = gps_sensor.data.position_Global.x;
//				lon = gps_sensor.data.position_Global.y;
//				gpsAlt = gps_sensor.data.position_Global.z;
//			}
//			//对地高
//			static double homeZ = 0;
//			getHomeLocalZ( &homeZ, 0, 0.01 );
//			vector3<double> position;
//			get_Position_Ctrl(&position);
//			double heightAboveGround = position.z - homeZ;
//			
//			char txt[200];
//			size_t len = sprintf( txt, "%4d/%2d/%2d, %2d:%2d:%2.3f, %.2f, %.2f, %.2f, %.7f, %.7f, %.1f, %.1f\r\n",
//				rtc.Year, rtc.Month, rtc.Date,
//				rtc.Hours, rtc.Minutes, rtc.Seconds_f,
//				rad2degree(airframe_quat.getRoll()),
//				rad2degree(airframe_quat.getPitch()),
//				rad2degree(heading),
//				lat, lon,
//				gpsAlt,
//				heightAboveGround*0.01
//			);
//			SDLog_Txt1( txt, len );
		}
//		/*姿态*/
//			uint8_t buf[30];
//			uint8_t i = 0;
//			uint8_t checksum = 0;
//			buf[i] = 0xaa;
//			checksum += buf[i++];
//			buf[i] = 0xaa;
//			checksum += buf[i++];
//			buf[i] = 0x01;
//			checksum += buf[i++];
//			buf[i] = 6;
//			checksum += buf[i++];
//			
//			int16_t temp = debug_test[0]*5729.578f;
//			buf[i] = temp >> 8;
//			checksum += buf[i++];
//			buf[i] = temp;
//			checksum += buf[i++];
//			
//			temp = debug_test[1]*5729.578f;
//			buf[i] = temp >> 8;
//			checksum += buf[i++];
//			buf[i] = temp;
//			checksum += buf[i++];
//			
//			temp = debug_test[2]*5729.578f;
//			buf[i] = temp >> 8;
//			checksum += buf[i++];
//			buf[i] = temp;
//			checksum += buf[i++];
//			
//			buf[i++] = checksum;
//			Write_USBD_VCOM( buf , i );
//		/*姿态*/
//		
//		/*其它*/
//			checksum = i = 0;
//			buf[i] = 0xaa;
//			checksum += buf[i++];
//			buf[i] = 0xaa;
//			checksum += buf[i++];
//			buf[i] = 0x02;
//			checksum += buf[i++];
//			buf[i] = 18;
//			checksum += buf[i++];
//			
//			temp = debug_test[4];
//			buf[i] = temp >> 8;
//			checksum += buf[i++];
//			buf[i] = temp;
//			checksum += buf[i++];
//			
//			temp = debug_test[5];
//			buf[i] = temp >> 8;
//			checksum += buf[i++];
//			buf[i] = temp;
//			checksum += buf[i++];
//			
//			temp = debug_test[6];
//			buf[i] = temp >> 8;
//			checksum += buf[i++];
//			buf[i] = temp;
//			checksum += buf[i++];
//			
//			temp = debug_test[7];
//			buf[i] = temp >> 8;
//			checksum += buf[i++];
//			buf[i] = temp;
//			checksum += buf[i++];
//			
//			temp = debug_test[8];
//			buf[i] = temp >> 8;
//			checksum += buf[i++];
//			buf[i] = temp;
//			checksum += buf[i++];
//			
//			temp = debug_test[9];
//			buf[i] = temp >> 8;
//			checksum += buf[i++];
//			buf[i] = temp;
//			checksum += buf[i++];
//			
//			temp = debug_test[10];
//			buf[i] = temp >> 8;
//			checksum += buf[i++];
//			buf[i] = temp;
//			checksum += buf[i++];
//			
//			temp = debug_test[11];
//			buf[i] = temp >> 8;
//			checksum += buf[i++];
//			buf[i] = temp;
//			checksum += buf[i++];
//			
//			temp = debug_test[12];
//			buf[i] = temp >> 8;
//			checksum += buf[i++];
//			buf[i] = temp;
//			checksum += buf[i++];
//			
//			buf[i++] = checksum;
//			Write_USBD_VCOM( buf , i );
//		/*其它*/
		
		os_delay(0.01);
	}
}

void init_Debug()
{
	xTaskCreate( Debug_task , "Debug" ,1000,NULL,SysPriority_UserTask,NULL);
}