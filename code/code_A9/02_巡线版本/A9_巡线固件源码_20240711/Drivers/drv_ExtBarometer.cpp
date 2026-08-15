#include "Basic.hpp"
#include "drv_ExtBarometer.hpp"
#include "drv_ExtIIC.hpp"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"
#include "Commulink.hpp"

//缓冲区
Static_AXIDMABuf uint8_t tx_buf[12];
__attribute__ ((aligned (4))) Static_AXIDMABuf uint8_t rx_buf[32];	
static uint32_t sensor_key = 0;

/*SPL06*/
	#define SPLO6_ADDR 0x77
	struct SPL06_COEFFICIENTS
	{
		int16_t c0;	int16_t c1;
		int32_t c00;	int32_t c10;	int16_t c01;	int16_t c11;
		int16_t c20;	int16_t c21;	
		int16_t c30;
		double KP;	double KT;
	};
		
	static void spl06_pressure_rateset( SPL06_COEFFICIENTS* SPL06_coefficients, uint8_t u8SmplRate, uint8_t u8OverSmpl)
	{
		uint8_t reg = 0;
		switch(u8SmplRate)
		{
			case 2:
				reg |= (1<<4);
				break;
			case 4:
				reg |= (2<<4);
				break;
			case 8:
				reg |= (3<<4);
				break;
			case 16:
				reg |= (4<<4);
				break;
			case 32:
				reg |= (5<<4);
				break;
			case 64:
				reg |= (6<<4);
				break;
			case 128:
				reg |= (7<<4);
				break;
			case 1:
			default:
				break;
		}
		switch(u8OverSmpl)
		{
			case 2:
				reg |= 1;
				SPL06_coefficients->KP = 1.0 / 1572864;
				break;
			case 4:
				reg |= 2;
				SPL06_coefficients->KP = 1.0 / 3670016;
				break;
			case 8:
				reg |= 3;
				SPL06_coefficients->KP = 1.0 / 7864320;
				break;
			case 16:
				SPL06_coefficients->KP = 1.0 / 253952;
				reg |= 4;
				break;
			case 32:
				SPL06_coefficients->KP = 1.0 / 516096;
				reg |= 5;
				break;
			case 64:
				SPL06_coefficients->KP = 1.0 / 1040384;
				reg |= 6;
				break;
			case 128:
				SPL06_coefficients->KP = 1.0 / 2088960;
				reg |= 7;
				break;
			case 1:
			default:
				SPL06_coefficients->KP = 1.0 / 524288;
				break;
		}
		tx_buf[0] = 0x06;
		tx_buf[1] = reg;
		ExtIIC_SendAddr7(SPLO6_ADDR, tx_buf, 2);
		os_delay(0.1);
		
		//Pressure result bit-shift,Must be set to '1' when the oversampling rate is  >8  times.
		if(u8OverSmpl > 8)
		{
			tx_buf[0] = 0x09;
			ExtIIC_SendReceiveAddr7(SPLO6_ADDR, tx_buf, 1, rx_buf, 1);

			tx_buf[0] = 0x09;
			tx_buf[1] = (1<<2)|rx_buf[0];
			ExtIIC_SendAddr7(SPLO6_ADDR, tx_buf, 2);
			os_delay(0.1);
		}
	}
	static void spl06_temperature_rateset( SPL06_COEFFICIENTS* SPL06_coefficients, uint8_t u8SmplRate, uint8_t u8OverSmpl)
	{
			uint8_t reg = 0;
			switch(u8SmplRate)
			{
				case 2:
					reg |= (1<<4);
					break;
				case 4:
					reg |= (2<<4);
					break;
				case 8:
					reg |= (3<<4);
					break;
				case 16:
					reg |= (4<<4);
					break;
				case 32:
					reg |= (5<<4);
					break;
				case 64:
					reg |= (6<<4);
					break;
				case 128:
					reg |= (7<<4);
					break;
				case 1:
				default:
					break;
			}
			switch(u8OverSmpl)
			{
				case 2:
					reg |= 1;
					SPL06_coefficients->KT = 1.0f / 1572864;
					break;
				case 4:
					reg |= 2;
					SPL06_coefficients->KT = 1.0f / 3670016;
					break;
				case 8:
					reg |= 3;
					SPL06_coefficients->KT = 1.0f / 7864320;
					break;
				case 16:
					SPL06_coefficients->KT = 1.0f / 253952;
					reg |= 4;
					break;
				case 32:
					SPL06_coefficients->KT = 1.0f / 516096;
					reg |= 5;
					break;
				case 64:
					SPL06_coefficients->KT = 1.0f / 1040384;
					reg |= 6;
					break;
				case 128:
					SPL06_coefficients->KT = 1.0f / 2088960;
					reg |= 7;
					break;
				case 1:
				default:
					SPL06_coefficients->KT = 1.0f / 524288;
					break;
			}
			tx_buf[0] = 0x07;
			tx_buf[1] = (1<<7) | reg;
			ExtIIC_SendAddr7(SPLO6_ADDR, tx_buf, 2);
			os_delay(0.01);
			//Temperature result bit-shift,Must be set to '1' when the oversampling rate is  >8  times.
			if(u8OverSmpl > 8)
			{
				tx_buf[0] = 0x09;
				ExtIIC_SendReceiveAddr7(SPLO6_ADDR, tx_buf, 1, rx_buf, 1);
				
				tx_buf[0] = 0x09;
				tx_buf[1] = (1<<3)|rx_buf[0];
				ExtIIC_SendAddr7(SPLO6_ADDR, tx_buf, 2);
				os_delay(0.01);
			}
		}

	static void ExtSPL06_Server(void* pvParameters)
	{
		SPL06_COEFFICIENTS SPL06_coefficients;
		
		//操作结果
		bool res;
		
		//读取ID
		tx_buf[0] = 0x0D;
		res = ExtIIC_SendReceiveAddr7( SPLO6_ADDR, tx_buf, 1, rx_buf, 1 );
		os_delay(0.1);
		if(!res || rx_buf[0]!=0x10)
			return;
		
		//复位
		tx_buf[0] = 0x0c;
		tx_buf[1] = (1<<7) | 0b1001;
		res = ExtIIC_SendAddr7(SPLO6_ADDR, tx_buf, 2);
		os_delay(0.5);
		if(!res)
			return;	
	
		//配置传感器
		spl06_pressure_rateset( &SPL06_coefficients, 64 , 32 );//pressure 64 samples per sec , 32 times over sampling		
		spl06_temperature_rateset( &SPL06_coefficients, 128 , 2 );//temperature 128 samples per sec , 2 times over sampling		
		tx_buf[0] = 0x08;
		tx_buf[1] = 0b111;  //Continuous pressure and  temperature  measur ement
		res = ExtIIC_SendAddr7(SPLO6_ADDR, tx_buf, 2);	
		os_delay(0.1);		
		if(!res)
			return;
		
		//读取校准系数		
		tx_buf[0] = 0x10;
		res = ExtIIC_SendReceiveAddr7( SPLO6_ADDR, tx_buf, 1, rx_buf, 18 );
		if(!res)
			return;
		SPL06_coefficients.c0 = ( rx_buf[0] << 4 ) | ( rx_buf[1] >> 4 );
		SPL06_coefficients.c0 = ( SPL06_coefficients.c0 & 0x0800 ) ? (0xF000|SPL06_coefficients.c0) : SPL06_coefficients.c0;
		SPL06_coefficients.c1 = ( (rx_buf[1] & 0xf) << 8 ) | ( rx_buf[2] );
		SPL06_coefficients.c1 = ( SPL06_coefficients.c1 & 0x0800 ) ? (0xF000|SPL06_coefficients.c1) : SPL06_coefficients.c1;
		SPL06_coefficients.c00 = ( rx_buf[3] << 12 ) | ( rx_buf[4] << 4 ) | ( rx_buf[5] >> 4 );
		SPL06_coefficients.c00 = ( SPL06_coefficients.c00 & 0x080000 ) ? (0xFFF00000|SPL06_coefficients.c00) : SPL06_coefficients.c00;
		SPL06_coefficients.c10 = ( (rx_buf[5] & 0xf) << 16 ) | ( rx_buf[6] << 8 ) | ( rx_buf[7] >> 0 );
		SPL06_coefficients.c10 = ( SPL06_coefficients.c10 & 0x080000 ) ? (0xFFF00000|SPL06_coefficients.c10) : SPL06_coefficients.c10;
		SPL06_coefficients.c01 = ( rx_buf[8] << 8 ) | ( rx_buf[9] << 0 );
		SPL06_coefficients.c11 = ( rx_buf[10] << 8 ) | ( rx_buf[11] << 0 );
		SPL06_coefficients.c20 = ( rx_buf[12] << 8 ) | ( rx_buf[13] << 0 );
		SPL06_coefficients.c21 = ( rx_buf[14] << 8 ) | ( rx_buf[15] << 0 );
		SPL06_coefficients.c30 = ( rx_buf[16] << 8 ) | ( rx_buf[17] << 0 );		
		//注册传感器
		sensor_key = PositionSensorRegister( external_baro_sensor_index , \
														"SPL06_Ext" ,\
														Position_Sensor_Type_RelativePositioning , \
														Position_Sensor_DataType_s_z , \
														Position_Sensor_frame_ENU , \
														0.05 , //延时
														0 ,	//xy信任度
														50 //z信任度
														) ;
		if( sensor_key==0 )
			return;
		
	ExtSPL06Detected:
		while(1)
		{
			os_delay(0.033);
			
			int32_t buf32[2];
			
			tx_buf[0] = 0x00;
			res = ExtIIC_SendReceiveAddr7( SPLO6_ADDR, tx_buf, 1, rx_buf, 6 );
			if(!res)
			{
				PositionSensorUnRegister(external_baro_sensor_index,sensor_key);
				return;
			}
			struct __SPL06_Data
			{
				unsigned int pressure:24 ;
				unsigned int temperature:24 ;
			}__PACKED;
			__SPL06_Data* datap = (__SPL06_Data*)rx_buf;
			buf32[0] = __REV( datap->pressure ) >> 8;		
			buf32[1] = __REV( datap->temperature ) >> 8;	
			
			buf32[0] = ( buf32[0] & 0x800000 ) ? (0xFF000000|buf32[0]) : buf32[0];
			buf32[1] = ( buf32[1] & 0x800000 ) ? (0xFF000000|buf32[1]) : buf32[1];
			
			double fPsc = buf32[0] * SPL06_coefficients.KP;
			double fTsc = buf32[1] * SPL06_coefficients.KT;
			double qua2 = SPL06_coefficients.c10 + fPsc * (SPL06_coefficients.c20 + fPsc* SPL06_coefficients.c30);
			double qua3 = fTsc * fPsc * (SPL06_coefficients.c11 + fPsc * SPL06_coefficients.c21);
			
			double pressure = SPL06_coefficients.c00 + fPsc * qua2 + fTsc * SPL06_coefficients.c01 + qua3;
			double temperature = SPL06_coefficients.c0*0.5 + SPL06_coefficients.c1*fTsc;
			//更新传感器数据
			if( pressure > 0 )
			{
				double velx = get_VelocityENU_Ctrl_x();
				double vely = get_VelocityENU_Ctrl_y();
				double trust = 200;// + constrain( 0.5*safe_sqrt( velx*velx + vely*vely ), 150.0 );
				double LTtrust = 50;				
				
				vector3<double> position;
				position.z = 4430000 * ( 1.0 - pow( pressure / 101325.0 , 1.0 / 5.256 ) );
				PositionSensorUpdatePosition( external_baro_sensor_index,sensor_key, position , 
					true , //available
					-1, //delay
					-1, //xy trust
					trust,	//z trust
					0,	//addition inf
					-1,	//xy LTtrust
					LTtrust	//z LTtrust
				);
				PositionSensorUpdatePosition( external_baro_sensor_index,sensor_key, position , true , -1, -1,trust );
			}
		}
	}
/*SPL06*/

/*MS5803*/
	#define MS5803_ADDR 0x76
	static void ExtMS5803_Server(void* pvParameters)
	{
		//传感器类型
		//0-MS5803
		//10-MS5837-02
		//11-MS5837-30
		uint8_t type = 0;
		//校准系数
		uint16_t C[8];
		//操作结果
		bool res;
		/*初始化*/
			//复位
			tx_buf[0] = 0x1e;
			res = ExtIIC_SendAddr7(MS5803_ADDR, tx_buf, 1);
			os_delay(0.5);
			if(!res)
				return;	
			
			//读取校准系数
			for( uint8_t i = 0; i < 7; ++i )
			{
				tx_buf[0] = 0xA0 + i*2;
				res = ExtIIC_SendReceiveAddr7( MS5803_ADDR, tx_buf, 1, rx_buf, 2 );
				if(!res)
					return;
				C[i] = (rx_buf[0]<<8) | rx_buf[1];
			}
			
			//判断传感器类型
			tx_buf[0] = 0xA0 + 7*2;
			res = ExtIIC_SendReceiveAddr7( MS5803_ADDR, tx_buf, 1, rx_buf, 2 );
			if( res )
				type = 0;
			else
			{
				uint8_t id = (C[0]>>5) & 0x7f;
				if( id==0 || id==0x15 )
					type = 10;
				else if( id==0x1a )
					type = 11;
				else
					return;
			}
			
			//注册传感器
			sensor_key = PositionSensorRegister( external_baro_sensor_index , \
																						"MS5803_Ext" ,\
																						Position_Sensor_Type_RelativePositioning , \
																						Position_Sensor_DataType_s_z , \
																						Position_Sensor_frame_ENU , \
																						0.05 , //延时
																						0 ,	//xy信任度
																						50 //z信任度
																						);
			if( sensor_key==0 )
				return;
		/*初始化*/
															
		switch(type)
		{
			case 0:
			{	//MS5803
				while(1)
				{
					//D2转换
					tx_buf[0] = 0x58;
					res = ExtIIC_SendAddr7(MS5803_ADDR, tx_buf, 1);
					if(!res)
					{
						PositionSensorUnRegister(external_baro_sensor_index,sensor_key);
						return;
					}
					os_delay(0.01);
					//读取D2
					tx_buf[0] = 0x00;
					res = ExtIIC_SendReceiveAddr7( MS5803_ADDR, tx_buf, 1, rx_buf, 3 );
					int64_t D2 = (rx_buf[0]<<16) | (rx_buf[1]<<8) | (rx_buf[2]<<0);
					if(!res)
					{
						PositionSensorUnRegister(external_baro_sensor_index,sensor_key);
						return;
					}
					
					//D1转换
					tx_buf[0] = 0x48;
					res = ExtIIC_SendAddr7(MS5803_ADDR, tx_buf, 1);
					if(!res)
					{
						PositionSensorUnRegister(external_baro_sensor_index,sensor_key);
						return;
					}
					os_delay(0.01);
					//读取D1
					tx_buf[0] = 0x00;
					res = ExtIIC_SendReceiveAddr7( MS5803_ADDR, tx_buf, 1, rx_buf, 3 );
					int64_t D1 = (rx_buf[0]<<16) | (rx_buf[1]<<8) | (rx_buf[2]<<0);
					if(!res)
					{
						PositionSensorUnRegister(external_baro_sensor_index,sensor_key);
						return;
					}
					
					// calcualte 2st order pressure and temperature
					double T2, OFF2, SENS2;
					double dT = D2 - ((double)C[5])*(1<<8);
					double TEMP = 2000 + (dT*(double)C[6])/(1<<23);
					if( TEMP < 2000 )
					{
						T2 = sq(dT) / (1<<31);
						OFF2 = 3*sq(TEMP-2000);
						SENS2 = 7*sq(TEMP-2000)/(1<<3);
						if( TEMP < -1500 )
							SENS2 += 2*sq(TEMP+1500);
					}
					else
					{
						T2 = 0;
						OFF2 = 0;
						SENS2 = 0;
						if( TEMP >= 4500 )
							SENS2 -= sq(TEMP-4500)/(1<<3);
					}
					
					// calcualte 1st order pressure and temperature (MS5607 1st order algorithm)
					double OFF = (double)C[2]*(1<<16) + (dT*(double)C[4])/(1<<7);
					double SENS = (double)C[1]*(1<<15) + (dT*(double)C[3])/(1<<8);
					OFF -= OFF2;
					SENS -= SENS2;
					TEMP -= T2;
					double T = TEMP*0.01;
					double P = ( (D1*SENS)/(1<<21) - OFF ) / (double)(1<<15);
					
					
					//更新传感器数据
					if( P > 0 )
					{
						double velx = get_VelocityENU_Ctrl_x();
						double vely = get_VelocityENU_Ctrl_y();
						double trust = 0 + constrain( 0.5*safe_sqrt( velx*velx + vely*vely ), 150.0 );			
						
						vector3<double> position;
						position.z = 4430000 * ( 1.0 - pow( P / 101325.0 , 1.0 / 5.256 ) );
						PositionSensorUpdatePosition( external_baro_sensor_index,sensor_key , position , true , -1, -1,trust );
					}
				}
				break;
			}
			
			case 10:
			{	//MS5837-02
				while(1)
				{
					//D2转换
					tx_buf[0] = 0x5A;
					res = ExtIIC_SendAddr7(MS5803_ADDR, tx_buf, 1);
					if(!res)
					{
						PositionSensorUnRegister(external_baro_sensor_index,sensor_key);
						return;
					}
					os_delay(0.018);
					//读取D2
					tx_buf[0] = 0x00;
					res = ExtIIC_SendReceiveAddr7( MS5803_ADDR, tx_buf, 1, rx_buf, 3 );
					int64_t D2 = (rx_buf[0]<<16) | (rx_buf[1]<<8) | (rx_buf[2]<<0);
					if(!res)
					{
						PositionSensorUnRegister(external_baro_sensor_index,sensor_key);
						return;
					}
					
					//D1转换
					tx_buf[0] = 0x4A;
					res = ExtIIC_SendAddr7(MS5803_ADDR, tx_buf, 1);
					if(!res)
					{
						PositionSensorUnRegister(external_baro_sensor_index,sensor_key);
						return;
					}
					os_delay(0.018);
					//读取D1
					tx_buf[0] = 0x00;
					res = ExtIIC_SendReceiveAddr7( MS5803_ADDR, tx_buf, 1, rx_buf, 3 );
					int64_t D1 = (rx_buf[0]<<16) | (rx_buf[1]<<8) | (rx_buf[2]<<0);
					if(!res)
					{
						PositionSensorUnRegister(external_baro_sensor_index,sensor_key);
						return;
					}
					
					// calcualte 2st order pressure and temperature
					double T2, OFF2, SENS2;
					double dT = D2 - ((double)C[5])*(1<<8);
					double TEMP = 2000 + (dT*(double)C[6])/(1<<23);
					if( TEMP < 2000 )
					{
						T2 = 11*sq(dT) / ((uint64_t)1<<35);
						OFF2 = 31*sq(TEMP-2000) / (1<<3);
						SENS2 = 63*sq(TEMP-2000)/(1<<5);
					}
					else
					{
						T2 = 0;
						OFF2 = 0;
						SENS2 = 0;
					}
					
					// calcualte 1st order pressure and temperature (MS5607 1st order algorithm)
					double OFF = (double)C[2]*(1<<17) + (dT*(double)C[4])/(1<<6);
					double SENS = (double)C[1]*(1<<16) + (dT*(double)C[3])/(1<<7);
					OFF -= OFF2;
					SENS -= SENS2;
					TEMP -= T2;
					double T = TEMP*0.01;
					double P = ( (D1*SENS)/(1<<21) - OFF ) / (double)(1<<15);
					
					//更新传感器数据
					if( P > 0 )
					{
						double velx = get_VelocityENU_Ctrl_x();
						double vely = get_VelocityENU_Ctrl_y();
						double trust = 50 + constrain( 0.5*safe_sqrt( velx*velx + vely*vely ), 150.0 );			
						
						vector3<double> position;
						position.z = 4430000 * ( 1.0 - pow( P / 101325.0 , 1.0 / 5.256 ) );
						PositionSensorUpdatePosition( external_baro_sensor_index,sensor_key , position , true , -1, -1,trust );
					}
				}
				break;
			}
			
		}
	}
/*MS5803*/
	
	struct External_BaroMeter
	{
		//传感器名称
		SName name;
		
		//iic地址
		unsigned char device_address;
		
		//ID寄存器地址
		unsigned char ID_address;
		//ID掩码
		unsigned char ID_mask;
		//ID
		unsigned char ID;
		
		//传感器服务函数
		void (*server)(void* pvParameters);
	};
	
	static const External_BaroMeter External_BaroMeters[] = 
	{
	/*     名称     , iic地址       , ID 地址 , ID 掩码 , ID   ,  传感器服务函数                 ,  */
		{   "SPL06"   , SPLO6_ADDR    ,  0x0d   , 0xff    , 0x10 ,  ExtSPL06_Server              } , //SPL06
		{   "MS5803"  , MS5803_ADDR   ,  0x00   , 0x00    , 0x00 ,  ExtMS5803_Server              } , //MS5803
	};
	static const uint8_t Supported_External_BaroMeter_Count = sizeof( External_BaroMeters ) / sizeof( External_BaroMeter );
	
	static void ExtBarometer_Server(void* pvParameters)
	{
		//操作结果
		bool res;
	ScanExtBaro:	
		while(1)
		{
			for( int8_t i = 0; i < Supported_External_BaroMeter_Count; ++i )
			{
				vTaskDelay( 0.1*configTICK_RATE_HZ );
				const External_BaroMeter* ext_baro = &External_BaroMeters[i];
				
				//判断传感器ID是否正确
				tx_buf[0] = ext_baro->ID_address;
				res = ExtIIC_SendReceiveAddr7( ext_baro->device_address, tx_buf, 1, rx_buf, 1 );
				if(!res)
					continue;
				if( (rx_buf[0] & ext_baro->ID_mask) != ext_baro->ID )
					continue;
				
				//识别成功
				//进入服务函数
				ext_baro->server(pvParameters);
			}
		}
	}
	
	
static bool I2C_ExtBarometer_DriverInit()
{
	return true;
}
static bool I2C_ExtBarometer_DriverRun()
{
	xTaskCreate( ExtBarometer_Server, "ExtBaro", 1500, NULL, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_ExtBarometer()
{
	I2CFunc_Register( 32, I2C_ExtBarometer_DriverInit, I2C_ExtBarometer_DriverRun );
}
