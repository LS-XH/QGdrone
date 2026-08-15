#include "Basic.hpp"
#include "drv_IIC_Ina226.hpp"
#include "drv_ExtIIC.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "MeasurementSystem.hpp"
#include "Parameters.hpp"
#include "ControlSystem.hpp"
#include "TD4.hpp"

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

#define i2cAddr 0x40

static void Ina226_Server(void* pvParameters)
{
	Aligned_DMABuf uint8_t tx_buf[32];
	Aligned_DMABuf uint8_t rx_buf[32];

	const uint8_t batInd = 0;
	
	//确认ID
	tx_buf[0] = 0xfe;
	ExtIIC_SendReceiveAddr7( i2cAddr, tx_buf,1, rx_buf,2 );
	if( rx_buf[0]!=0b01010100 || rx_buf[1]!=0b01001001 )
		vTaskDelete(0);
	
	uint32_t batId = batteryRegister(batInd);
	if( batId == 0 )
		vTaskDelete(0);
	
	//reset
	tx_buf[0] = 0;
	tx_buf[1] = (1<<7);
	tx_buf[2] = 0;
	ExtIIC_SendAddr7(i2cAddr, tx_buf,3 );
	os_delay(0.2);
	
	//设置连续采样模式
	tx_buf[0] = 0;
	tx_buf[1] = (0b111<<1);
	tx_buf[2] = (0b111<<0);
	ExtIIC_SendAddr7(i2cAddr, tx_buf,3 );
	
	//设置电流校准值
	double resisterR = 0.005;
	double maxCurrent = 100;
	double currentLSB = maxCurrent / 32768;
	uint16_t CAL = 0.00512 / (currentLSB*resisterR);
	tx_buf[0] = 5;
	tx_buf[1] = CAL >> 8;
	tx_buf[2] = CAL * 0xff;
	ExtIIC_SendAddr7(i2cAddr, tx_buf,3 );
	
	TD4_Lite RMPercentFilter1;
	RMPercentFilter1.x1 = -1;
	BatteryCfg batCfg;
	if( ReadParamGroup( "Battery", (uint64_t*)&batCfg, 0 ) != PR_OK )
		vTaskDelete(0);
	
reTry:
	while(1)
	{	
		const double h = 0.05;
		
		//读取电压
		tx_buf[0] = 2;
		ExtIIC_SendReceiveAddr7( i2cAddr, tx_buf,1, rx_buf,2 );
		float batVolt = (uint16_t)( (rx_buf[0]<<8) | rx_buf[1] ) * 0.00125f;
		
		//读取电流
		tx_buf[0] = 4;
		ExtIIC_SendReceiveAddr7( i2cAddr, tx_buf,1, rx_buf,2 );
		float batCurrent = (int16_t)( (rx_buf[0]<<8) | rx_buf[1] ) * currentLSB;
		
		//计算剩余电量
		#define RMPercentFilterP 0.5
		float rmPercent = get_MainBatteryRMPercent( batVolt, &batCfg );
		if( RMPercentFilter1.x1 < 0 )
		{
			RMPercentFilter1.reset();
			RMPercentFilter1.x1 = rmPercent;
		}
		RMPercentFilter1.track4( rmPercent, h, RMPercentFilterP,RMPercentFilterP,RMPercentFilterP,RMPercentFilterP );
		
		//更新电池信息
		batteryUpdate( batInd, batId,
										true,	//available
										batVolt,	//totalVoltRaw
										batVolt, //totalVolt
										batCfg.STVoltage[0],	//stVolt
										batCurrent,	//total current
										0,	//power usage
										batCfg.Capacity[0], //capacity
										RMPercentFilter1.get_x1(),	//percent
										-500,	//temperature
										UINT16_MAX, //cycle count
										0,	//error flags
										0, 0 
									);
		
		os_delay(h);
	}
}

static bool Ina226_DriverInit()
{
	return true;
}
static bool Ina226_DriverRun()
{
	xTaskCreate( Ina226_Server, "Ina226", 812, NULL, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_IIC_Ina226()
{
	I2CFunc_Register( 3, Ina226_DriverInit, Ina226_DriverRun );
}
