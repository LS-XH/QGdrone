#include "Basic.hpp"
#include "drv_IICTFMini_Av.hpp"
#include "drv_ExtIIC.hpp"
#include "Avoidance.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"

typedef struct
{
	uint16_t header;
	uint16_t Dist;	//Dist距离（30-1200cm）
	uint16_t Strength;	//Strength信号强度（20-2000可信）
	uint16_t temp;
	uint8_t checksum;
}__PACKED _TfMini;

static void TFMiniAv_Server(void* pvParameters)
{
	Aligned_DMABuf uint8_t tx_buf[32];
	Aligned_DMABuf uint8_t rx_buf[32];
	
	uint32_t sensor_key1 = registere_AvTarget();
	uint32_t sensor_key2 = registere_AvTarget();
	uint32_t sensor_key3 = registere_AvTarget();
	uint32_t sensor_key4 = registere_AvTarget();
	
	tx_buf[0] = 0x5a;
	tx_buf[1] = 0x05;
	tx_buf[2] = 0x00;
	tx_buf[3] = 0x01;
	tx_buf[4] = 0x60;
	
	//获取避障参数
	const AvoidanceCfg* avCfg = getAvCfg();
	
	bool res;
	_TfMini* data;
reTry:
	while(1)
	{	
		ExtIIC_SendAddr7(0x10, tx_buf,5 );
		ExtIIC_SendAddr7(0x11, tx_buf,5 );
		ExtIIC_SendAddr7(0x12, tx_buf,5 );
		ExtIIC_SendAddr7(0x13, tx_buf,5 );
		
		res = ExtIIC_ReceiveAddr7(0x10, rx_buf, 9 );
		if(res) {
			data = (_TfMini*)rx_buf;
			vector3<double> dis;
			dis.x = data->Dist;
			if( data->Dist>avCfg->wheelbase[0] && data->Dist<65500 )
				set_AvTargetPoint3dPoint_RelativeFlu( sensor_key1, dis );
			else
				set_AvTargetInavailable(sensor_key1);
		}
		else
			set_AvTargetInavailable(sensor_key1);
		
		res = ExtIIC_ReceiveAddr7(0x11, rx_buf, 9 );
		if(res) {
			data = (_TfMini*)rx_buf;
			vector3<double> dis;
			dis.y = data->Dist;
			if( data->Dist>avCfg->wheelbase[0] && data->Dist<65500 )
				set_AvTargetPoint3dPoint_RelativeFlu( sensor_key2, dis );
			else
				set_AvTargetInavailable(sensor_key2);
		}
		else
			set_AvTargetInavailable(sensor_key2);
		
		res = ExtIIC_ReceiveAddr7(0x12, rx_buf, 9 );
		if(res) {
			data = (_TfMini*)rx_buf;
			vector3<double> dis;
			dis.x = -data->Dist;
			if( data->Dist>avCfg->wheelbase[0] && data->Dist<65500 )
				set_AvTargetPoint3dPoint_RelativeFlu( sensor_key3, dis );
			else
				set_AvTargetInavailable(sensor_key3);
		}
		else
			set_AvTargetInavailable(sensor_key3);
		
		res = ExtIIC_ReceiveAddr7(0x13, rx_buf, 9 );
		if(res) {
			data = (_TfMini*)rx_buf;
			vector3<double> dis;
			dis.y = -data->Dist;
			if( data->Dist>avCfg->wheelbase[0] && data->Dist<65500 )
				set_AvTargetPoint3dPoint_RelativeFlu( sensor_key4, dis );
			else
				set_AvTargetInavailable(sensor_key4);
		} 
		else
			set_AvTargetInavailable(sensor_key4);
		
		os_delay(0.05);
	}
}

static bool I2C_TFMiniAv_DriverInit()
{
	return true;
}
static bool I2C_TFMiniAv_DriverRun()
{
	xTaskCreate( TFMiniAv_Server, "IICTFMiniAv", 500, NULL, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_IICTFMiniAv()
{
	I2CFunc_Register( 96, I2C_TFMiniAv_DriverInit, I2C_TFMiniAv_DriverRun );
}
