#include "drv_CRSF.hpp"
#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "StorageSystem.hpp"
#include "ReceiverBackend.hpp"

struct DriverInfo
{
	uint32_t param;
	Port port;
};

#define CRSF_Type_RC_Channels		(0x16)

typedef struct
{   
	uint8_t device_addr; 
    uint8_t frame_size;  
    uint8_t type;        
	union {
		uint8_t data[22];
		struct __PACKED
		{
			uint16_t channel_1 	: 11;
			uint16_t channel_2 	: 11;
			uint16_t channel_3 	: 11;
			uint16_t channel_4 	: 11;
			uint16_t channel_5 	: 11;
			uint16_t channel_6 	: 11;
			uint16_t channel_7 	: 11;
			uint16_t channel_8 	: 11;
			uint16_t channel_9 	: 11;
			uint16_t channel_10 : 11;
			uint16_t channel_11 : 11;
			uint16_t channel_12 : 11;
			uint16_t channel_13 : 11;
			uint16_t channel_14 : 11;
			uint16_t channel_15 : 11;
			uint16_t channel_16 : 11;
		};
	};
}__PACKED _CRSF_RC_Channels;

_CRSF_RC_Channels crsf_rc_channels;
float rc_buf[16];

/* CRC8 implementation with polynom = x7+ x6+ x4+ x2+ x0 (0xD5) */ 
const unsigned char crc8tab[256] = { 
0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D, 
0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F, 
0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9, 
0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B, 
0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0, 
0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2, 
0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44, 
0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16, 
0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92, 
0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0, 
0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36, 
0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64, 
0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F, 
0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D, 
0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB, 
0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9}; 
static uint8_t crc8(const uint8_t * ptr, uint8_t len) 
{ 
	uint8_t crc = 0; 
	for (uint8_t i=0; i<len; i++) { 
		crc = crc8tab[crc ^ *ptr++]; 
	} 
	return crc;
}

static void CRSF_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	//_CRSF_RC_Channels crsf_rc_channels;
	uint8_t rc_counter = 0;
	
	while(1)
	{	
		uint8_t rdata;
		if( driver_info.port.read( &rdata, 1, 2, 0.5 ) ) 
		{
			if(rc_counter == 0)
			{	//目标设备 0xC8为飞控
				if(rdata == 0xC8)
				{
					crsf_rc_channels.device_addr = rdata;
					rc_counter++;
				}
				else 
				{
					rc_counter = 0;
				}
			}
			else if(rc_counter == 1)
			{	//发送字节 
				crsf_rc_channels.frame_size = rdata;
				rc_counter++;
			}
			else if(rc_counter == 2)
			{	//包类型 只处理摇杆数据
				if(rdata == CRSF_Type_RC_Channels)
				{
					crsf_rc_channels.type = rdata;
					rc_counter++;
				}
				else 
				{
					rc_counter = 0;
				}
			}
			else if(rc_counter < 25)
			{	
				crsf_rc_channels.data[ rc_counter - 3 ] = rdata;
				rc_counter++;
			}
			else 
			{
				uint8_t crc = crc8((const uint8_t *)&crsf_rc_channels.type, crsf_rc_channels.frame_size - 1);
				
				if(crc == rdata)
				{	//上传遥控数据
					rc_buf[0] = (crsf_rc_channels.channel_1 - 992) * 5 / 8 + 1500 - 989;
					rc_buf[1] = (crsf_rc_channels.channel_2 - 992) * 5 / 8 + 1500 - 989;
					rc_buf[2] = (crsf_rc_channels.channel_3 - 992) * 5 / 8 + 1500 - 989;
					rc_buf[3] = (crsf_rc_channels.channel_4 - 992) * 5 / 8 + 1500 - 989;
					rc_buf[4] = (crsf_rc_channels.channel_5 - 992) * 5 / 8 + 1500 - 989;
					rc_buf[5] = (crsf_rc_channels.channel_6 - 992) * 5 / 8 + 1500 - 989;
					rc_buf[6] = (crsf_rc_channels.channel_7 - 992) * 5 / 8 + 1500 - 989;
					rc_buf[7] = (crsf_rc_channels.channel_8 - 992) * 5 / 8 + 1500 - 989;
					rc_buf[8] = (crsf_rc_channels.channel_9 - 992) * 5 / 8 + 1500 - 989;
					rc_buf[9] = (crsf_rc_channels.channel_10 - 992) * 5 / 8 + 1500 - 989;
					rc_buf[10] = (crsf_rc_channels.channel_11 - 992) * 5 / 8 + 1500 - 989;
					rc_buf[11] = (crsf_rc_channels.channel_12 - 992) * 5 / 8 + 1500 - 989;
					rc_buf[12] = (crsf_rc_channels.channel_13 - 992) * 5 / 8 + 1500 - 989;
					rc_buf[13] = (crsf_rc_channels.channel_14 - 992) * 5 / 8 + 1500 - 989;
					rc_buf[14] = (crsf_rc_channels.channel_15 - 992) * 5 / 8 + 1500 - 989;
					rc_buf[15] = (crsf_rc_channels.channel_16 - 992) * 5 / 8 + 1500 - 989;
					
					for(int i = 0; i < 16; i++)
					{
						rc_buf[i] *= 0.09784735812133072407045009784736; 
					}
					
					ReceiverUpdate( "Crsf", true, rc_buf, 16, 0.01 );
				}
				
				rc_counter = 0;
			}
		}
	}
}

static bool CRSF_DriverInit( Port port, uint32_t param )
{
	//波特率416000
	port.SetBaudRate( 416000, 2, 2 );
	
	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	xTaskCreate( CRSF_Server, "Crsf", 1024, (void*)driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_CRSF()
{
	//接收机注册
	ReceiverRegister("Crsf");
	
	PortFunc_Register( 50, CRSF_DriverInit );
}