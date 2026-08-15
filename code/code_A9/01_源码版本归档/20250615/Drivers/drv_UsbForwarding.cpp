#include "drv_UsbForwarding.hpp"
#include "drv_USB.hpp"
#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"

struct DriverInfo
{
	uint32_t param;
	Port port;
};

static void UsbForwarding_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	//更改端口波特率
	if( driver_info.param > 500 )
		driver_info.port.SetBaudRate( driver_info.param , 2, 2 );
	else
		driver_info.port.SetBaudRate( 115200, 2, 2 );
	
	uint8_t buf[24];
	while(1)
	{
		uint8_t len = driver_info.port.read( buf, 24, 0.01, 0.5 );
		if( len )
			Write_USBD_VCOM( buf, len );
		
		len = Read_USBD_VCOM( buf, 24, 0.01, 0.5 );
		if( len )
			driver_info.port.write( buf, len, 0.1, 0.1 );
	}
}

static bool UsbForwarding_DriverInit( Port port, uint32_t param )
{
	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	xTaskCreate( UsbForwarding_Server, "UsbForwarding", 512, (void*)driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_UsbForwarding()
{
	PortFunc_Register( 3, UsbForwarding_DriverInit );
}