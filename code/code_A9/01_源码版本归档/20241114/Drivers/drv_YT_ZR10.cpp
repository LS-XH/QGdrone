#include "drv_YT_ZR10.hpp"
#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"
#include "AuxFuncs.hpp"
#include "MavlinkCMDProcess.hpp"
#include "drv_PWMOut.hpp"
#include "Parameters.hpp"
#include "drv_CRC.hpp"
									
static uint8_t componentID = MAV_COMP_ID_CAMERA_SIYI_ZR10;

struct DriverInfo
{
	uint32_t param;
	Port port;
};

static inline void cmdPackSend(DriverInfo* driver_info,int8_t* data, uint16_t dataLen, uint8_t cmd, bool need_ack)
{
	static uint16_t seq=0;
	Aligned_DMABuf int8_t sdata[64];
	*(uint16_t*)&sdata[0] = 0x6655;
	*(uint8_t*) &sdata[2] = need_ack ? 1: 0;
	*(uint16_t*)&sdata[3] = dataLen;
	*(uint16_t*)&sdata[5] = seq++;
	*(uint8_t*) &sdata[7] = cmd;
	memcpy(&sdata[8],data,dataLen);
	uint32_t res =0;
	CRC_Calc( cg_CRC_CONFIG_CRC16_XMODEM, (uint8_t*)&sdata[0], dataLen+8, &res);
	*(uint16_t*)&sdata[dataLen+8] = res;	
  driver_info->port.write( (uint8_t*)&sdata, dataLen+8+2, 0.1, 0.1 );	
}


static void YT_ZR10_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	CmdMsg msg;
	TIME attSendTime;
	
	while(1)
	{
		msg.cmd=0;
		bool msgAvailable=ReceiveCmdMsgFromTask( &msg, componentID, 0.01);
		if(msgAvailable)
		{
				if(msg.cmd == MAV_CMD_IMAGE_START_CAPTURE)
				{// 拍照
					int8_t data=0;
					cmdPackSend(&driver_info,&data,1,0x0C,false);
		      // 反馈
					SendCmdMsgToTask( msg, CAMERA_FEEDBACK_QUEUE_ID, 0.005 );
					//sendLedSignal(LEDSignal_Success1);
				}
		}				
		
		if(attSendTime.get_pass_time()>0.02){
			attSendTime = TIME::now();
			/*发送姿态数据*/
				typedef struct
				{
					uint32_t time_boot_ms;
					float rol;
					float pit;
					float yaw;
					float rolRate;
					float pitRate;
					float yawRate;
				}__PACKED _attPack;
				_attPack attPack;
				
				attPack.time_boot_ms = TIME::get_System_Run_Time() * 1000;
				Quaternion quat;
				get_AirframeY_quat(&quat);
				attPack.rol = +quat.getRoll();
				attPack.pit = -quat.getPitch();
				attPack.yaw = -quat.getYaw();
				vector3<double> angularRate;
				get_AngularRate_Ctrl(&angularRate);
				attPack.rolRate = +angularRate.x;
				attPack.pitRate = -angularRate.y;
				attPack.yawRate = -angularRate.z;
				
				cmdPackSend( &driver_info, (int8_t*)&attPack, sizeof(_attPack), 0x22, false );
			/*发送姿态数据*/
		}
	}
}

static bool YT_ZR10_DriverInit( Port port, uint32_t param )
{
	// 注册消息队列
	TaskQueueRegister(componentID,30);
	
	//波特率19200
	port.SetBaudRate( 115200, 2, 2 );

	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	xTaskCreate( YT_ZR10_Server, "YT_ZR10", 1000, (void*)driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_YT_ZR10()
{
	PortFunc_Register( 107, YT_ZR10_DriverInit );
}