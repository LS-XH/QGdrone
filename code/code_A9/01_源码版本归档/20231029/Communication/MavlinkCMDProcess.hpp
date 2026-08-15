#pragma once

#include "mavlink.h"
#define MAX_CMD_QUEUE_ID 5
#define CAMERA_GIMBAL_CONTROL_CMD_QUEUE_ID 0
#define CAMERA_GIMBAL_FEEDBACK_QUEUE_ID 1

struct CmdMsg
{
	uint8_t port_id;
	uint8_t target_system;
	uint8_t target_component;
	uint32_t cmd;
	uint8_t cmdSourceType;
	bool need_ack;
	double params[16];
};
bool SendCmdMsgToAllTask( CmdMsg msg, double TIMEOUT );
bool SendCmdMsgToTask( CmdMsg msg, uint8_t queue_id, double TIMEOUT );
bool TaskReceiveCmdMsg( CmdMsg* msg, uint8_t queue_id, double TIMEOUT );

//发送版本信息
void send_AutoPilot_Version( uint8_t port_index );
//Mavlink命令处理函数表
extern void (*const Mavlink_CMD_Process[])( uint8_t port_index , const mavlink_message_t* msg_rd );
//Mavlink命令处理函数个数
extern const uint16_t Mavlink_CMD_Process_Count;

extern void Cmd2004_MAV_CMD_CAMERA_TRACK_POINT( uint8_t port_index , const mavlink_message_t* msg );
extern void Cmd1010_MAV_CMD_DO_GIMBAL_DISABLE_OSD( uint8_t port_index , const mavlink_message_t* msg );
extern void Cmd1009_MAV_CMD_DO_GIMBAL_ENABLE_OSD( uint8_t port_index , const mavlink_message_t* msg );
extern void Cmd1008_MAV_CMD_DO_GIMBAL_STOP_TARGET_DETECTION( uint8_t port_index , const mavlink_message_t* msg );
extern void Cmd1007_MAV_CMD_DO_GIMBAL_START_TARGET_DETECTION( uint8_t port_index , const mavlink_message_t* msg );
extern void Cmd1006_MAV_CMD_DO_GIMBAL_MANAGER_DOWN( uint8_t port_index , const mavlink_message_t* msg );
extern void Cmd1004_MAV_CMD_DO_GIMBAL_MANAGER_CENTER( uint8_t port_index , const mavlink_message_t* msg );
extern void Cmd1003_MAV_CMD_DO_GIMBAL_MANAGER_THERMAL( uint8_t port_index , const mavlink_message_t* msg );
extern void Cmd1002_MAV_CMD_DO_GIMBAL_MANAGER_LASER( uint8_t port_index , const mavlink_message_t* msg );
extern void Cmd1000_MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW( uint8_t port_index , const mavlink_message_t* msg );
extern void Cmd2000_MAV_CMD_IMAGE_START_CAPTURE( uint8_t port_index , const mavlink_message_t* msg );
extern void Cmd2010_MAV_CMD_CAMERA_STOP_TRACKING( uint8_t port_index , const mavlink_message_t* msg );
extern void Cmd2500_MAV_CMD_VIDEO_START_CAPTURE( uint8_t port_index , const mavlink_message_t* msg );
extern void Cmd2501_MAV_CMD_VIDEO_STOP_CAPTURE( uint8_t port_index , const mavlink_message_t* msg );
extern void Cmd532_MAV_CMD_SET_CAMERA_FOCUS( uint8_t port_index , const mavlink_message_t* msg );
extern void Cmd531_MAV_CMD_SET_CAMERA_ZOOM( uint8_t port_index , const mavlink_message_t* msg );
extern void Cmd530_MAV_CMD_SET_CAMERA_MODE( uint8_t port_index , const mavlink_message_t* msg );

