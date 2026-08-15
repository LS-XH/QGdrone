#pragma once
#include "mavlink.h"

bool Msg206_ACFlyPosSensor_INFO( uint8_t port, mavlink_message_t* msg_sd, uint8_t ind );

extern bool (*const Mavlink_Send_Funcs[])( uint8_t port , mavlink_message_t* msg_sd );
extern const uint16_t Mavlink_Send_Funcs_Count;