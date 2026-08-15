#pragma once

#include "Basic.hpp"
#include <stdbool.h>
#include "Receiver.hpp"

bool ReceiverRegister( SName name, bool is_JoyStick=false, const RC_CONFIG* initialCfg=0 );

//更新接收机数据
bool ReceiverUpdate( SName name, bool connected, float raw_data[16], uint8_t channel_count, double TIMEOUT );