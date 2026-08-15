#pragma once

#include <stdint.h>

struct DGT_PWM_CONFIG
{
	uint32_t calib[2];
	//校准电调id
	uint32_t calib_id[2];
	//校准参数
	uint32_t calib_param1[2];
	uint32_t calib_param2[2];
}__attribute__((__packed__));

void init_dgtPwm();