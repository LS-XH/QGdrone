#include "dgtPwm.hpp"
#include "Commulink.hpp"
#include "Parameters.hpp"

void init_dgtPwm()
{
	DGT_PWM_CONFIG init_cfg;
	init_cfg.calib[0] = 0;
	init_cfg.calib_id[0] = 0;
	init_cfg.calib_param1[0] = 0;
	init_cfg.calib_param2[0] = 0;
	
	MAV_PARAM_TYPE param_types[] = {
		MAV_PARAM_TYPE_UINT32,
		MAV_PARAM_TYPE_UINT32,
		MAV_PARAM_TYPE_UINT32,
		MAV_PARAM_TYPE_UINT32,
	};
	SName param_names[] = {
		"dgtPwm_Calib",
		"dgtPwm_C_Id",
		"dgtPwm_C_Param1",
		"dgtPwm_C_Param2",
	};
	ParamGroupRegister( "DGTPWM", 1, sizeof(init_cfg)/8, param_types, param_names, (uint64_t*)&init_cfg );	
}