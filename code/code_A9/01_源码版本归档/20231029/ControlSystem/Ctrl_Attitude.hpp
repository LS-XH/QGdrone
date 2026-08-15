#pragma once

#include "Basic.hpp"

void ctrl_Attitude();

void init_Ctrl_Attitude();

/*内部接口*/
	float getVoltKp();
	int8_t getCtrlBatId();
/*内部接口*/