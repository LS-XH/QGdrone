#pragma once

#include "Modes.hpp"

class M14_HorizonCalib:public Mode_Base 
{
	private:
		
	public:
		M14_HorizonCalib();
		virtual ModeResult main_func( void* param1, uint32_t param2 );
};