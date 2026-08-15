#pragma once

//高度无参ESO
//朱文杰 20180102
//请勿用于商业用途
//！！抄袭必究！！

#include <stdbool.h>
#include "AC_Math.hpp"
#include "Filters_LP.hpp"
#include "TD4.hpp"

class ESO_h
{
	private:
		double beta;
		double betaAcc;
		
		double Hz;
		double h;
		
		double T;
		double invT;
		double z_inertia;
		double z2;
		double u;
		TD3_Lite b_filter;
		Filter_Butter2_LP acc_filter;
	
		//输出->加速度增益
		double b;		
		//当前输出力（加速度）
		double force;

		bool err_sign;
		double err_continues_time;
	
	public:		
		inline void init( double T, double beta, double betaAcc, double Hz )
		{			
			this->T = T;
			this->invT = 1.0f / T;
			this->beta = beta*10;
			this->betaAcc = betaAcc;
			this->u = 0;
			this->z2 =1.0f;
			
			b_filter.reset();
			this->b = b_filter.x1 = 1000;
			acc_filter.reset(0);
			acc_filter.set_cutoff_frequency( Hz, betaAcc );
			
			this->Hz = Hz;
			this->h = 1.0 / Hz;
		}
		ESO_h()
		{
			b_filter.reset();
			this->b = b_filter.x1 = 1000;
		}
		
		inline void update_u( double u )
		{
			this->u = u;
			this->z_inertia += this->h * this->invT * ( this->u - this->z_inertia );
			this->force = this->b * this->z_inertia;
		}
		
		inline double get_u(){ return this->u; }
		inline double get_T(){ return this->T; }
		inline double get_b(){ return this->b; }
		inline double get_force(){ return this->force; }
		inline double get_EsAcc(){ return this->force + this->acc_filter.get_result() - GravityAcc; }
		
		inline double get_hover_throttle(){ return this->z2; }
		
		double run( double acc );
};

