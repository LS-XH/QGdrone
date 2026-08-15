#pragma once

//角速度带模型ESO
//朱文杰 20180102
//请勿用于商业用途
//！！抄袭必究！！

#include "Basic.h"
#include <stdbool.h>
#include "AC_Math.hpp"
#include "Filters_LP.hpp"
#include "Filters_BS.hpp"
#include "complex.hpp"

//FT采样点数目(必须为偶数)
#define FT_N 80

class ESO_AngularRate_Base
{
	public:
		virtual ~ESO_AngularRate_Base() = default;
	
		virtual double get_T() const = 0;
		virtual double get_b() const = 0;
		virtual double get_u() const = 0;
	
		virtual void update_u( double u ) = 0;
		virtual double run( double rate, bool inFlight ) = 0;
	
		//获取傅里叶变换结果
		virtual const complex<double>* get_ftFreqs() const = 0;
	
		//获取学习参数
		virtual const double* get_LearnKs() const = 0;
	
		//装载学习结果
		virtual void load_LearnKs( double* learnKs ) = 0;
	
		//获取估计角速度
		virtual double get_EsAngularRate() const = 0;
		//获取估计扰动
		virtual double get_EsDisturbance() const = 0;
		//获取估计滤波扰动
		virtual double get_EsDisturbanceFilted() const = 0;
		//获取估计扰动(未知)
		virtual double get_EsDisturbanceZ() const = 0;
		//获取估计扰动(未知)
		virtual double get_EsDisturbanceZFilted() const = 0;
		//获取估计扰动(学习)
		virtual double get_EsDisturbanceLearn() const = 0;
		//获取估计扰动(学习)
		virtual double get_EsDisturbanceLearnF() const = 0;
		//获取估计扰动导数
		virtual double get_EsDisturbanceD() const = 0;
		//获取估计角加速度
		virtual double get_EsAngularAcceleration() const = 0;
		//获取估计主动力
		virtual double get_EsMainPower() const = 0;
};

extern const uint16_t learnK_Count;
class ESO_AngularRate : public ESO_AngularRate_Base
{
	private:
		double invT;
		double z_inertia;
		double z1;
		double z2;
		double z2Learn;
		double z2LearnF;
		double z2LearnD;
		
		double last_err;
		double last_rate;
		double last_disturbanceFilted;
		double disturbanceD_Filted;

		double Hz;
		double h;
	
		//陷波频点
		double bsFreq1;	double bsRange1;	uint16_t bs1Counter;
		double bsFreq2;	double bsRange2;	uint16_t bs2Counter;
	
		//角速度滤波器
		Filter_Butter_LP rate_filter;
		Filter_Butter2_BS rate_BSfilter1;
		Filter_Butter2_BS rate_BSfilter2;
	
		//角加速度滤波器
		Filter_Butter_LP acc_filter;
		Filter_Butter2_BS acc_BSfilter1;
		Filter_Butter2_BS acc_BSfilter2;
		
		//学习滤波器
		Filter_Butter_LP learnZin_filter;
		Filter_Butter2_BS learnZin_BSfilter1;
		Filter_Butter_LP learnErr_filter;
		Filter_Butter2_BS learnErr_BSfilter1;
		
		//扰动滤波器
		Filter_Butter_LP disturbanceZ_filter;
	
		//傅里叶变换
		uint16_t ft_ind;
		double ft_in[FT_N];
		complex<double> ft_freqs[FT_N/2];
		//傅里叶变换参数
		static bool ft_coeffs_initialized;
		static complex<double> ft_coeffs[FT_N];
		//初始化傅里叶变换
		void init_ft();
		
		/*学习系统*/
			//是否开启学习
			bool learnEna;
			//学习参数
			double learnK[4];
			//学习数据
			double X3_filted;
			double last_z2Learn;
			//模型短期数据1
			double learn_ST1_sumX1;
			double learn_ST1_sumX2;
			double learn_ST1_sumX3;
			double learn_ST1_sumy;
			double learn_ST1_sumX1_2;
			double learn_ST1_sumX2_2;
			double learn_ST1_sumX3_2;
			double learn_ST1_sumX1X2;
			double learn_ST1_sumX1X3;
			double learn_ST1_sumX2X3;
			double learn_ST1_sumX1y;
			double learn_ST1_sumX2y;
			double learn_ST1_sumX3y;
			//模型短期数据2
			double learn_ST2_sumX1;
			double learn_ST2_sumX2;
			double learn_ST2_sumX3;
			double learn_ST2_sumy;
			double learn_ST2_sumX1_2;
			double learn_ST2_sumX2_2;
			double learn_ST2_sumX3_2;
			double learn_ST2_sumX1X2;
			double learn_ST2_sumX1X3;
			double learn_ST2_sumX2X3;
			double learn_ST2_sumX1y;
			double learn_ST2_sumX2y;
			double learn_ST2_sumX3y;
			//长期学习
			uint8_t learn_hisCnt;
			uint16_t learn_ltCounter;
			double learn_sumX1;
			double learn_sumX2;
			double learn_sumX3;
			double learn_sumy;
			double learn_sumX1_2;
			double learn_sumX2_2;
			double learn_sumX3_2;
			double learn_sumX1X2;
			double learn_sumX1X3;
			double learn_sumX2X3;
			double learn_sumX1y;
			double learn_sumX2y;
			double learn_sumX3y;
			
			//初始化学习
			void init_learn();
		/*学习系统*/
	
	public:	
		
		double beta1;
		double beta2;
	
		double T;	double get_T() const{ return this->T; };
		double b;	double get_b() const{ return this->b; };
		double u;	double get_u() const{ return this->u; };
		
		const complex<double>* get_ftFreqs() const { return ft_freqs; }
		const double* get_LearnKs() const { return learnK; }
		
		void load_LearnKs( double* learnKs )
		{
			memcpy( this->learnK, learnKs, 4*sizeof(double) );
			for( uint8_t i=0; i<4; ++i )
			{
				if( !isvalid(this->learnK[i]) )
				{
					memset( this->learnK, 0, 4*sizeof(double) );
					break;
				}
			}
		}
		
		double get_bsFreq1() const{ return this->bsFreq1; };	double get_bsRange1() const{ return this->bsRange1; };
		double get_bsFreq2() const{ return this->bsFreq2; };	double get_bsRange2() const{ return this->bsRange2; };
		
		ESO_AngularRate()
		{
		}
		
		inline void init( 
				bool learnEna,
				uint8_t order, double T , double b , double beta1 , double beta2 ,
				uint8_t orderD, double betaD,
				double Hz )
		{
			this->learnEna = learnEna;
			this->Hz = Hz;
			this->h = 1.0 / Hz;
			this->beta1 = beta1;
			this->beta2 = beta2;
			
			//重置角速度滤波器
			rate_filter.set_cutoff_frequency( order, Hz, beta1 );	rate_filter.reset(0);
			rate_BSfilter1.set_inavailable();	rate_BSfilter1.reset(0);
			rate_BSfilter2.set_inavailable();	rate_BSfilter2.reset(0);
			//重置角加速度滤波器
			acc_filter.set_cutoff_frequency( order, Hz, beta2 );	acc_filter.reset(0);
			acc_BSfilter1.set_inavailable();	acc_BSfilter1.reset(0);
			acc_BSfilter2.set_inavailable();	acc_BSfilter2.reset(0);
			//重置扰动滤波器
			disturbanceZ_filter.set_cutoff_frequency( orderD, Hz, betaD );	disturbanceZ_filter.reset(0);
			disturbanceD_Filted = 0;
			//重置学习滤波器
			learnZin_filter.set_cutoff_frequency( order, Hz, beta2 );	learnZin_filter.reset(0);
			learnZin_BSfilter1.set_inavailable();	learnZin_BSfilter1.reset(0);
			learnErr_filter.set_cutoff_frequency( order, Hz, beta2 );	learnErr_filter.reset(0);
			learnErr_BSfilter1.set_inavailable();	learnErr_BSfilter1.reset(0);
			X3_filted = 0;
			last_z2Learn = 0;
			
			bsFreq1 = bsRange1 = bs1Counter = 0;
			bsFreq2 = bsRange2 = bs2Counter = 0;
			
			this->z1 = this->z2 = this->z2Learn = this->z2LearnF = this->z2LearnD = this->z_inertia = 0;
			this->last_err = this->last_rate = this->last_disturbanceFilted = 0;
			this->learnK[0] = this->learnK[1] = this->learnK[2] = this->learnK[3] = 0;
			this->u = 0;
			
			this->T = T;	this->invT = 1.0 / T;
			this->b = b;
			init_ft();
			init_learn();
		}
		
		inline void update_u( double u )
		{
			this->u = u;
			this->z1 += this->h * ( this->z_inertia + this->z2 + this->z2Learn );
			this->z_inertia += this->h * this->invT * ( this->b*this->u - this->z_inertia );
		}
		
		double run( double rate, bool inFlight );
		
		inline double get_EsAngularRate() const
		{
			return this->z1;
		}
		inline double get_EsDisturbance() const
		{
			return this->z2 + this->z2Learn;
		}
		inline double get_EsDisturbanceFilted() const
		{
			return disturbanceZ_filter.get_result()+ this->z2Learn;
		}
		inline double get_EsDisturbanceZ() const
		{
			return this->z2;
		}
		inline double get_EsDisturbanceZFilted() const
		{
			return disturbanceZ_filter.get_result();
		}
		inline double get_EsDisturbanceLearn() const
		{
			return this->z2Learn;
		}
		inline double get_EsDisturbanceLearnF() const
		{
			return this->z2LearnF;
		}
		inline double get_EsDisturbanceD() const
		{
			return disturbanceD_Filted + z2LearnD;
		}
		inline double get_EsAngularAcceleration() const
		{
			return this->z2 + this->z_inertia + this->z2Learn;
		}
		inline double get_EsMainPower() const
		{
			return this->z_inertia;
		}
};

class ESO_AngularRateHeli : public ESO_AngularRate_Base
{
	private:
		double invT;
		double z_inertia;
		double z1;

		double last_v;
		double acc;
	
		double Hz;
		double h;
		
		double vibeFreq1;
		double vibeFreq2;
	
		Filter_Butter_LP rate_filter;
		Filter_Butter2_BS rate_BSfilter1;
		Filter_Butter2_BS rate_BSfilter2;
	
		Filter_Butter_LP disturbance_filter;
	
		//傅里叶变换
		uint16_t ft_ind;
		double ft_in[FT_N];
		complex<double> ft_freqs[FT_N/2];
		//傅里叶变换参数
		static bool ft_coeffs_initialized;
		static complex<double> ft_coeffs[FT_N];
	
	public:	
		double beta1;
	
		double T;	double get_T() const{ return this->T; };
		double b;	double get_b() const{ return this->b; };
		double u;	double get_u() const{ return this->u; };
	
		const complex<double>* get_ftFreqs() const { return ft_freqs; }
		const double* get_LearnKs() const { return 0; }
		
		void load_LearnKs( double* learnKs )
		{
			
		}
		
		void init_ft();
		
		inline void init( uint8_t order, double T , double b , double beta1 ,
			uint8_t orderD, double betaD,
			double Hz )
		{
			this->Hz = Hz;
			this->h = 1.0 / Hz;
			this->beta1 = beta1;
			rate_filter.set_cutoff_frequency( order, Hz, beta1 );
			rate_BSfilter1.set_inavailable();
			rate_BSfilter2.set_inavailable();
			disturbance_filter.set_cutoff_frequency( orderD, Hz, betaD );
			
			this->z1 = this->z_inertia = 0;
			
			this->T = T;	this->invT = 1.0f / T;
			this->b = b;
			
			vibeFreq1 = vibeFreq2 = 0;
			init_ft();
		}
		
		inline void update_u( double u )
		{
			this->u = u;
			this->z_inertia += this->h * this->invT * ( this->b*this->u - this->z_inertia );
		}
		
		double run( double rate, bool inFlight );
		
		inline double get_EsAngularRate() const
		{
			return this->z_inertia + this->z1;
		}
		inline double get_EsDisturbance() const
		{
			return this->z1;
		}
		inline double get_EsDisturbanceFilted() const
		{
			return this->z1;
		}
		inline double get_EsDisturbanceZ() const
		{
			return this->z1;
		}
		inline double get_EsDisturbanceZFilted() const
		{
			return this->z1;
		}
		inline double get_EsDisturbanceLearn() const
		{
			return 0;
		}
		inline double get_EsDisturbanceLearnF() const
		{
			return 0;
		}
		inline double get_EsDisturbanceD() const
		{
			return 0;
		}
		inline double get_EsAngularAcceleration() const
		{
			return this->acc;
		}
		inline double get_EsMainPower() const
		{
			return this->z_inertia;
		}
};