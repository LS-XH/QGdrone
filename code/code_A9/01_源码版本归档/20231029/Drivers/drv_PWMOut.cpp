#include "drv_PwmOut.hpp"
#include "Basic.hpp"
#include "stm32h7xx.h"
#include "TimeBase.hpp"
#include "FreeRTOS.h"
#include "event_groups.h"
//舵机通道频率
#define AUX_FREQ 50
//主电机个数
static uint8_t MainMotorCount = 8;
//主舵机个数
static uint8_t MainStCount = 0;
//获取Aux通道个数
uint8_t get_AuxChannelCount()
{
	return PWMChannelsCount - MainMotorCount;
}
//获取主电机个数
uint8_t get_MainMotorCount()
{
	return MainMotorCount;
}

//channel类型记录
static ChannelOutputType channelsType[PWMChannelsCount] = {(ChannelOutputType)0};

/*
	PWM8( TIM4_CH3) -- PD14
	PWM7( TIM4_CH4) -- PD15
	PWM6( TIM8_CH1) -- PC6
	PWM5( TIM8_CH2) -- PC7
	PWM4( TIM3_CH1) -- PB4
	PWM3( TIM3_CH2) -- PB5
	PWM2(TIM15_CH1) -- PE5
	PWM1(TIM15_CH2) -- PE6
*/

static inline bool _getChannelPort( uint8_t ind, GPIO_TypeDef* &gpio, uint8_t& gpio_pin )
{
	switch(ind)
	{
		case 0:
			gpio = GPIOD;
			gpio_pin = 14;
			break;
		case 1:
			gpio = GPIOD;
			gpio_pin = 15;
			break;
		case 2:
			gpio = GPIOC;
			gpio_pin = 6;
			break;
		case 3:
			gpio = GPIOC;
			gpio_pin = 7;
			break;
		case 4:
			gpio = GPIOB;
			gpio_pin = 4;
			break;
		case 5:
			gpio = GPIOB;
			gpio_pin = 5;
			break;
		case 6:
			gpio = GPIOE;
			gpio_pin = 5;
			break;
		case 7:
			gpio = GPIOE;
			gpio_pin = 6;
			break;
		default:
			return false;
	}
	return true;
}

//func: 0-pwm推挽输出
//			3-上拉输入检测
static inline bool _setChannelFunc( uint8_t ind, ChannelOutputType func )
{
	if( ind >= PWMChannelsCount )
		return false;
	if( func == channelsType[ind] )
		return true;
	
	GPIO_TypeDef* gpio;
	uint8_t gpio_pin;
	if( _getChannelPort(ind, gpio, gpio_pin) == false )
		return false;
	
	switch(func)
	{
		default:
			return false;
		
		case ChannelOutputType_PWMPPOut:
		{	//PWM输出
			
			//复用模式
			set_register( gpio->MODER, 0b10, gpio_pin*2, 2 );
			//推挽输出
			set_register( gpio->OTYPER, 0, gpio_pin, 1 );
			//上拉
			set_register( gpio->PUPDR, 0b00, gpio_pin*2, 2 );
			
			break;
		}
		
		case ChannelOutputType_ODPUIn:
		{	//上拉输入检测
			
			//复用模式
			set_register( gpio->MODER, 0b00, gpio_pin*2, 2 );
			//开漏输出
			set_register( gpio->OTYPER, 1, gpio_pin, 1 );
			//上拉
			set_register( gpio->PUPDR, 0b01, gpio_pin*2, 2 );
			
			break;
		}
	}
	//更改channel类型
	channelsType[ind] = func;
	return true;
}

void set_MainMotorCount( uint8_t count, uint8_t st_count, float StFreq )
{
	//电机数必须是偶数
	if( count & 1 )
		++count;
	//舵机通道个数不得大于主通道数
	if( st_count > count )
		st_count = count;
	
	MainMotorCount = count;
	MainStCount = st_count;
	uint8_t MainMtCount = count - st_count;
	
	for( uint8_t i=0; i<count; ++i )
		_setChannelFunc(i, ChannelOutputType_PWMPPOut);
	
	if( MainMtCount >= 2 )
		TIM4->ARR = 1e6 / 400;
	else if( count >= 2 )
		TIM4->ARR = 1e6 / StFreq;
	else
		TIM4->ARR = 1e6 / AUX_FREQ;
	
	if( MainMtCount >= 4 )
		TIM8->ARR = 1e6 / 400;
	else if( count >= 4 )
		TIM8->ARR = 1e6 / StFreq;
	else
		TIM8->ARR = 1e6 / AUX_FREQ;
	
	if( MainMtCount >= 6 )
		TIM3->ARR = 1e6 / 400;
	else if( count >= 6 )
		TIM3->ARR = 1e6 / StFreq;
	else
		TIM3->ARR = 1e6 / AUX_FREQ;
	
	if( MainMtCount >= 8 )
		TIM15->ARR = 1e6 / 400;
	else if( count >= 8 )
		TIM15->ARR = 1e6 / StFreq;
	else
		TIM15->ARR = 1e6 / AUX_FREQ;
}

void MainMotor_PWM_Out( double out[8] )
{
	if( MainMotorCount >= 1 )
		TIM4->CCR3 = out[0]*10 + 1000;
	if( MainMotorCount >= 2 )
		TIM4->CCR4 = out[1]*10 + 1000;
	if( MainMotorCount >= 3 )
		TIM8->CCR1 = out[2]*10 + 1000;
	if( MainMotorCount >= 4 )
		TIM8->CCR2 = out[3]*10 + 1000;
	if( MainMotorCount >= 5 )
		TIM3->CCR1 = out[4]*10 + 1000;
	if( MainMotorCount >= 6 )
		TIM3->CCR2 = out[5]*10 + 1000;
	if( MainMotorCount >= 7 )
		TIM15->CCR1 = out[6]*10 + 1000;
	if( MainMotorCount >= 8 )
		TIM15->CCR2 = out[7]*10 + 1000;
}

void Aux_PWM_Out( double out, uint8_t ind )
{
	if( ind >= PWMChannelsCount )
		return;
	if( ind < MainMotorCount )
		return;
	
	//设置通道为推挽输出
	_setChannelFunc(ind, ChannelOutputType_PWMPPOut);
	
	if( ind==0 )
		TIM4->CCR3 = out*10 + 1000;
	else if( ind==1 )
		TIM4->CCR4 = out*10 + 1000;
	else if( ind==2 )
		TIM8->CCR1 = out*10 + 1000;
	else if( ind==3 )
		TIM8->CCR2 = out*10 + 1000;
	else if( ind==4 )
		TIM3->CCR1 = out*10 + 1000;
	else if( ind==5 )
		TIM3->CCR2 = out*10 + 1000;
	else if( ind==6 )
		TIM15->CCR1 = out*10 + 1000;
	else if( ind==7 )
		TIM15->CCR2 = out*10 + 1000;
}

int32_t getPWMus( uint8_t ind )
{
	if( ind >= PWMChannelsCount )
		return -1;
	if( ind < MainMotorCount )
		return -1;
	
	//设置通道为推挽输出
	_setChannelFunc(ind, ChannelOutputType_PWMPPOut);
	
	if( ind==0 )
		return TIM4->CCR3;
	else if( ind==1 )
		return TIM4->CCR4;
	else if( ind==2 )
		return TIM8->CCR1;
	else if( ind==3 )
		return TIM8->CCR2;
	else if( ind==4 )
		return TIM3->CCR1;
	else if( ind==5 )
		return TIM3->CCR2;
	else if( ind==6 )
		return TIM15->CCR1;
	else if( ind==7 )
		return TIM15->CCR2;
	
	return -1;
}

bool Aux_ChannelRead( double ind, bool* res )
{
	if( ind >= PWMChannelsCount )
		return false;
	if( ind < MainMotorCount )
		return false;
	
	GPIO_TypeDef* gpio;
	uint8_t gpio_pin;
	if( _getChannelPort(ind, gpio, gpio_pin) == false )
		return false;
	
	//设置通道为上拉输入
	if( _setChannelFunc(ind, ChannelOutputType_ODPUIn) == false )
		return false;
	
	if(res)
	{
		*res = gpio->IDR & (1<<gpio_pin);
		return true;
	}
	return false;
}

void PWM_DisableAll()
{
  TIM15->CCR2=0000;
	TIM15->CCR1=0000;
  TIM3->CCR2 =0000;
  TIM3->CCR1 =0000;
  TIM8->CCR2 =0000;
	TIM8->CCR1 =0000;
	TIM4->CCR4 =0000;
	TIM4->CCR3 =0000;
}
void PWM_PullDownAll()
{
  TIM15->CCR2=1000;
	TIM15->CCR1=1000;
  TIM3->CCR2 =1000;
  TIM3->CCR1 =1000;
  TIM8->CCR2 =1000;
	TIM8->CCR1 =1000;
	TIM4->CCR4 =1000;
	TIM4->CCR3 =1000;
}
void MainMotor_PullUpAll()
{
	uint8_t MainMTCount = MainMotorCount - MainStCount;
	if( MainMTCount >= 1 )
		TIM4->CCR3 = 2000;
	if( MainMTCount >= 2 )
		TIM4->CCR4 = 2000;
	if( MainMTCount >= 3 )
		TIM8->CCR1 = 2000;
	if( MainMTCount >= 4 )
		TIM8->CCR2 = 2000;
	if( MainMTCount >= 5 )
		TIM3->CCR1 = 2000;
	if( MainMTCount >= 6 )
		TIM3->CCR2 = 2000;
	if( MainMTCount >= 7 )
		TIM15->CCR1 = 2000;
	if( MainMTCount >= 8 )
		TIM15->CCR2 = 2000;
}
void MainMotor_PullDownAll()
{
	uint8_t MainMTCount = MainMotorCount - MainStCount;
	if( MainMTCount >= 1 )
		TIM4->CCR3 = 1000;
	if( MainMTCount >= 2 )
		TIM4->CCR4 = 1000;
	if( MainMTCount >= 3 )
		TIM8->CCR1 = 1000;
	if( MainMTCount >= 4 )
		TIM8->CCR2 = 1000;
	if( MainMTCount >= 5 )
		TIM3->CCR1 = 1000;
	if( MainMTCount >= 6 )
		TIM3->CCR2 = 1000;
	if( MainMTCount >= 7 )
		TIM15->CCR1 = 1000;
	if( MainMTCount >= 8 )
		TIM15->CCR2 = 1000;
}
void PWM_PullUpAll()
{
	TIM15->CCR2=2000;
	TIM15->CCR1=2000;
  TIM3->CCR2 =2000;
  TIM3->CCR1 =2000;
  TIM8->CCR2 =2000;
	TIM8->CCR1 =2000;
	TIM4->CCR4 =2000;
	TIM4->CCR3 =2000;
}

void init_drv_PWMOut()
{
/*配置TIM*/ 	
  //使能TIM时钟
	RCC->APB1LENR|=(1<<2)|(1<<1);
  RCC->APB2ENR|=(1<<16)|(1<<1);
	os_delay(0.01);
	
	PWM_DisableAll();
	
	TIM3->PSC  =(APB1TIMERCLK / 1e6) - 1;
	TIM3->ARR  = 1e6 / 400;
  TIM3->CCMR1 = (0b110<<12)|(1<<11)|(0b110<<4)|(1<<3);
  TIM3->CCER =(1<<4)|(1<<0);
  TIM3->EGR = (1<<0);
  TIM3->CR1 = (1<<7)|(1<<0);

	TIM4->PSC = (APB1TIMERCLK / 1e6) - 1;
	TIM4->ARR = 1e6 / 400;
	TIM4->CCMR2 = (0b110<<12)|(1<<11)|(0b110<<4)|(1<<3);
	TIM4->CCER =(1<<12) | (1<<8);
	TIM4->EGR |=(1<<0);
	TIM4->CR1 = (1<<7)|(1<<0);
	

  TIM8->PSC = (APB2TIMERCLK / 1e6)-1;
  TIM8->ARR = 1e6 / 400;
  TIM8->CCMR1 = (0b110<<4)|(0b110<<12)|(1<<3)|(1<<11);                                       
	TIM8->CCER = (1<<4)|(1<<0);
	TIM8->BDTR = (1<<15);
  TIM8->EGR = (1<<0);                    
  TIM8->CR1 = (1<<7)|(1<<0);          

  TIM15->PSC=(APB2TIMERCLK / 1e6)-1;
  TIM15->ARR = 1e6 / 400;
  TIM15->CCMR1 = (0b110<<4)|(0b110<<12)|(1<<3)|(1<<11);                                       
	TIM15->CCER = (1<<4)|(1<<0);
	TIM15->BDTR = (1<<15);
  TIM15->EGR = (1<<0);                    
  TIM15->CR1 = (1<<7)|(1<<0);   
/*配置TIM*/ 

/*配置GPIO*/  	
	//使能GPIO时钟
	RCC->AHB4ENR |= (1<<4)|(1<<1)|(1<<2)|(1<<3);
	os_delay(0.01);

	//推挽输出
	set_register( GPIOB->OTYPER, 0, 4, 1 );	//PB4
	set_register( GPIOB->OTYPER, 0, 5, 1 );	//PB5
	set_register( GPIOC->OTYPER, 0, 6, 1 );	//PC6
	set_register( GPIOC->OTYPER, 0, 7, 1 );	//PC7
	set_register( GPIOD->OTYPER, 0, 14, 1 );	//PD14
	set_register( GPIOD->OTYPER, 0, 15, 1 );	//PD15
	set_register( GPIOE->OTYPER, 0, 5, 1 );	//PE5
	set_register( GPIOE->OTYPER, 0, 6, 1 );	//PE6
	
	//无上拉
	set_register( GPIOB->PUPDR, 0b00, 4*2, 2 );	//PB4
	set_register( GPIOB->PUPDR, 0b00, 5*2, 2 );	//PB5
	set_register( GPIOC->PUPDR, 0b00, 6*2, 2 );	//PC6
	set_register( GPIOC->PUPDR, 0b00, 7*2, 2 );	//PC7
	set_register( GPIOD->PUPDR, 0b00, 14*2, 2 );	//PD14
	set_register( GPIOD->PUPDR, 0b00, 15*2, 2 );	//PD15
	set_register( GPIOE->PUPDR, 0b00, 5*2, 2 );	//PE5
	set_register( GPIOE->PUPDR, 0b00, 6*2, 2 );	//PE6
	
	//速度
	set_register( GPIOB->OSPEEDR, 0b01, 4*2, 2 );	//PB4
	set_register( GPIOB->OSPEEDR, 0b01, 5*2, 2 );	//PB5
	set_register( GPIOC->OSPEEDR, 0b01, 6*2, 2 );	//PC6
	set_register( GPIOC->OSPEEDR, 0b01, 7*2, 2 );	//PC7
	set_register( GPIOD->OSPEEDR, 0b01, 14*2, 2 );	//PD14
	set_register( GPIOD->OSPEEDR, 0b01, 15*2, 2 );	//PD15
	set_register( GPIOE->OSPEEDR, 0b01, 5*2, 2 );	//PE5
	set_register( GPIOE->OSPEEDR, 0b01, 6*2, 2 );	//PE6
	
	//复用功能配置
	set_register( GPIOB->AFR[0], 2, 4*4, 4 );	//PB4
	set_register( GPIOB->AFR[0], 2, 5*4, 4 );	//PB5
	set_register( GPIOC->AFR[0], 3, 6*4, 4 );	//PC6
	set_register( GPIOC->AFR[0], 3, 7*4, 4 );	//PC7
	set_register( GPIOD->AFR[1], 2, 14*4-32, 4 );	//PD14
	set_register( GPIOD->AFR[1], 2, 15*4-32, 4 );	//PD15
	set_register( GPIOE->AFR[0], 4, 5*4, 4 );	//PE5
	set_register( GPIOE->AFR[0], 4, 6*4, 4 );	//PE6
	
	//复用模式
	set_register( GPIOB->MODER, 0b10, 4*2, 2 );	//PB4
	set_register( GPIOB->MODER, 0b10, 5*2, 2 );	//PB5
	set_register( GPIOC->MODER, 0b10, 6*2, 2 );	//PC6
	set_register( GPIOC->MODER, 0b10, 7*2, 2 );	//PC7
	set_register( GPIOD->MODER, 0b10, 14*2, 2 );	//PD14
	set_register( GPIOD->MODER, 0b10, 15*2, 2 );	//PD15
	set_register( GPIOE->MODER, 0b10, 5*2, 2 );	//PE5
	set_register( GPIOE->MODER, 0b10, 6*2, 2 );	//PE6
/*配置GPIO*/  

	for( uint8_t i=0; i<PWMChannelsCount; ++i )
		_setChannelFunc( i, ChannelOutputType_PWMPPOut );
}