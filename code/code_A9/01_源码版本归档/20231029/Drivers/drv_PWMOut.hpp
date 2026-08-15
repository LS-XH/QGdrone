#pragma once

#include <stdint.h>

#define PWMChannelsCount 8
//获取Aux通道个数
uint8_t get_AuxChannelCount();
//获取主电机个数
uint8_t get_MainMotorCount();

enum ChannelOutputType
{
	//推挽PWM输出
	ChannelOutputType_PWMPPOut = 0 ,
	//开漏上拉输入检测
	ChannelOutputType_ODPUIn ,
};

/*设置主电机个数
	count：主电机+舵机个数
	st_count：主舵机个数(须小于count)
	StFreq：主舵机频率
*/
void set_MainMotorCount( uint8_t count, uint8_t st_count=0, float StFreq=50 );
/*在主通道输出PWM信号
	out：通道信号值0-100
		0:		1000us高电平
		100:	2000us高电平
*/
void MainMotor_PWM_Out( double out[8] );
/*在辅助通道输出PWM信号
	注意：该操作会将辅助通道对应端口设置为PWM输出模式
	out：通道信号值0-100
		0:		1000us高电平
		100:	2000us高电平
	ind：通道号 0-通道1 1-通道2...
*/
void Aux_PWM_Out( double out, uint8_t ind );
/*读取IO口电平值
	注意：该操作会将辅助通道对应端口设置为IO输入模式
	ind：通道号 0-通道1 1-通道2...
	res：高电平返回true 低电平返回false
*/
bool Aux_ChannelRead( double ind, bool* res );
/*获取通道输出的PWM值
	注意：该操作会将辅助通道对应端口设置为PWM输出模式
	ind：通道号 0-通道1 1-通道2...
*/
int32_t getPWMus( uint8_t ind );

//拉高全部主通道PWM(2000us)
void MainMotor_PullUpAll();
//拉低全部主通道PWM(1000us)
void MainMotor_PullDownAll();
//拉低全部PWM(1000us)
void PWM_PullDownAll();
//拉高全部PWM(2000us)
void PWM_PullUpAll();

void init_drv_PWMOut();