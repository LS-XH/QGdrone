#include "Fences.hpp"

#include "Parameters.hpp"
#include "semphr.h"
#include "Modes.hpp"

#define FenceParamVersion 1
#define MaxFences 512
#define FencesInParamGroupBit 5
#define FencesInParamGroup (1<<FencesInParamGroupBit)

static SemaphoreHandle_t FencesSemphr = xSemaphoreCreateMutex();

static inline bool Lock_Fences( double TIMEOUT = -1 )
{
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake( FencesSemphr , TIMEOUT_Ticks ) )
		return true;
	return false;
}
static inline void UnLock_Fences()
{
	xSemaphoreGive(FencesSemphr);
}

//航点个数
static uint16_t FencesCount = 0;
static uint16_t UploadingFencesCount = 0;

//当前航点
static int16_t CurrentFence = -1;
/*
	设置当前任务
	wpInd：当前任务序号
*/
bool setCurrentFence( int16_t fenceInd )
{
	if( fenceInd >= FencesCount )
		return false;
	CurrentFence = fenceInd;
	return true;
}
/*
	读取当前任务序号
*/
int16_t getCurrentFenceInd() { return CurrentFence; }

/*
	获取航点个数
*/
uint16_t getFencesCount()
{
	return FencesCount;
}
/*
	获取正在上传航点个数
*/
uint16_t getUploadingFencesCount()
{
	return UploadingFencesCount;
}

/*
	清除所有航点任务
	TIMEOUT: 线程同步超时时间

	返回值：
	true:添加成功
	false:添加失败（航点过多或航点信息错误或参数表错误）
*/
bool clearFences( double TIMEOUT )
{
	if( Lock_Fences(TIMEOUT) )
	{
		TruncateVolatileParamGroup( "Fences", 0 );
		
		FencesCount = UploadingFencesCount = 0;
		CurrentFence = -1;
		
		UnLock_Fences();
		
	}
	else
		return false;
	
	return true;
}

/*
	添加航点任务
	wp_inf：航点信息
	st：是否写入存储器（只有当前实际航点数量为0才可以缓存不写入存储器）
	TIMEOUT: 线程同步超时时间

	返回值：
	true:添加成功
	false:添加失败（航点过多或航点信息错误或参数表错误）
*/
bool addFence( FenceInf wp_inf, bool st, double TIMEOUT )
{
	if( wp_inf.cmd == 0 )
		return false;
	if( FencesCount > MaxFences )
		return false;
	if( st==false && FencesCount>0 )
		return false;
	if( st && FencesCount!=UploadingFencesCount )
		return false;
	
	if( Lock_Fences(TIMEOUT) )
	{
		WriteVolatileParamGroup( "Fences", &wp_inf, UploadingFencesCount, 1, st );
		if(st)
		{
			++FencesCount;
			UploadingFencesCount = FencesCount;
		}
		else
			++UploadingFencesCount;
		
		UnLock_Fences();
		return true;
	}
	return false;
}

/*
	保存航点任务
	TIMEOUT: 线程同步超时时间

	返回值：
	true:保存成功
	false:保存失败（超时）
*/
bool saveFences( double TIMEOUT )
{
	if( UploadingFencesCount == 0 )
		return true;
	
	if( Lock_Fences(TIMEOUT) )
	{
		SaveVolatileParamGroup( "Fences" );
		FencesCount = UploadingFencesCount;
		
		UnLock_Fences();
		return true;
	}
	return false;
}

/*
	读取航点任务
	wp_ind：航点序号
	wp_inf：获取的航点信息
	TIMEOUT: 线程同步超时时间

	返回值：
	true:获取成功
	false:获取失败（无航点或参数表错误）
*/
bool ReadFence( uint16_t wp_ind, FenceInf* wp_inf, double TIMEOUT )
{
	if( wp_ind >= FencesCount )
		return false;
	
	PR_RESULT res = ReadVolatileParamGroup( "Fences", wp_inf, wp_ind, 1, TIMEOUT );
	if( res == PR_ERR )
		return false;
	return true;
}

void init_Fences()
{	
	char WPGroupName[17];	
	FencesCount = UploadingFencesCount = 0;
	
	VolatileParamGroupRegister( "Fences", 1, sizeof(FenceInf), 50 );
	uint16_t Fences_count;
	GetVolatileParamGroupParamCount( "Fences", &Fences_count );
	FencesCount = UploadingFencesCount = Fences_count;
}