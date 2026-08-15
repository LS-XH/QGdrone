#include "drv_CRC.hpp"
#include "Basic.hpp"

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"
#include "stream_buffer.h"

#define CRC_REV_IN_None (0<<5)
#define CRC_REV_IN8 (1<<5)
#define CRC_REV_IN16 (2<<5)
#define CRC_REV_IN32 (3<<5)

#define CRC_REV_OUT_None (0<<7)
#define CRC_REV_OUT (1<<7)

#define CRC_POLYSIZE_32bit (0<<3)
#define CRC_POLYSIZE_16bit (1<<3)
#define CRC_POLYSIZE_8bit (2<<3)
#define CRC_POLYSIZE_7bit (3<<3)
#define CRC_RESET (1<<0)

#define CRC_DR8 (*(uint8_t*)&CRC->DR)
#define CRC_DR16 (*(uint16_t*)&CRC->DR)
#define CRC_DR32 (*(uint32_t*)&CRC->DR)

/*CRC多项式配置*/
	//默认的CRC-32校验-比如以太网
	const CRC_Cfg cg_CRC_CONFIG_CRC32 = 
	{
		CRC_POLY_SIZE_32bit, 0x04C11DB7, 0xFFFFFFFF, 0xFFFFFFFF, true, true
	};
	
	//mavlink CRC-32
	const CRC_Cfg cg_CRC_CONFIG_CRC32_MAVLINK = 
	{
		CRC_POLY_SIZE_32bit, 0x04C11DB7, 0, 0, true, true
	};
	 
	//CRC-32/MPEG-2校验
	const CRC_Cfg cg_CRC_CONFIG_CRC32_MPEG2 = 
	{
		CRC_POLY_SIZE_32bit, 0x04C11DB7, 0xFFFFFFFF, 0x00000000, false, false
	};
	 
	//CRC-16/XMODEM校验
	const CRC_Cfg cg_CRC_CONFIG_CRC16_XMODEM = 
	{
		CRC_POLY_SIZE_16bit, 0x1021, 0x0000, 0x0000, false, false
	};
	
	//CRC-16/CCITT校验
	const CRC_Cfg cg_CRC_CONFIG_CRC16_CCITT = 
	{
		CRC_POLY_SIZE_16bit, 0x1021, 0x0000, 0x0000, true, true
	};
	
	//CRC-16/MODBUS校验
	const CRC_Cfg cg_CRC_CONFIG_CRC16_MODBUS = 
	{
		CRC_POLY_SIZE_16bit, 0x8005, 0xffff, 0x0000, true, true
	};
	
	//CRC-16/USB校验
	const CRC_Cfg cg_CRC_CONFIG_CRC16_USB = 
	{
		CRC_POLY_SIZE_16bit, 0x8005, 0xffff, 0xffff, true, true
	};
	
	//CRC-16/IBM校验
	const CRC_Cfg cg_CRC_CONFIG_CRC16_IBM = 
	{
		CRC_POLY_SIZE_16bit, 0x8005, 0x0000, 0x0000, true, true
	};
	 
	//CRC-8
	const CRC_Cfg cg_CRC_CONFIG_CRC8 = 
	{
		CRC_POLY_SIZE_8bit, 0x07, 0x00, 0x00, false, false
	};
	 
	//CRC-8/MAXIM校验
	const CRC_Cfg cg_CRC_CONFIG_CRC8_MAXIM = 
	{
		CRC_POLY_SIZE_8bit, 0x31, 0x00, 0x00, true, true
	};
	 
	//CRC-8/ROHC校验
	const CRC_Cfg cg_CRC_CONFIG_CRC8_ROHC = 
	{
		CRC_POLY_SIZE_8bit, 0x07, 0xFF, 0x00, true, true
	};
	 
	//CRC-8/ITU校验
	const CRC_Cfg cg_CRC_CONFIG_CRC8_ITU = 
	{
		CRC_POLY_SIZE_8bit, 0x07, 0x00, 0x55, false, false
	};
 
	//CRC-7/MMC校验
	const CRC_Cfg cg_CRC_CONFIG_CRC7_MMC = 
	{
		CRC_POLY_SIZE_7bit, 0x09, 0x00, 0x00, false, false
	};
/*CRC多项式配置*/

//发送完成标志
static EventGroupHandle_t events = xEventGroupCreate();
//CRC互斥锁
static SemaphoreHandle_t CRCSemphr = xSemaphoreCreateRecursiveMutex();

static inline void crc_calc_start( const CRC_Cfg& cfg, uint32_t crcLastRs=0, bool crcConinue=false )
{
	//配置CRC
	uint32_t CRC_CR = 0;
	if( cfg.isREFOUT )
		CRC_CR |= CRC_REV_OUT;
	if( cfg.isREFIN )
		CRC_CR |= CRC_REV_IN8;
	switch( cfg.Size )
	{
		case CRC_POLY_SIZE_7bit:
			CRC_CR |= CRC_POLYSIZE_7bit;
			break;
		case CRC_POLY_SIZE_8bit:
			CRC_CR |= CRC_POLYSIZE_8bit;
			break;
		case CRC_POLY_SIZE_16bit:
			CRC_CR |= CRC_POLYSIZE_16bit;
			break;
		case CRC_POLY_SIZE_32bit:
			CRC_CR |= CRC_POLYSIZE_32bit;
			break;
	}
	CRC->CR = CRC_CR | CRC_RESET;
	CRC->POL = cfg.Poly;
	if(crcConinue)
	{
		if(cfg.isREFOUT)
		{	//输出反转时
			//需要先反转上次输出到本次输入
			crcLastRs = __RBIT((uint32_t)crcLastRs);
			switch( cfg.Size )
			{
				case CRC_POLY_SIZE_7bit:
				case CRC_POLY_SIZE_8bit:
					crcLastRs = crcLastRs >> 24;
					break;
				case CRC_POLY_SIZE_16bit:
					crcLastRs = crcLastRs >> 16;
					break;
				default:
					break;
			}
		}
		CRC->INIT = crcLastRs;
	}
	else
		CRC->INIT = cfg.Init;
}
static inline bool crc_calcPart( const uint8_t* data, uint16_t length )
{
	//开始DMA传输CRC数据
	xEventGroupClearBits( events, 1 );
	DMA2->HIFCR = (1<<11);
	DMA2_Stream5->PAR = (uint32_t)data;
	DMA2_Stream5->NDTR = length;
	DMA2_Stream5->CR |= 1;
	
	EventBits_t rtbits = xEventGroupWaitBits( events, (1<<0), pdFALSE, pdTRUE, 1*configTICK_RATE_HZ );
	if( rtbits == (1<<0) )
		return true;
	else
		return false;
}
static inline void calc_crcResult( const CRC_Cfg& cfg, uint32_t* res )
{
	if(res)
	{
		switch( cfg.Size )
		{
			case CRC_POLY_SIZE_7bit:
				*res = CRC_DR8;
				*res ^= (uint8_t)cfg.XOROUT;
				break;
			case CRC_POLY_SIZE_8bit:
				*res = CRC_DR8;
				*res ^= (uint8_t)cfg.XOROUT;
				break;
			case CRC_POLY_SIZE_16bit:
				*res = CRC_DR16;
				*res ^= (uint16_t)cfg.XOROUT;
				break;
			case CRC_POLY_SIZE_32bit:
				*res = CRC_DR32;
				*res ^= (uint32_t)cfg.XOROUT;
				break;
		}
	}
}
	
static inline bool calc_crc( const CRC_Cfg& cfg, const uint8_t* data, uint16_t length, uint32_t* res, uint32_t crcLastRs=0, bool crcConinue=false )
{
	crc_calc_start( cfg, crcLastRs, crcConinue );
	if( crc_calcPart( data, length ) )
	{
		calc_crcResult( cfg, res );
		return true;
	}
	return false;
}

static inline bool lockCRC( double waitTime )
{
	uint32_t waitTicks;
	if( waitTime >= 0 )
		waitTicks = waitTime*configTICK_RATE_HZ;
	else
		waitTicks = portMAX_DELAY;
	
	//获取信号量
	if( xSemaphoreTakeRecursive( CRCSemphr , waitTicks ) == pdTRUE )
		return true;
	return false;
}

static inline void unlockCRC()
{
	xSemaphoreGiveRecursive(CRCSemphr);
}

static bool crcStarted = false;
#ifdef DCACHE_SIZE
	Static_AXIDMABuf uint8_t crcBuf[DCACHE_SIZE];
#endif
bool CRC_Calc( CRC_Cfg cfg, const uint8_t* data, uint16_t length, uint32_t* res, uint32_t crcLastRs, bool crcConinue, double waitTime )
{
	//获取信号量
	if( lockCRC(waitTime) )
	{
		if( crcStarted )
		{
			unlockCRC();
			return false;
		}
		
		//操作结果
		bool result = false;
		
#ifdef DCACHE_SIZE
		if( isDMABuf( (uint32_t)data ) )
		{	//缓冲区为非cache区域
			//直接用dma发送
#endif
			result = calc_crc( cfg, data, length, res, crcLastRs, crcConinue );
	
#ifdef DCACHE_SIZE
		}
		else if( ((uint32_t)data & 0x1f) == 0 )
		{	//缓冲区32字节对齐
			//直接dma送进缓冲区
			//将对应Cache数据清除
			SCB_CleanDCache_by_Addr((uint32_t*)data, length);
			result = calc_crc( cfg, data, length, res, crcLastRs, crcConinue );
		}
		else
		{	//将数据复制进入DMA缓冲区进行传送
			crc_calc_start( cfg, crcLastRs, crcConinue );
			int32_t current_length = length;
			while( current_length > 0 )
			{
				uint8_t current_cpyLength;
				if( DCACHE_SIZE < current_length )
					current_cpyLength = DCACHE_SIZE;
				else
					current_cpyLength = current_length;
				memcpy( crcBuf, &data[length-current_length], current_cpyLength );
				current_length -= current_cpyLength;
				if( crc_calcPart( crcBuf, current_cpyLength ) == false )
				{
					unlockCRC();
					return false;
				}
			}
			calc_crcResult( cfg, res );
			result = true;
		}
#endif
		unlockCRC();
		return result;
	}
  return false;
}

bool CRC_CalcStart( CRC_Cfg cfg, double waitTime )
{
	//获取信号量
	if( lockCRC(waitTime) )
	{
		if( crcStarted )
		{
			unlockCRC();
			return false;
		}
		//操作结果
		crc_calc_start(cfg);
		crcStarted = true;
		return true;
	}
  return false;
}
bool CRC_CalcPart( const uint8_t* data, uint16_t length, double waitTime )
{
	//获取信号量
	if( lockCRC(waitTime) )
	{
		if( !crcStarted )
		{
			unlockCRC();
			return false;
		}
		
		bool result;
		#ifdef DCACHE_SIZE
		if( isDMABuf( (uint32_t)data ) )
		{	//缓冲区为非cache区域
			//直接用dma发送
#endif
			result = crc_calcPart( data, length );
	
#ifdef DCACHE_SIZE
		}
		else if( ((uint32_t)data & 0x1f) == 0 )
		{	//缓冲区32字节对齐
			//直接dma送进缓冲区
			//将对应Cache数据清除
			SCB_CleanDCache_by_Addr((uint32_t*)data, length);
			result = crc_calcPart( data, length );
		}
		else	//不允许非DMABuf区域内存数据进行CRC转换
			result = false;
#endif
		unlockCRC();
		return result;
	}
  return false;
}
bool CRC_CalcResult( CRC_Cfg cfg, uint32_t* res, double waitTime )
{
	//获取信号量
	if( lockCRC(waitTime) )
	{
		if( !crcStarted )
		{
			unlockCRC();
			return false;
		}
		//操作结果
		calc_crcResult( cfg, res );
		crcStarted = false;
		unlockCRC();
		unlockCRC();
		return true;
	}
  return false;
}

extern "C" void DMA2_Stream5_IRQHandler()
{	//接收完成中断
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	DMA2->HIFCR = (1<<11) | (1<<10)  | (1<<9)  | (1<<8)  | (1<<6);
	xEventGroupSetBitsFromISR( events, 1, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void init_drv_CRC()
{
	//打开CRC时钟
	RCC->AHB4ENR |= (1<<19);
	//打开DMA2时钟
	RCC->AHB1ENR |= (1<<1);
	os_delay(0.01);
	
	CRC->CR = CRC_REV_OUT | CRC_REV_IN8 | CRC_POLYSIZE_16bit | CRC_RESET;
	CRC->POL = 0X1021;
	CRC->INIT = 0;
	
	//存储器-外设 MEM-MEM
	//DMA1_Stream1->CR = (3<<16) | (0<<13) | (1<<10) | (0<<9) | (0b00<<6) | (1<<4);
		
	DMA2_Stream5->CR = (0<<13) | (0<<11) | (1<<9) | (2<<6) | (1<<4);
	DMA2_Stream5->M0AR = (uint32_t)&CRC->DR;
	DMA2_Stream5->FCR = (1<<2) | (3<<0);
	NVIC_SetPriority( DMA2_Stream5_IRQn , 6 );
	NVIC_EnableIRQ(DMA2_Stream5_IRQn);
}