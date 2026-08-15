#pragma once

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdbool.h>
		
//名称连接宏
#define _LINK(a,b) a##b
#define _LINK3(a,b,c) a##b##c
#define NAME_CAT(a,b) _LINK(a,b)
#define NAME_CAT3(a,b,c) NAME_CAT(NAME_CAT(a,b),c)
#define NAME_CAT4(a,b,c,d) NAME_CAT(NAME_CAT(a,b),NAME_CAT(c,d))
#define NAME_CAT5(a,b,c,d,e) NAME_CAT(NAME_CAT(a,b),NAME_CAT3(c,d,e))

//DCache相关定义
#ifdef DCACHE_SIZE
	//DMA缓冲区定义
	#define Aligned_DMABuf __attribute__ ((aligned (DCACHE_SIZE)))
	#define Static_AXIDMABuf static __attribute__((section(".AXI_DMA_RAM")))
	//判断是否为DMA缓冲区（无cache内存）
	#define isDMABuf(addr) ( (addr>=0x24070000 && addr<0x24080000) || (addr>=0x20000000 && addr<0x20020000) || (addr>=0x8000000 && addr<0x8200000) )
	
	#define CACHELINE_ALIGN_DOWN(n) ((uint32_t)(n) & ~(DCACHE_SIZE - 1U))
	#define CACHELINE_ALIGN_UP(n)   CACHELINE_ALIGN_DOWN((uint32_t)(n) + DCACHE_SIZE - 1U)
#else
	#define Aligned_DMABuf
	#define Static_AXIDMABuf static
	#define isDMABuf(addr) (1)
#endif
//静态DTCM内存
#define Static_DTCMBuf static __attribute__((section(".DTCM")))
//静态AXI内存
#define Static_AXIBuf static __attribute__((section(".AXI_RAM")))
//静态SRAM1内存
#define Static_SRAM1Buf static __attribute__((section(".SRAM1")))
//静态SRAM2内存
#define Static_SRAM2Buf static __attribute__((section(".SRAM2")))
//静态SRAM3内存
#define Static_SRAM3Buf static __attribute__((section(".SRAM3")))
	
//操作系统优先级定义
#define SysPriority_MeasurementSystem_Integral 6
#define SysPriority_MeasurementSystem 5
#define SysPriority_ControlSystem 4
#define SysPriority_ExtSensor 3
#define SysPriority_SafeTask 2
#define SysPriority_UserTask 1

//中断优先级定义
#define IRQPrioriy_USB 6
#define IRQPrioriy_SensorSpi 4
#define IRQPrioriy_ImuSensor 3

#define isvalid(x) (!std::isnan(x) && !std::isinf(x))

// HTL
bool HTL_isEnabled();

#ifdef __cplusplus
	}
#endif