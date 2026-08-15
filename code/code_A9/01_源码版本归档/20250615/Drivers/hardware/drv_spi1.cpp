#include "drv_spi1.hpp"
#include "Basic.hpp"

#include "AC_Math.hpp"
#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"
#include "event_groups.h"
#include <limits>

// 配置SPI和DMA
#define SPI_NUM 1
#define SPI_FREQ 5e6
#define TX_DMA_NUM 1
#define TX_DMA_CHN 0
#define RX_DMA_NUM 1
#define RX_DMA_CHN 1
// 引脚配置
#define SPI_SCK_GPIO GPIOA
#define SPI_SCK_PIN 5
#define SPI_MISO_GPIO GPIOA
#define SPI_MISO_PIN 6
#define SPI_MOSI_GPIO GPIOA
#define SPI_MOSI_PIN 7

// SPI超时时间
#define SPI_TIMEOUT 1.0 * configTICK_RATE_HZ


// 自动生成配置名称
#define SPI_BASE NAME_CAT(SPI, SPI_NUM)
#define TX_DMA NAME_CAT(DMA, TX_DMA_NUM)
#define TX_DMA_STREAM NAME_CAT(DMA, NAME_CAT3(TX_DMA_NUM, _Stream, TX_DMA_CHN))
#define TX_DMA_IRQn NAME_CAT(DMA, NAME_CAT4(TX_DMA_NUM, _Stream, TX_DMA_CHN, _IRQn))
#define TX_DMA_MUX_CHAN ((DMAMUX_Channel_TypeDef *)(DMAMUX1_BASE + 0x0004UL*((TX_DMA_NUM-1)*8+TX_DMA_CHN)))
static __IO uint32_t* TX_DMA_IFCR = 0;
static uint32_t TX_DMA_IFCR_OFFSET = 0;
#define RX_DMA NAME_CAT(DMA, RX_DMA_NUM)
#define RX_DMA_STREAM NAME_CAT(DMA, NAME_CAT3(RX_DMA_NUM, _Stream, RX_DMA_CHN))
#define Rx_DMA_IRQn NAME_CAT(DMA, NAME_CAT4(RX_DMA_NUM, _Stream, RX_DMA_CHN, _IRQn))
#define RX_DMA_MUX_CHAN ((DMAMUX_Channel_TypeDef *)(DMAMUX1_BASE + 0x0004UL*((RX_DMA_NUM-1)*8+RX_DMA_CHN)))
static __IO uint32_t* RX_DMA_IFCR = 0;
static uint32_t RX_DMA_IFCR_OFFSET = 0;

// 互斥锁
static SemaphoreHandle_t mutex;
// 发送完成标志
static EventGroupHandle_t events;

/*DMA操作*/
	#define DMARxStream_Enable_IRQ ( NVIC_EnableIRQ(Rx_DMA_IRQn) )
	#define DMARxStream_Disable_IRQ ( NVIC_DisableIRQ(Rx_DMA_IRQn) )
/*DMA操作*/

/*SPI操作*/
	// spi缓冲区
	Static_AXIDMABuf uint8_t trCache[256];

	/*上锁保证通信连续性
			上锁之后必须解锁
			Sync_waitTime：超时时间
	*/
	bool NAME_CAT(lock_spi, SPI_NUM)(double Sync_waitTime)
	{
		uint32_t Sync_waitTicks;
		if (Sync_waitTime >= 0)
			Sync_waitTicks = Sync_waitTime * configTICK_RATE_HZ;
		else
			Sync_waitTicks = portMAX_DELAY;
		if (xSemaphoreTakeRecursive(mutex, Sync_waitTicks) == pdTRUE)
			return true;
		return false;
	}
	void NAME_CAT(unlock_spi, SPI_NUM)()
	{
		xSemaphoreGiveRecursive(mutex);
	}

	//在SPI发送数据
	SPI_Result NAME_CAT3(spi, SPI_NUM, _write)(
		const uint8_t *tx_data, const uint8_t tx_size, const uint8_t tx_buffer_size, 
		double TIMEOUT)
	{
		if (tx_data == 0 || tx_size == 0)
        return SPI_Result_InvalidArgs;

    if (NAME_CAT(lock_spi, SPI_NUM)(TIMEOUT))
    {
			//关闭DMA通道
			TX_DMA_STREAM->CR &= ~(1<<0);
			RX_DMA_STREAM->CR &= ~(1<<0);
			// 清空event状态
			xEventGroupClearBits(events, 0xff);
			
			// 初始化发送dma缓冲区
			const uint8_t *data_buff;
			#ifdef DCACHE_SIZE
			uint32_t aligned_end = CACHELINE_ALIGN_UP((uint32_t)tx_data + tx_size);
			uint32_t aligned_size = aligned_end - (uint32_t)tx_data;
			if( isDMABuf((uint32_t)tx_data) )
			#endif
			{	// 无CACHE
				data_buff = tx_data;
			}
			#ifdef DCACHE_SIZE
			else if( (uint32_t)tx_data % DCACHE_SIZE ==0 && aligned_size<=tx_buffer_size )
			{	// CACHE区域
				data_buff = tx_data;
				SCB_CleanDCache_by_Addr((uint32_t*)tx_data, aligned_size);
			}
			else
			{
				data_buff = trCache;
			}
			#endif
			
			//关闭SPI外设
			SPI_BASE->CR1 &= ~(1<<0);
			//关闭DMA Request
			SPI_BASE->CFG1 &= ~( (1<<15) | (1<<14) );
			//清空DMA状态
			*TX_DMA_IFCR = ( (1<<5) | (1<<4)  | (1<<3)  | (1<<2)  | (1<<0) ) << TX_DMA_IFCR_OFFSET;
			*RX_DMA_IFCR = ( (1<<5) | (1<<4)  | (1<<3)  | (1<<2)  | (1<<0) ) << RX_DMA_IFCR_OFFSET;
			//设置DMA存储器地址
			TX_DMA_STREAM->M0AR = (uint32_t)data_buff;
			RX_DMA_STREAM->M0AR = (uint32_t)trCache;
			//清空SPI状态
			SPI_BASE->IFCR = (1<<9) | (1<<4);
			//设置DMA传输数量
			TX_DMA_STREAM->NDTR = RX_DMA_STREAM->NDTR = SPI_BASE->CR2 = tx_size;
			//使能Rx DMA
			RX_DMA_STREAM->CR |= (1<<0);
			//打开Rx DMA Request
			SPI_BASE->CFG1 |= (1<<14);
			//使能Tx DMA
			TX_DMA_STREAM->CR |= (1<<0);
			//打开Tx DMA Request
			SPI_BASE->CFG1 |= (1<<15);
			//打开SPI外设开始传输
			SPI_BASE->CR1 |= (1<<0);
			SPI_BASE->CR1 |= (1<<9);
			//开启中断
			DMARxStream_Enable_IRQ;
			
			// 等待传输完成
			uint32_t r_status = xEventGroupWaitBits(
					events,
					(1<<5) | (1<<4)  | (1<<3)  | (1<<2)  | (1<<0),
					pdTRUE, pdFALSE, SPI_TIMEOUT);
			// 关闭中断
			DMARxStream_Disable_IRQ;
			
			// 解锁外设
			NAME_CAT(unlock_spi, SPI_NUM)();
			if (r_status & (1<<4))
				return SPI_Result_Success;
			return SPI_Result_Fail;
    }
    return SPI_Result_timeout;
	}
	
	//在SPI接收数据
	SPI_Result NAME_CAT3(spi, SPI_NUM, _writeNread)(
    const uint8_t *tx_data, const uint8_t tx_size, const uint8_t tx_buffer_size,
    uint8_t *rx_data, const uint8_t rx_buffer_size,
    double TIMEOUT)
	{
		if (tx_data == 0 || tx_size == 0 || tx_buffer_size < tx_size || rx_data == 0 || rx_buffer_size < tx_size)
			return SPI_Result_InvalidArgs;
		
		if (NAME_CAT(lock_spi, SPI_NUM)(TIMEOUT))
    {
			//关闭DMA通道
			TX_DMA_STREAM->CR &= ~(1<<0);
			RX_DMA_STREAM->CR &= ~(1<<0);
			// 清空event状态
			xEventGroupClearBits(events, 0xff);
			
			// 初始化发送tx dma缓冲区
			const uint8_t *tx_data_buff;
			#ifdef DCACHE_SIZE
			uint32_t aligned_end = CACHELINE_ALIGN_UP((uint32_t)tx_data + tx_size);
			uint32_t aligned_size = aligned_end - (uint32_t)tx_data;
			if( isDMABuf((uint32_t)tx_data) )
			{	// 无CACHE
				tx_data_buff = tx_data;
				SCB_CleanDCache_by_Addr((uint32_t*)tx_data, aligned_size);
			}
			#else
			tx_data_buff = tx_data;
			#endif
			#ifdef DCACHE_SIZE
			else if( (uint32_t)tx_data % DCACHE_SIZE ==0 && aligned_size<=tx_buffer_size )
			{	// CACHE区域
				tx_data_buff = tx_data;
				SCB_CleanDCache_by_Addr((uint32_t*)tx_data, aligned_size);
			}
			else
			{
				tx_data_buff = trCache;
			}
			#endif
			
			// 初始化发送rx dma缓冲区
			const uint8_t *rx_data_buff;
			#ifdef DCACHE_SIZE
			aligned_end = CACHELINE_ALIGN_UP((uint32_t)rx_data + tx_size);
			aligned_size = aligned_end - (uint32_t)rx_data;
			if( isDMABuf((uint32_t)rx_data) )
			{	// 无CACHE
				rx_data_buff = rx_data;
				SCB_InvalidateDCache_by_Addr((uint32_t*)rx_data, aligned_size);
			}
			#else
			rx_data_buff = rx_data;
			#endif
			#ifdef DCACHE_SIZE
			else if( (uint32_t)rx_data % DCACHE_SIZE ==0 && aligned_size<=rx_buffer_size )
			{	// CACHE区域
				rx_data_buff = rx_data;
				SCB_InvalidateDCache_by_Addr((uint32_t*)rx_data, aligned_size);
			}
			else
			{
				rx_data_buff = trCache;
			}
			#endif
			
			//关闭SPI外设
			SPI_BASE->CR1 &= ~(1<<0);
			//关闭DMA Request
			SPI_BASE->CFG1 &= ~( (1<<15) | (1<<14) );
			//清空DMA状态
			*TX_DMA_IFCR = ( (1<<5) | (1<<4)  | (1<<3)  | (1<<2)  | (1<<0) ) << TX_DMA_IFCR_OFFSET;
			*RX_DMA_IFCR = ( (1<<5) | (1<<4)  | (1<<3)  | (1<<2)  | (1<<0) ) << RX_DMA_IFCR_OFFSET;
			//设置DMA存储器地址
			TX_DMA_STREAM->M0AR = (uint32_t)tx_data_buff;
			RX_DMA_STREAM->M0AR = (uint32_t)rx_data_buff;
			//清空SPI状态
			SPI_BASE->IFCR = (1<<9) | (1<<4);
			//设置DMA传输数量
			TX_DMA_STREAM->NDTR = RX_DMA_STREAM->NDTR = SPI_BASE->CR2 = tx_size;
			//使能Rx DMA
			RX_DMA_STREAM->CR |= (1<<0);
			//打开Rx DMA Request
			SPI_BASE->CFG1 |= (1<<14);
			//使能Tx DMA
			TX_DMA_STREAM->CR |= (1<<0);
			//打开Tx DMA Request
			SPI_BASE->CFG1 |= (1<<15);
			
			//打开SPI外设开始传输
			SPI_BASE->CR1 |= (1<<0);
			SPI_BASE->CR1 |= (1<<9);
			
			//开启中断
			DMARxStream_Enable_IRQ;
			
			// 等待传输完成
			uint32_t r_status = xEventGroupWaitBits(
					events,
					(1<<5) | (1<<4)  | (1<<3)  | (1<<2)  | (1<<0),
					pdTRUE, pdFALSE, SPI_TIMEOUT);
			// 关闭中断
			DMARxStream_Disable_IRQ;
			
			// 如数据在noncache缓冲区则复制回来
			if (r_status & (1<<4))
			{
				if (rx_data_buff != rx_data)
					memcpy(rx_data, rx_data_buff, tx_size);
			}
			
			// 解锁外设
			NAME_CAT(unlock_spi, SPI_NUM)();
			if (r_status & (1<<4))
				return SPI_Result_Success;
			return SPI_Result_Fail;
    }
    return SPI_Result_timeout;
	}
/*SPI操作*/

// DMA Rx中断
extern "C" void NAME_CAT(DMA, NAME_CAT4(RX_DMA_NUM, _Stream, RX_DMA_CHN, _IRQHandler))()
{
	// 清空DMA状态
	uint32_t ISR = *(RX_DMA_IFCR - 2);
	ISR >>= RX_DMA_IFCR_OFFSET;
	*RX_DMA_IFCR = ( (1<<5) | (1<<4)  | (1<<3)  | (1<<2)  | (1<<0) ) << RX_DMA_IFCR_OFFSET;
	// 关闭DMA中断
	DMARxStream_Disable_IRQ;
	
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xEventGroupSetBitsFromISR(events, ISR, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void NAME_CAT(init_drv_spi, SPI_NUM)()
{
	/*SPI初始化*/
		//打开SPI时钟
		#if SPI_NUM==1
			RCC->APB2ENR |= (1<<12);
			uint32_t spi_clock = APB2CLK;
			const uint32_t dma_tx_mux = 38;
			const uint32_t dma_rx_mux = 37;
			const uint32_t sck_af = 5;
			const uint32_t miso_af = 5;
			const uint32_t mosi_af = 5;
		#elif SPI_NUM==4
			RCC->APB2ENR |= (1<<13);
			uint32_t spi_clock = APB2CLK;
			const uint32_t dma_tx_mux = 84;
			const uint32_t dma_rx_mux = 83;
			const uint32_t sck_af = 5;
			const uint32_t miso_af = 5;
			const uint32_t mosi_af = 5;
		#else
			#error "Spi selection error."
		#endif
		vTaskDelay(1);
		
		uint32_t freq_splt = round(log2((double)spi_clock / SPI_FREQ)) - 1;
		freq_splt &= 0x7f;
		SPI_BASE->CR1 = (1<<12);
		SPI_BASE->CFG1 = (freq_splt<<28) | (1<<15) | (1<<14) | (7<<0);
		SPI_BASE->CFG2 = (1<<31) | (1<<30) | (1<<26) | (1<<25) | (1<<24) | (1<<22);
		SPI_BASE->IFCR = (1<<9);
		SPI_BASE->CR1 = (1<<12);
	/*SPI初始化*/
	
	/*DMA初始化*/
		//打开DMA时钟
		RCC->AHB1ENR |= (1<<(RX_DMA_NUM-1)) | (1<<(TX_DMA_NUM-1));
		vTaskDelay(1);
		
		//SPI TX DMA
		TX_DMA_STREAM->PAR = (uint32_t)&SPI_BASE->TXDR;
		TX_DMA_STREAM->NDTR = 2;
		TX_DMA_MUX_CHAN->CCR = (dma_tx_mux<<0);
		TX_DMA_STREAM->CR = (3<<16) | (0<<13) | (1<<10) | (0<<9) | (0b01<<6);
		TX_DMA_STREAM->FCR = (1<<2) | (3<<0);
		// Tx IFCR
		TX_DMA_IFCR = TX_DMA_CHN >= 4 ? &TX_DMA->HIFCR : &TX_DMA->LIFCR;
		TX_DMA_IFCR_OFFSET = TX_DMA_CHN;
		if( TX_DMA_IFCR_OFFSET >= 4 )
			TX_DMA_IFCR_OFFSET -= 4;
		if( TX_DMA_IFCR_OFFSET >= 2 )
			TX_DMA_IFCR_OFFSET = 16 + 6*( TX_DMA_IFCR_OFFSET - 2);
		else
			TX_DMA_IFCR_OFFSET = 6*TX_DMA_IFCR_OFFSET;
		
		//SPI RX DMA
		RX_DMA_STREAM->PAR = (uint32_t)&SPI_BASE->RXDR;
		RX_DMA_STREAM->NDTR = 2;
		RX_DMA_MUX_CHAN->CCR = (dma_rx_mux<<0);
		RX_DMA_STREAM->CR = (3<<16) | (0<<13) | (1<<10) | (0<<9) | (0b00<<6) | (1<<4) | (1<<2);
		RX_DMA_STREAM->FCR = (1<<2) | (3<<0);
		NVIC_SetPriority( Rx_DMA_IRQn , IRQPrioriy_SensorSpi );
		// Rx IFCR
		RX_DMA_IFCR = RX_DMA_CHN >= 4 ? &RX_DMA->HIFCR : &RX_DMA->LIFCR;
		RX_DMA_IFCR_OFFSET = RX_DMA_CHN;
		if( RX_DMA_IFCR_OFFSET >= 4 )
			RX_DMA_IFCR_OFFSET -= 4;
		if( RX_DMA_IFCR_OFFSET >= 2 )
			RX_DMA_IFCR_OFFSET = 16 + 6*( RX_DMA_IFCR_OFFSET - 2);
		else
			RX_DMA_IFCR_OFFSET = 6*RX_DMA_IFCR_OFFSET;
	/*DMA初始化*/
		
	/*IO初始化
	*/
		//打开GPIO时钟
		uint32_t GPIO_BASE_G = GPIOB - GPIOA;
		RCC->AHB4ENR |= 
			( 1 << (SPI_SCK_GPIO-GPIOA)/GPIO_BASE_G ) |
			( 1 << (SPI_MISO_GPIO-GPIOA)/GPIO_BASE_G ) |
			( 1 << (SPI_MOSI_GPIO-GPIOA)/GPIO_BASE_G );
		vTaskDelay(1);
	
		//设置Moder复用功能(SPI)
		set_register( SPI_SCK_GPIO->MODER , 0b10 , 2*SPI_SCK_PIN , 2 );
		set_register( SPI_MISO_GPIO->MODER , 0b10 , 2*SPI_MISO_PIN , 2 );
		set_register( SPI_MOSI_GPIO->MODER , 0b10 , 2*SPI_MOSI_PIN , 2 );
		
		//SCLK、MOSI、CS推挽输出，MISO开漏上拉
		set_register( SPI_SCK_GPIO->OTYPER , 0 , 1*SPI_SCK_PIN , 1 );
		set_register( SPI_MISO_GPIO->OTYPER , 1 , 1*SPI_MISO_PIN , 1 );
		set_register( SPI_MOSI_GPIO->OTYPER , 0 , 1*SPI_MOSI_PIN , 1 );
		set_register( SPI_SCK_GPIO->PUPDR , 0 , 2*SPI_SCK_PIN , 2 );
		set_register( SPI_MISO_GPIO->PUPDR , 1 , 2*SPI_MISO_PIN , 2 );
		set_register( SPI_MOSI_GPIO->PUPDR , 0 , 2*SPI_MOSI_PIN , 2 );
		
		//设置速度
		set_register( SPI_SCK_GPIO->OSPEEDR , 1 , 2*SPI_SCK_PIN , 2 );
		set_register( SPI_MISO_GPIO->OSPEEDR , 1 , 2*SPI_MISO_PIN , 2 );
		set_register( SPI_MOSI_GPIO->OSPEEDR , 1 , 2*SPI_MOSI_PIN , 2 );
		
		//设置复用功能
		uint8_t afOffset;
		afOffset = 4*SPI_SCK_PIN;
		set_register( SPI_SCK_GPIO->AFR[afOffset>=32?1:0] , sck_af , afOffset>=32?afOffset-32:afOffset , 4 );
		afOffset = 4*SPI_MISO_PIN;
		set_register( SPI_MISO_GPIO->AFR[afOffset>=32?1:0] , miso_af , afOffset>=32?afOffset-32:afOffset , 2 );
		afOffset = 4*SPI_MOSI_PIN;
		set_register( SPI_MOSI_GPIO->AFR[afOffset>=32?1:0] , mosi_af , afOffset>=32?afOffset-32:afOffset , 2 );
	/*IO初始化*/
		
	// 生成互斥锁
	mutex = xSemaphoreCreateRecursiveMutex();
	// 生成事件
	events = xEventGroupCreate();
}