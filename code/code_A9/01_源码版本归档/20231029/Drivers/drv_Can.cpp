#include "drv_Can.hpp"
#include "stm32h743xx.h"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "stm32h7xx_hal.h"
#include "freertos.h"
#include "queue.h"
#include <map>
#include "Parameters.hpp"
#include "Commulink.hpp"

using namespace std;
FDCAN_HandleTypeDef hfdcan;

/*CanMailBox*/
	static map<uint32_t, CanMailBox*> CanMailIdFilterMap;
	
	CanMailBox::CanMailBox( uint16_t Capacity, 
								CanId* Ids, uint16_t Ids_count )
	{
		if( Ids )
		{
			for( uint16_t i = 0; i < Ids_count; ++i )
			{	//Id-MailBox关系放置在Map中
				//Map存放Id（标准Id+0x20000000）
				pair<uint32_t, CanMailBox*> p;
				if( Ids[i].IdType )
					p.first = Ids[i].Identifier;	//扩展帧
				else
					p.first = Ids[i].Identifier + 0x20000000;	//标准帧
				p.second = this;
				CanMailIdFilterMap.insert(p);
			}
			packet_queue = xQueueCreate( Capacity, sizeof(CanPacket) );
		}
		else
			Ids_count = 0;
		this->rx_packets_cnt = Ids_count;
		this->mail_overflow_cnt = 0;
	}
	
	bool CanMailBox::receiveMail( CanPacket* mail, double waitTime )
	{
		if( this->rx_packets_cnt == 0 )
			return false;
		
		uint32_t waitTicks;
		if( waitTime >= 0 )
			waitTicks = waitTime*configTICK_RATE_HZ;
		else
			waitTicks = portMAX_DELAY;
		BaseType_t res = xQueueReceive( this->packet_queue, mail, waitTicks );
		if( res == pdTRUE )
			return true;
		return false;
	}
	
	/*邮件发送*/
		//发送互斥锁
		static SemaphoreHandle_t TxSemphr;
		//发送完成标志
		static EventGroupHandle_t events = xEventGroupCreate();
	
		static inline bool Lock_Can( double Sync_waitTime )
		{
			uint32_t Sync_waitTicks;
			if( Sync_waitTime > 0 )
				Sync_waitTicks = Sync_waitTime*configTICK_RATE_HZ;
			else
				Sync_waitTicks = portMAX_DELAY;
			if( xSemaphoreTakeRecursive( TxSemphr , Sync_waitTicks ) == pdTRUE )
				return true;
			return false;
		}
		static inline void Unlock_Can()
		{
			xSemaphoreGiveRecursive( TxSemphr );
		}
		bool CanMailBox::SendMail( CanPacket mail, double Send_waitTime, double Sync_waitTime )
		{
			uint32_t mail_DataLength;
			switch(mail.DataLength)
			{
				case 0:
					mail_DataLength = FDCAN_DLC_BYTES_0;
					break;
				case 1:
					mail_DataLength = FDCAN_DLC_BYTES_1;
					break;
				case 2:
					mail_DataLength = FDCAN_DLC_BYTES_2;
					break;
				case 3:
					mail_DataLength = FDCAN_DLC_BYTES_3;
					break;
				case 4:
					mail_DataLength = FDCAN_DLC_BYTES_4;
					break;
				case 5:
					mail_DataLength = FDCAN_DLC_BYTES_5;
					break;
				case 6:
					mail_DataLength = FDCAN_DLC_BYTES_6;
					break;
				case 7:
					mail_DataLength = FDCAN_DLC_BYTES_7;
					break;
				case 8:
					mail_DataLength = FDCAN_DLC_BYTES_8;
					break;
				case 12:
					mail_DataLength = FDCAN_DLC_BYTES_12;
					break;
				case 16:
					mail_DataLength = FDCAN_DLC_BYTES_16;
					break;
				case 20:
					mail_DataLength = FDCAN_DLC_BYTES_20;
					break;
				case 24:
					mail_DataLength = FDCAN_DLC_BYTES_24;
					break;
				case 32:
					mail_DataLength = FDCAN_DLC_BYTES_32;
					break;
				case 48:
					mail_DataLength = FDCAN_DLC_BYTES_48;
					break;
				case 64:
					mail_DataLength = FDCAN_DLC_BYTES_64;
					break;
				default:
					return false;
					break;
			}
			
			uint32_t Send_waitTicks;
			if( Send_waitTime >= 0 )
				Send_waitTicks = Send_waitTime*configTICK_RATE_HZ;
			else
				Send_waitTicks = portMAX_DELAY;
			
			if( Lock_Can( Sync_waitTime ) )
			{//获取信号量
				uint16_t RealByteSend = 0;
				
				while( (FDCAN1->TXFQS & (uint32_t)(1<<21)) && Send_waitTicks )
				{	//等待FIFO未满
					vTaskDelay(1);
					--Send_waitTicks;
				}
				bool res = false;
				if( (FDCAN1->TXFQS & (uint32_t)(1<<21)) != (uint32_t)(1<<21) )
				{//发送FIFO未满
					FDCAN_TxHeaderTypeDef TxHeader;
					TxHeader.Identifier = mail.IdType==0 ? (mail.Identifier&0x7ff) : (mail.Identifier&0x1FFFFFFF);
					TxHeader.IdType = mail.IdType==0 ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID;
					TxHeader.TxFrameType = mail.FrameType==0 ? FDCAN_DATA_FRAME : FDCAN_REMOTE_FRAME;
					TxHeader.DataLength = mail_DataLength;
					TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
					TxHeader.BitRateSwitch = FDCAN_BRS_ON;
					TxHeader.FDFormat = mail.FDFormat==0 ? FDCAN_FRAME_CLASSIC : FDCAN_FRAME_FD_BRS;
					TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
					TxHeader.MessageMarker = 0;	
					
					HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &TxHeader, mail.data);
					res = true;
				}
				//释放信号量
				Unlock_Can();
				return res;
			}
			else
				return false;
		}
	/*邮件发送*/

	void CanMailBox::CanRxTCB( void *pvParameter1, uint32_t ulParameter2 )
	{
		FDCAN_RxHeaderTypeDef RxHeader;
		uint8_t datas[64];
		CanPacket packet;
		for( uint8_t i = 0; i < 5; ++i )
		{
			if( HAL_FDCAN_GetRxMessage(&hfdcan, FDCAN_RX_FIFO0, &RxHeader, packet.data) == HAL_ERROR )
				break;
			
			//求Map存放Id（标准Id+0x20000000）
			uint32_t MapId = RxHeader.Identifier;
			if( RxHeader.IdType == FDCAN_STANDARD_ID )
				MapId += 0x20000000;
			
			map<uint32_t, CanMailBox*>::iterator it = CanMailIdFilterMap.find(MapId);
			if( it != CanMailIdFilterMap.end() )
			{	//找到对应邮箱
				CanMailBox* mail_box = it->second;
				
				packet.IdType = RxHeader.IdType==FDCAN_STANDARD_ID ? 0 : 1;
				packet.DataLength = RxHeader.DataLength >> 16;
				packet.FrameType = RxHeader.RxFrameType==FDCAN_DATA_FRAME ? 0 : 1;
				packet.FDFormat = RxHeader.FDFormat==FDCAN_CLASSIC_CAN ? 0 : 1;
				packet.Identifier = RxHeader.Identifier;
				
				BaseType_t res = xQueueSend( mail_box->packet_queue, &packet, 0 );
				if( res != pdTRUE )
					++mail_box->mail_overflow_cnt;
			}
		}
	}
/*CanMailBox*/



#define CANVER 0
#if CANVER==1
	#define FDCANx_Enable GPIOD->BSRR = ( 1 << 21 )
	#define FDCANx_Disable GPIOD->BSRR = ( 1 << 5 )
#elif CANVER==0
	#define FDCANx_Enable GPIOD->BSRR = ( 1 << 19 )
	#define FDCANx_Disable GPIOD->BSRR = ( 1 << 3 )
#endif

#define FDCANx_CLK_ENABLE()         __HAL_RCC_FDCAN_CLK_ENABLE()
#define FDCANx_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE()
#define FDCANx_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE()

#define FDCANx_FORCE_RESET()   __HAL_RCC_FDCAN_FORCE_RESET()
#define FDCANx_RELEASE_RESET() __HAL_RCC_FDCAN_RELEASE_RESET()

/* Definition for FDCANx Pins */    
#define FDCANx_TX_PIN       GPIO_PIN_1
#define FDCANx_TX_GPIO_PORT GPIOD
#define FDCANx_TX_AF        GPIO_AF9_FDCAN1
#define FDCANx_RX_PIN       GPIO_PIN_0
#define FDCANx_RX_GPIO_PORT GPIOD
#define FDCANx_RX_AF        GPIO_AF9_FDCAN1       

/* Definition for FDCANx's NVIC IRQ and IRQ Handlers */
#define FDCANx_IT0_IRQn   FDCAN1_IT0_IRQn
#define FDCANx_IT1_IRQn   FDCAN1_IT1_IRQn
#define FDCANx_IRQHandler FDCAN1_IRQHandler


//FDCAN配置
#define TxFifoElmtsCnt 32  //TxFifo元素个数,0-32
#define RxFifo0ElmtsCnt 64 //RxFifo0元素个数,0-64
#define RxFifo1ElmtsCnt 0 //RxFifo1元素个数,0-64
#define Enable_Standard_Filter 0 //是否开启标准帧接收过滤
#define Enable_Extended_Filter 0 //是否开启拓展帧接收过滤

#if Enable_Standard_Filter 

//需要接收的标准消息ID, 0-0x7FF
uint16_t Msg_Filter_Standard_ID[]=
{         

};
#endif

#if Enable_Extended_Filter 

//需要接收的拓展消息ID, 0-0x20000000
uint32_t Msg_Filter_Extended_ID[]=
{
  0x500, 0x501, 0x600, 0x601
};
							 
#endif								 


//发送完成中断回调函数
extern "C" void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
	BaseType_t HigherPriorityTaskWoken = pdFALSE;
	xEventGroupSetBitsFromISR( events, (1<<0) , &HigherPriorityTaskWoken );	
}


//接收中断回调函数
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	BaseType_t HigherPriorityTaskWoken = pdFALSE;
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {//接收到新消息		
		xLPDaemonTaskCallFromISR( CanMailBox::CanRxTCB, 0, 0, &HigherPriorityTaskWoken );
  }
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_WATERMARK) != RESET)
  {//接收缓冲区已满
		
		//暂不做处理
	}
	portYIELD_FROM_ISR(HigherPriorityTaskWoken);
}

extern "C" void FDCAN1_IT0_IRQHandler(void)
{
  HAL_FDCAN_IRQHandler(&hfdcan);
}

extern "C" void FDCAN1_IT1_IRQHandler(void)
{
  HAL_FDCAN_IRQHandler(&hfdcan);
}

static inline bool solve_req_baud( double pclk, double req_baud, 
																	uint16_t* Prescaler, uint16_t* TimeSeg1, uint16_t* TimeSeg2, uint16_t* SyncJumpWidth )
{
	double rp_multiple = pclk / req_baud;
	double min_err = 1;
	int16_t optimal_Seg1pSeg2 = -1;
	for( uint16_t i = 48; i >= 4; --i )
	{	//找最优分频系数（余数最小）
		double p = rp_multiple / (double)(i+1);
		double err = p - (int32_t)p;
		if( err > 0.5 )
			err = 1.0 - err;
		if( err < 0.00001 )
		{
			min_err = err;
			optimal_Seg1pSeg2 = i;
			break;
		}
		if( err < min_err )
		{
			min_err = err;
			optimal_Seg1pSeg2 = i;
		}
	}
	
	if( min_err > 0.1 )
		return false;
	*Prescaler = round(rp_multiple / double(optimal_Seg1pSeg2+1));
	*TimeSeg1 = round( double(optimal_Seg1pSeg2+1) * 0.7 ) - 1;
	if( *TimeSeg1 == 0 )
		*TimeSeg1 = 1;
	else if( *TimeSeg1 == optimal_Seg1pSeg2 )
		*TimeSeg1 -= 1;
	*TimeSeg2 = optimal_Seg1pSeg2 - *TimeSeg1;
	*SyncJumpWidth = round(200/(*Prescaler));
	if( *SyncJumpWidth < 1 )
		*SyncJumpWidth = 1;
	else if( *SyncJumpWidth >127 )
		*SyncJumpWidth = 127;
	return true;
}

void init_drv_Can(void)
{
	/*端口初始化*/
	TxSemphr = xSemaphoreCreateRecursiveMutex();
	//RxSemphr = xSemaphoreCreateMutex();

	/*端口初始化*/	
	
	HAL_FDCAN_DeInit(&hfdcan);            
  hfdcan.Instance = FDCAN1;
  hfdcan.Init.FrameFormat = FDCAN_FRAME_FD_BRS;   //帧格式
  hfdcan.Init.Mode = FDCAN_MODE_NORMAL;   				//工作模式
  hfdcan.Init.AutoRetransmission = DISABLE;       //关闭自动重传
  hfdcan.Init.TransmitPause = DISABLE;            //关闭传输暂停
  hfdcan.Init.ProtocolException = ENABLE;         //开启协议异常处理

/*生成波特率*/
	#ifdef SYSFREQ480
		uint32_t pclk = 240e6;
	#else
		uint32_t pclk = 200e6;
	#endif
	double req_NominalFreq = 500e3;
	double req_DataFreq = 5e6;
	CANConfig CAN_config;
	if( ReadParamGroup( "CAN", (uint64_t*)&CAN_config, 0 ) == PR_OK )
	{
		req_NominalFreq = CAN_config.Baud[0];
		req_DataFreq = CAN_config.DataBaud[0];
	}
	
	uint16_t Prescaler; uint16_t TimeSeg1; uint16_t TimeSeg2; uint16_t SyncJumpWidth;
	if( solve_req_baud( pclk, req_NominalFreq, 
											&Prescaler, &TimeSeg1, &TimeSeg2, &SyncJumpWidth ) == false )
	{
		solve_req_baud( pclk, 500e3, 
										&Prescaler, &TimeSeg1, &TimeSeg2, &SyncJumpWidth );
	}
	hfdcan.Init.NominalPrescaler = Prescaler;
  hfdcan.Init.NominalSyncJumpWidth = SyncJumpWidth;
  hfdcan.Init.NominalTimeSeg1 = TimeSeg1;
  hfdcan.Init.NominalTimeSeg2 = TimeSeg2;
	
	if( solve_req_baud( pclk, req_DataFreq, 
											&Prescaler, &TimeSeg1, &TimeSeg2, &SyncJumpWidth ) == false )
	{
		solve_req_baud( pclk, 5e6, 
										&Prescaler, &TimeSeg1, &TimeSeg2, &SyncJumpWidth );
	}
	hfdcan.Init.DataPrescaler = Prescaler;
  hfdcan.Init.DataSyncJumpWidth = SyncJumpWidth;
  hfdcan.Init.DataTimeSeg1 = TimeSeg1;
  hfdcan.Init.DataTimeSeg2 = TimeSeg2;
/*生成波特率*/

  //标准帧滤波器个数，0~128
#if Enable_Standard_Filter	
	uint16_t msg_count_standard = sizeof(Msg_Filter_Standard_ID) / sizeof(uint16_t);
	hfdcan.Init.StdFiltersNbr = msg_count_standard % 2 ? msg_count_standard / 2 + 1 : msg_count_standard / 2;
	if( msg_count_standard % 2 != 0 )
		msg_count_standard--; 
#else
  hfdcan.Init.StdFiltersNbr = 0;                  
#endif

  //拓展帧滤波器个数，0~128
#if Enable_Extended_Filter	
	uint16_t msg_count_extended = sizeof(Msg_Filter_Extended_ID) / sizeof(uint32_t);
	hfdcan.Init.ExtFiltersNbr = msg_count_extended % 2 ? msg_count_extended / 2 + 1 : msg_count_extended / 2;
	if( msg_count_extended % 2 != 0 )
		msg_count_extended--; 
#else
  hfdcan.Init.ExtFiltersNbr = 0;                   
#endif

  hfdcan.Init.MessageRAMOffset = 0;           		 //信息RAM偏移
  hfdcan.Init.RxFifo0ElmtsNbr = RxFifo0ElmtsCnt;   //接收FIFO0元素个数，0-64
  hfdcan.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;//接收FIFO0元素的数据域大小:8字节	
  hfdcan.Init.RxFifo1ElmtsNbr = RxFifo1ElmtsCnt;   //接收FIFO1元素个数，0-64
	hfdcan.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_64;//接收FIFO1元素的数据域大小:8字节 
	hfdcan.Init.RxBuffersNbr = 0;                    //接收buffer元素个数，0~64
  
	hfdcan.Init.TxEventsNbr = 0;                     //发送事件FIFO元素个数，0~32
  hfdcan.Init.TxBuffersNbr = 0;                    //发送buffer元素个数，0~32
  hfdcan.Init.TxFifoQueueElmtsNbr = TxFifoElmtsCnt;//发送Buffer被用作发送FIFO/队列的元素个数，0~32
  hfdcan.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;//发送FIFO模式选择，可以选择FIFO模式或队列模式
  hfdcan.Init.TxElmtSize = FDCAN_DATA_BYTES_64;     //发送元素的数据域大小:8字节
  HAL_FDCAN_Init(&hfdcan);

  //标准帧滤波器配置
#if Enable_Standard_Filter
	for( uint8_t i=0; i < msg_count_standard; i+=2 )
	{
		sFilterConfig.IdType = FDCAN_STANDARD_ID;        //标准ID
		sFilterConfig.FilterIndex = (int)(i/2);          //滤波器索引
		sFilterConfig.FilterType = FDCAN_FILTER_DUAL;  	 //滤波器类型
		sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//过滤器0关联到FIFO0
		sFilterConfig.FilterID1 = Msg_Filter_Standard_ID[i];           //11位ID1
		sFilterConfig.FilterID2 = Msg_Filter_Standard_ID[i+1];         //11位ID2
		HAL_FDCAN_ConfigFilter(&hfdcan, &sFilterConfig);
	}
	if( msg_count_standard < sizeof(Msg_Filter_Standard_ID)/2){
		sFilterConfig.IdType = FDCAN_STANDARD_ID;        //标准ID
		sFilterConfig.FilterIndex = msg_count_standard/2;         //滤波器索引
		sFilterConfig.FilterType = FDCAN_FILTER_DUAL;    //滤波器类型
		sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//过滤器0关联到FIFO0
		sFilterConfig.FilterID1 = Msg_Filter_Standard_ID[msg_count_standard];  //11位ID1
		sFilterConfig.FilterID2 = Msg_Filter_Standard_ID[msg_count_standard];  //11位ID2
		HAL_FDCAN_ConfigFilter(&hfdcan, &sFilterConfig);		
	}
#endif	

  //拓展帧滤波器配置
#if Enable_Extended_Filter
	for( uint8_t i=0; i < msg_count_extended; i+=2 )
	{
		sFilterConfig.IdType = FDCAN_EXTENDED_ID;        //拓展ID
		sFilterConfig.FilterIndex = (int)(i/2);          //滤波器索引
		sFilterConfig.FilterType = FDCAN_FILTER_DUAL;  	 //滤波器类型
		sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//过滤器0关联到FIFO0
		sFilterConfig.FilterID1 = Msg_Filter_Extended_ID[i];           //11位ID1
		sFilterConfig.FilterID2 = Msg_Filter_Extended_ID[i+1];         //11位ID2
		HAL_FDCAN_ConfigFilter(&hfdcan, &sFilterConfig);
	}
	if( msg_count_extended < sizeof(Msg_Filter_Extended_ID)/2){
		sFilterConfig.IdType = FDCAN_EXTENDED_ID;         //拓展ID
		sFilterConfig.FilterIndex = msg_count_extended/2; //滤波器索引
		sFilterConfig.FilterType = FDCAN_FILTER_DUAL;     //滤波器类型
		sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; //过滤器0关联到FIFO0
		sFilterConfig.FilterID1 = Msg_Filter_Extended_ID[msg_count_extended];  //11位ID1
		sFilterConfig.FilterID2 = Msg_Filter_Extended_ID[msg_count_extended];  //11位ID2
		HAL_FDCAN_ConfigFilter(&hfdcan, &sFilterConfig);		
	}
#endif	


#if Enable_Extended_Filter || Enable_Standard_Filter
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
#else 
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
#endif	

  /* Configure Rx FIFO 0 watermark to 1 */
  HAL_FDCAN_ConfigFifoWatermark(&hfdcan, FDCAN_CFG_RX_FIFO0, 1);

  /* Activate Rx FIFO 0 watermark notification */
  HAL_FDCAN_ActivateNotification(&hfdcan, FDCAN_IT_RX_FIFO0_WATERMARK, RxFifo0ElmtsCnt);
	
	/* Activate Rx FIFO 0 NEW_MESSAGE notification */
  HAL_FDCAN_ActivateNotification(&hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
		
  /* Activate FDCAN_IT_TX_COMPLETE notification */
  HAL_FDCAN_ActivateNotification(&hfdcan, FDCAN_IT_TX_COMPLETE, 0xffffffff);	
	
  /* Start the FDCAN module */
  HAL_FDCAN_Start(&hfdcan);
}

extern "C" void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* hfdcan)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;
  
  /*##-1- Enable peripherals and GPIO Clocks */
  /* Enable GPIO TX/RX clock */
  FDCANx_TX_GPIO_CLK_ENABLE();
  FDCANx_RX_GPIO_CLK_ENABLE();
  
  /* Enable FDCANx clock */
  FDCANx_CLK_ENABLE();
  
  /*##-2- Configure peripheral GPIO */
  /* FDCANx TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = FDCANx_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = FDCANx_TX_AF;
  HAL_GPIO_Init(FDCANx_TX_GPIO_PORT, &GPIO_InitStruct);
  
  /* FDCANx RX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = FDCANx_RX_PIN;
  GPIO_InitStruct.Alternate = FDCANx_RX_AF;
  HAL_GPIO_Init(FDCANx_RX_GPIO_PORT, &GPIO_InitStruct);
  
	/* FDCANx Enable GPIO pin configuration   */
	
  #if CANVER==1
		GPIO_InitStruct.Pin       = GPIO_PIN_5;
	#elif CANVER==0
		GPIO_InitStruct.Pin       = GPIO_PIN_3;
	#endif
  GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);	
	
	delay(1e-4);
	FDCANx_Enable;

  /*##-3- Configure the NVIC */   
  NVIC_SetPriority(FDCANx_IT0_IRQn,5);
  NVIC_SetPriority(FDCANx_IT1_IRQn, 5);
  NVIC_EnableIRQ(FDCANx_IT0_IRQn);
  NVIC_EnableIRQ(FDCANx_IT1_IRQn);
}

extern "C" void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* hfdcan)
{
  /*##-1- Reset peripherals */
  FDCANx_FORCE_RESET();
  FDCANx_RELEASE_RESET();
  
  /*##-2- Disable peripherals and GPIO Clocks */
  /* Configure FDCANx Tx as alternate function  */
  HAL_GPIO_DeInit(FDCANx_TX_GPIO_PORT, FDCANx_TX_PIN);
  
  /* Configure FDCANx Rx as alternate function  */
  HAL_GPIO_DeInit(FDCANx_RX_GPIO_PORT, FDCANx_RX_PIN);
  
  /*##-3- Disable the NVIC for FDCANx */
  HAL_NVIC_DisableIRQ(FDCANx_IT0_IRQn);
  HAL_NVIC_DisableIRQ(FDCANx_IT1_IRQn);
  HAL_NVIC_DisableIRQ(FDCAN_CAL_IRQn);
}
