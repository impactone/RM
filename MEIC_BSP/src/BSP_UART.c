#include "BSP_UART.h"


/**
  * @brief  UART初始化
  * @param  void
  * @retval void
  */
void BSP_UART_InitConfig(void)
{
    USART_InitTypeDef   USART_InitStructure;
    GPIO_InitTypeDef	  GPIO_InitStructure;
	  NVIC_InitTypeDef	  NVIC_InitStructure;
	  DMA_InitTypeDef     DMA_InitStructure;
	
	  
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	  
	 
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);
	
	  //USART1
	  GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		USART_InitStructure.USART_BaudRate              =  100000;
	  USART_InitStructure.USART_WordLength            =  USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits              =  USART_StopBits_1;
	  USART_InitStructure.USART_Parity                =  USART_Parity_Even;
  	USART_InitStructure.USART_HardwareFlowControl   =  USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode                  =  USART_Mode_Rx;	
	  USART_Init(USART1, &USART_InitStructure); 
		USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
	  
	  //USART3
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
		GPIO_Init(GPIOB,&GPIO_InitStructure);	 
    
    USART_InitStructure.USART_BaudRate              =   115200;
    USART_InitStructure.USART_HardwareFlowControl   =   USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                  =   USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_Parity                =   USART_Parity_No;
    USART_InitStructure.USART_StopBits              =   USART_StopBits_1;
    USART_InitStructure.USART_WordLength            =   USART_WordLength_8b;
    USART_Init(USART3, &USART_InitStructure);
    
		//开DMA(USART1_Rx)
	  DMA_DeInit(DMA2_Stream2);
		while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE){}
    DMA_InitStructure.DMA_Channel                   = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr        = (uint32_t)(&USART1->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr           = (uint32_t)DBUS_BUFF;
    DMA_InitStructure.DMA_DIR                       = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize                = DBUS_LENGTH;
    DMA_InitStructure.DMA_PeripheralInc             = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc                 = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize        = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize            = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode                      = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority                  = DMA_Priority_Medium;
    DMA_InitStructure.DMA_FIFOMode                  = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold             = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst               = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst           = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream2, &DMA_InitStructure);	
		DMA_Cmd(DMA2_Stream2, ENABLE);
			
		
		//USART1空闲中断
		NVIC_InitStructure.NVIC_IRQChannel						         =	USART1_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelCmd					         =	ENABLE;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	 =	1;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority			     =	1;
	  NVIC_Init(&NVIC_InitStructure);
	  USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
		
		//USART3接收中断
		NVIC_InitStructure.NVIC_IRQChannel						         =	USART3_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelCmd					         =	ENABLE;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	 =	0;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority			     =	0;
	  NVIC_Init(&NVIC_InitStructure);
	  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
		
	  //使能串口
		USART_Cmd(USART1, ENABLE);
    USART_Cmd(USART3, ENABLE);
		USART_ClearFlag(USART1, USART_FLAG_IDLE);
    USART_ClearFlag(USART3, USART_IT_RXNE);
	
}



