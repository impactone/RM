#include "BSP_TIM.h"

void BSP_TIM_InitConfig(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef        TIM_OCInitStructure;
  TIM_ICInitTypeDef        TIM_ICInitStructure;
  GPIO_InitTypeDef 				 GPIOInitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);   //PCLK1=42MHz,TIM5 clk =84MHz
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA ,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  //TIM3 ENCODER
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_TIM3);
	GPIOInitStruct.GPIO_Pin      = GPIO_Pin_4 | GPIO_Pin_5;
  GPIOInitStruct.GPIO_Speed    = GPIO_Speed_100MHz;
	GPIOInitStruct.GPIO_Mode     = GPIO_Mode_AF;
	GPIOInitStruct.GPIO_PuPd     = GPIO_PuPd_UP;
	GPIO_Init(GPIOB,&GPIOInitStruct);	
	
	TIM_DeInit(TIM3);	
  TIM_TimeBaseStructure.TIM_Period           = 0xffff;      
  TIM_TimeBaseStructure.TIM_Prescaler        = 0;	    
  TIM_TimeBaseStructure.TIM_ClockDivision    = TIM_CKD_DIV1 ;	
  TIM_TimeBaseStructure.TIM_CounterMode      = TIM_CounterMode_Up; 
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
	TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 6;         //比较滤波器
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
  TIM_Cmd(TIM3,ENABLE);
	TIM_SetCounter(TIM3,0X7FFF);  //0x7fff = 0xffff / 2;
	
	//TIM5 摩擦轮
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5); 
	GPIOInitStruct.GPIO_Pin      = GPIO_Pin_0 | GPIO_Pin_1;
	GPIOInitStruct.GPIO_Mode     = GPIO_Mode_AF;
	GPIOInitStruct.GPIO_Speed    = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&GPIOInitStruct);	

	TIM_TimeBaseStructure.TIM_Prescaler = 84-1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 2500;   //2.5ms
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 1000;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;
	TIM_OC1Init(TIM5,&TIM_OCInitStructure);
	TIM_OC2Init(TIM5,&TIM_OCInitStructure);
	 
  TIM_OC1PreloadConfig(TIM5,TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM5,TIM_OCPreload_Enable);	 
		
  TIM_ARRPreloadConfig(TIM5,ENABLE);
	TIM_Cmd(TIM5,ENABLE);
	TIM_SetCompare1(TIM5,1000);
	TIM_SetCompare2(TIM5,1000);
	
	//TIM9 PWM_MOTOR
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM9);	
	GPIOInitStruct.GPIO_Pin               = GPIO_Pin_2;
	GPIOInitStruct.GPIO_Mode              = GPIO_Mode_AF;
	GPIOInitStruct.GPIO_Speed             = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&GPIOInitStruct);

  TIM_TimeBaseStructure.TIM_Prescaler              = 168-1;
  TIM_TimeBaseStructure.TIM_CounterMode            = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period                 = 1000;   //1ms,1KHz
  TIM_TimeBaseStructure.TIM_ClockDivision          = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM9,&TIM_TimeBaseStructure);
  
	TIM_OCInitStructure.TIM_OCMode                   = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState              = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState             = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_Pulse                    = 0;
	TIM_OCInitStructure.TIM_OCPolarity               = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCNPolarity              = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState              = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState             = TIM_OCIdleState_Set;
	TIM_OC1Init(TIM9,&TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM9,TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM9,ENABLE);
	TIM_Cmd(TIM9,ENABLE);
	
	
	
}

void BSP_GPIO_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOF时钟

  //GPIOF9,F10初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;//LED0和LED1对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO
	
	GPIO_SetBits(GPIOC,GPIO_Pin_1 | GPIO_Pin_2);//GPIOF9,F10设置高，灯灭
}
