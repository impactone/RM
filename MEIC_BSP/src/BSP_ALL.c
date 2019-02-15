#include "BSP_ALL.h"

void BSP_Init(void)
{
	 BSP_GPIO_Init();
 	 BSP_NVIC_InitConfig();
	 BSP_UART_InitConfig();	
	 BSP_IIC_InitConfig(); 
	 BSP_CAN_InitConfig();  
	 BSP_TIM_InitConfig();
	 BSP_EXTI_InitConfig();
}
