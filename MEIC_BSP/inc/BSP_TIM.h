#ifndef __BSP_TIM_H
#define __BSP_TIM_H

#include "stm32f4xx.h"

void BSP_TIM_InitConfig(void);
void BSP_GPIO_Init(void);

#define DotShoootMotor1 PCout(1)
#define DotShoootMotor2 PCout(2)
#endif

