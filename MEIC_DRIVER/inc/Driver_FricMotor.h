#ifndef __DRIVER_FRICMOTOR_H
#define __DRIVER_FRICMOTOR_H

#include "BSP_TIM.h"

#define FRIC_MOTOR1 TIM5->CCR1 
#define FRIC_MOTOR2 TIM5->CCR2

void Fric_Init(void);
void SetFricMotor(uint8_t status);


#endif
