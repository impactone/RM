#include "Driver_FricMotor.h"

void Fric_Init(void)
{
  SetFricMotor(0);	
}

void SetFricMotor(uint8_t status)
{
	if (!status)
	{
		FRIC_MOTOR1 = 1000;
		FRIC_MOTOR2 = 1000;
	}else
	{
		FRIC_MOTOR1 = 1250;
		FRIC_MOTOR2 = 1250;
	}
}

