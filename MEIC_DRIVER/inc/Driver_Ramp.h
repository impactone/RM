#ifndef __DRIVER_RAMP_H
#define __DRIVER_RAMP_H

#include "stdint.h"
#include "Config.h"

typedef struct 
{
	uint32_t CNT;
	uint32_t TICK_CNT;
	float    Output;
}Ramp_Struct;

void Ramp_InitAllRamp(void);
float Ramp_Calc(Ramp_Struct *ramp);
void Ramp_ReInit(Ramp_Struct *ramp);

extern Ramp_Struct CloudPitch_Ramp;
extern Ramp_Struct CloudYaw_Ramp;
extern Ramp_Struct Chasis1_Ramp;
extern Ramp_Struct Chasis2_Ramp;
extern Ramp_Struct Chasis3_Ramp;
extern Ramp_Struct Chasis4_Ramp;

#endif
