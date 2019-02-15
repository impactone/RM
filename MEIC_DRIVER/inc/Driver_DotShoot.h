#ifndef __DRIVER_DOTSHOOT_H
#define __DRIVER_DOTSHOOT_H

#include "stm32f4xx.h"
#include "stdio.h"
#include "Driver_PID.h"

typedef enum
{
	Moving,
	Stop,
	Stuck
}DotShootStatus;

typedef struct
{
	int16_t         Speed;
	long            Position;
	DotShootStatus  Status;
}DotShootType;

extern POS_PID_Struct PID_DotShoot_VO;
extern POS_PID_Struct PID_DotShoot_P;
extern POS_PID_Struct PID_DotShoot_V;

void DotShoot_Init(void);
void DotShootTickInit(void);
void DotShootControl(uint8_t fric_status,uint8_t mode);
void GetEncoderFeedBack(DotShootType *dot_shoot);

#endif

