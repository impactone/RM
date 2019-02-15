#ifndef __DRIVER_PID_H
#define __DRIVER_PID_H

#include "Config.h"

typedef struct
{
	float P;
	float I;
	float D;
	float Imax;
	
  float NowErr;
  float LstErr;
	float SumErr;
	
  float Expect;
  float Measured;
	
	float OutPut;
	float MaxOutput;
}POS_PID_Struct;


extern POS_PID_Struct PID_ChasisMotor1;
extern POS_PID_Struct PID_ChasisMotor2;
extern POS_PID_Struct PID_ChasisMotor3;
extern POS_PID_Struct PID_ChasisMotor4;
extern POS_PID_Struct PID_CV_ENEMY_X;
extern POS_PID_Struct PID_CV_ENEMY_Y;
extern POS_PID_Struct PID_DotShoot_VO;
extern POS_PID_Struct PID_DotShoot_P;
extern POS_PID_Struct PID_DotShoot_V;
extern POS_PID_Struct PID_Pitch_P;
extern POS_PID_Struct PID_Pitch_V;
extern POS_PID_Struct PID_Yaw_P;
extern POS_PID_Struct PID_Yaw_V;
extern POS_PID_Struct PID_Servo;

void PID_Init(void);
float PID_Calc(POS_PID_Struct *pid);
void PID_Reset_SumErr(POS_PID_Struct *pid);

#endif
