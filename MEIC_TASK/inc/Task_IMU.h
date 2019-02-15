#ifndef  __TASK_IMU_H
#define  __TASK_IMU_H


#include "includes.h"
#include "Driver_MPU6050.h"

extern Attitude CloudAttitude;
extern OS_TCB  Task_SysInitTCB;

void Task_IMU(void *p_arg);
void InitQ(Attitude *InitAng);
void IMU_GetPitchRollYaw(Attitude *angles);

#endif
