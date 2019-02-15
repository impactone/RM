#ifndef __TASK_CONTROL_H
#define __TASK_CONTROL_H

#include "includes.h"
#include "Driver_CAN.h"
#include "Driver_PID.h"
#include "Driver_Ramp.h"
#include "Driver_Remoter.h"
#include "Driver_DotShoot.h"
#include "Driver_FricMotor.h"
#include "Driver_RMControl.h"
#include "Driver_ComputerVision.h"

#include "Task_IMU.h"
#include "Config.h"

extern uint16_t ErrCode;

extern uint8_t FricStatus;

void Task_Control(void *p_arg);

void RMProcess(void);
void KMProcess(void);
void STProcess(void);


#endif
