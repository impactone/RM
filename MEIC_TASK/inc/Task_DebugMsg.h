#ifndef __TASK_DEBUGMSG_H
#define __TASK_DEBUGMSG_H

#include "includes.h"
#include "Driver_DBUS.h"
#include "Driver_CAN.h"
#include "Driver_Ramp.h"
#include "Driver_DotShoot.h"
#include "Driver_MPU6050.h"
#include "Driver_Remoter.h"
#include "Driver_ComputerVision.h"

void Task_DebugMsg(void *p_arg);

#endif
