#ifndef __DRIVER_RMCONTROL_H
#define __DRIVER_RMCONTROL_H

#include "stdint.h"
#include "Driver_PID.h"
#include "Driver_CAN.h"
#include "Driver_DBUS.h"
#include "Driver_Remoter.h"
#include "Driver_DotShoot.h"
#include "Driver_FricMotor.h"

#include "Task_IMU.h"

extern POS_PID_Struct PID_ChasisMotor1;
extern POS_PID_Struct PID_ChasisMotor2;
extern POS_PID_Struct PID_ChasisMotor3;
extern POS_PID_Struct PID_ChasisMotor4;
extern POS_PID_Struct PID_Pitch_P;
extern POS_PID_Struct PID_Pitch_V;
extern POS_PID_Struct PID_Yaw_P;
extern POS_PID_Struct PID_Yaw_V;
extern POS_PID_Struct PID_Servo;

extern Attitude CloudAttitude;

void RM_YawControl(int16_t *current);
void RM_PitchControl(int16_t *current);
void RM_ChasisControl(int16_t *current);
void RM_SetLeftSwtichFlag(uint8_t code);
void RM_FricControl(uint8_t *fric_status);


#endif
