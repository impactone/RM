#ifndef TASK_MONITOR_H
#define TASK_MONITOR_H

#include "stdint.h"
#include "includes.h"

#define YAW_IMU_FRAME_RATE       10
#define MINI_PC_FRAME_RATE       10
#define REMOTER_FRAME_RATE       7
#define YAW_MOTOR_FRAME_RATE     98
#define PITCH_MOTOR_FRAME_RATE   98
#define CHASIS_MOTOR1_FRAME_RATE 10
#define CHASIS_MOTOR2_FRAME_RATE 10
#define CHASIS_MOTOR3_FRAME_RATE 10
#define CHASIS_MOTOR4_FRAME_RATE 10
#define CHASIS_CONTROL_BOARD_FRAME_RATE 10

extern uint8_t YawIMUFrameRate             ;
extern uint8_t MiniPCFrameRate             ;
extern uint8_t RemoterFrameRate            ;
extern uint8_t YawMotorFrameRate           ;
extern uint8_t PitchMotorFrameRate         ;
extern uint8_t ChasisMotor1FrameRate       ;
extern uint8_t ChasisMotor2FrameRate       ;
extern uint8_t ChasisMotor3FrameRate       ;
extern uint8_t ChasisMotor4FrameRate       ;
extern uint8_t ChasisControlBoardFrameRate ;

void Task_Monitor(void *p_arg);


#endif
