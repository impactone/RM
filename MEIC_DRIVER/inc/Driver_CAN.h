#ifndef __DRIVER_CAN_H
#define __DRIVER_CAN_H


#include "stm32f4xx.h"
#include "includes.h"
#include "Config.h"

#include "Driver_PID.h"
#include "Driver_MPU6050.h"

#include "Task_SysInit.h"


#define RM3510_MOTOR1ID 0X201
#define RM3510_MOTOR2ID 0X202
#define RM3510_MOTOR3ID 0X203
#define RM3510_MOTOR4ID 0X204
#define RM6623_YAWID    0X205
#define RM6623_PITCHID  0X206
#define ZGYRO_ID        0X401




typedef struct
{
	int16_t last_enc;
	int16_t now_enc;
	int16_t dif_enc;
	int16_t smooth_enc;
	
	int8_t rount_cnt;
	int16_t bias;
}EncoderType;

void GMEncoder_Init(void);
void ZGYRO_Reset(void);
void Send2Cloud(int16_t pitch,int16_t yaw);
void Send2Chasis(int16_t v1,int16_t v2,int16_t v3,int16_t v4);
void CAN1_Process(CanRxMsg RxMsg);
void CAN2_Process(CanRxMsg RxMsg);

#endif
