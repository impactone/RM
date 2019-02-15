#ifndef __COMPUTERVISION_H
#define __COMPUTERVISION_H

#include "Config.h"
#include "stdint.h"
#include "Driver_PID.h"
#include "Driver_CAN.h"
#include "Driver_Ramp.h"
#include "Driver_FricMotor.h"

#include "Task_IMU.h"
#include "Task_Control.h"



#define VISION_DATA_LENGTH 4
#define PC_BUFF_LENGTH   20

#define FITTING_SAMPLE_NUM   5    //拟合样本数

#define GUARD_AGAINST_SPEED  35   //警戒速度

#define MAX_PCPACK_DATA 2000
#define MIN_PCPACK_DATA -2000

extern POS_PID_Struct PID_Pitch_P;
extern POS_PID_Struct PID_Pitch_V;
extern POS_PID_Struct PID_Yaw_P;
extern POS_PID_Struct PID_Yaw_V;

extern Attitude CloudAttitude;

extern uint16_t Scan_Cnt;


typedef union
{
	uint8_t data[4];
	float Trans;
}DataTransType;	

typedef struct 
{
	float X;
	float Y;
	float Z;
	float T;
}EnemyType;

typedef struct
{
	float Yaw;
	float Pitch;
}AngleType;

extern uint8_t MiniPCFrameRate;

void CVProcess(void);
void MiniPCDecode(void);
void CV_ClearEnemy(void);
void MiniPCRec(uint8_t res);


#endif


