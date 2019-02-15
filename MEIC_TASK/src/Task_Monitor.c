#include "Task_Monitor.h"

char ErrMsg[100]                    = {0};
uint16_t ErrCode = 0;
uint8_t YawIMUFrameRate             = 0;
uint8_t MiniPCFrameRate             = 0;
uint8_t RemoterFrameRate            = 0;
uint8_t YawMotorFrameRate           = 0;
uint8_t PitchMotorFrameRate         = 0;
uint8_t ChasisMotor1FrameRate       = 0;
uint8_t ChasisMotor2FrameRate       = 0;
uint8_t ChasisMotor3FrameRate       = 0;
uint8_t ChasisMotor4FrameRate       = 0;
uint8_t ChasisControlBoardFrameRate = 0;


void FrameRateReInit(void)
{
	   ErrMsg[0] = '\0';
     YawIMUFrameRate             = 0;
     MiniPCFrameRate             = 0;	
	   RemoterFrameRate            = 0;
	   YawMotorFrameRate           = 0;
	   PitchMotorFrameRate         = 0;
     ChasisMotor1FrameRate       = 0;
     ChasisMotor2FrameRate       = 0;
     ChasisMotor3FrameRate       = 0;
     ChasisMotor4FrameRate       = 0;    
     ChasisControlBoardFrameRate = 0;
}

void Task_Monitor(void *p_arg)
{
	(void)p_arg;
	OS_ERR err;
	
	for (;;)
	{
			if (YawIMUFrameRate < YAW_IMU_FRAME_RATE)
			{
				strcat(ErrMsg,"YawIMU Error!  ");
				ErrCode |= 1<<0;
			}
			if (MiniPCFrameRate < MINI_PC_FRAME_RATE)
			{
				strcat(ErrMsg,"MiniPC Error!  ");
				ErrCode |= 1<<1;		
			}
			if (RemoterFrameRate < REMOTER_FRAME_RATE)
			{
				strcat(ErrMsg,"Remoter Error!  ");
				ErrCode |= 1<<2;		
			}
			if (YawMotorFrameRate < YAW_MOTOR_FRAME_RATE)
			{
				strcat(ErrMsg,"YawMotor Error!  ");
				ErrCode |= 1<<3;
			}
			if (PitchMotorFrameRate < PITCH_MOTOR_FRAME_RATE)
			{
				strcat(ErrMsg,"PitchMotor Error!  ");
				ErrCode |= 1<<4;
			}
			if (ChasisMotor1FrameRate < CHASIS_MOTOR1_FRAME_RATE)
			{
				strcat(ErrMsg,"ChasisMotor1 Error!  ");
				ErrCode |= 1<<5;
			}
			if (ChasisMotor2FrameRate < CHASIS_MOTOR2_FRAME_RATE)
			{
				strcat(ErrMsg,"ChasisMotor2 Error!  ");
				ErrCode |= 1<<6;
			}
			if (ChasisMotor3FrameRate < CHASIS_MOTOR3_FRAME_RATE)
			{
				strcat(ErrMsg,"ChasisMotor3 Error!  ");
				ErrCode |= 1<<7;
			}
			if (ChasisMotor4FrameRate < CHASIS_MOTOR4_FRAME_RATE)
			{
				strcat(ErrMsg,"ChasisMotor4 Error!  ");
				ErrCode |= 1<<8;
			}	
			if (ChasisControlBoardFrameRate < CHASIS_CONTROL_BOARD_FRAME_RATE)
			{
				strcat(ErrMsg,"ChasisControlBoard Error!  ");
				ErrCode |= 1<<9;
			}
			
			//printf("%s\r\n",ErrMsg);
//			printf("Chasis1:%d Chasis2:%d Chasis3:%d Chasis4:%d PitchMotor:%d YawMotor:%d ChasisBoard:%d YawIMU:%d MiniPC:%d Remoter:%d\r\n",
//			ChasisMotor1FrameRate,ChasisMotor2FrameRate,ChasisMotor3FrameRate,ChasisMotor4FrameRate,
//			PitchMotorFrameRate,YawMotorFrameRate,ChasisControlBoardFrameRate,YawIMUFrameRate,MiniPCFrameRate,
//			RemoterFrameRate);
			//printf("MiniPCFrameRate : %d\r\n",MiniPCFrameRate);
			FrameRateReInit();			
			OSTimeDly(300,OS_OPT_TIME_DLY,&err);
	}
}
