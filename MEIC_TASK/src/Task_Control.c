#include "Task_Control.h"

/*
	RightSwitchFlag有如下三位
		 2    |    1     |   0
	ReMoter | KeyMouse | STOP
*/
uint8_t RightSwitchFlag = 0x07;  //1  1  1

uint8_t FricStatus = 0;

void Task_Control(void *p_arg)
{
	(void)p_arg;
	OS_ERR err;	
	DotShootTickInit();

	for (;;)
	{
		ControlModeSwitch();
		switch (GetControlMode())
		{
			case RMMode://遥控器模式				
				RMProcess();
				break;
			case KMMode://键鼠模式
				KMProcess();
//				CVProcess();						
				break;
			case STMode://停车模式
//				STProcess();
				break;
			default:
				
				break;
		}
	  DotShootControl(FricStatus,0);           //点射控制	0为双闭环 1为速度闭环	
		OSTimeDly(10,OS_OPT_TIME_DLY,&err);  
	}
}

/*遥控器控制*/
void RMProcess(void)
{
	int16_t PitchCurrent;
	int16_t YawCurrent;
	int16_t ChasisCurrent[4];
	
	if (RightSwitchFlag & 0x04)  //第一次进入遥控器模式且遥控器还活着
	{
		PID_Pitch_P.Expect = 0;
		PID_Yaw_P.Expect   = -CloudAttitude.yaw; 
		FricStatus         = 0;
		RightSwitchFlag    = 0x03;  //0 1 1
		RM_SetLeftSwtichFlag(0x02); //0 1 0		
		Ramp_ReInit(&CloudPitch_Ramp);
	}
	
	RM_FricControl(&FricStatus);              //摩擦轮控制以及状态更新
	RM_YawControl(&YawCurrent);               //Yaw轴云台闭环
	RM_PitchControl(&PitchCurrent);           //Pitch轴云台闭环
	RM_ChasisControl(ChasisCurrent);          //底盘闭环	
	
//	Send2Cloud(PitchCurrent,YawCurrent);     					//云台信息发送	
//	Send2Chasis(ChasisCurrent[0],ChasisCurrent[1],ChasisCurrent[2],ChasisCurrent[3]);  //底盘信息发送

}


/*键鼠控制*/
void KMProcess(void)
{
	if (RightSwitchFlag & 0x02)  //第一次进入键鼠模式
	{
		//各种初始化
		Scan_Cnt = 500;
		PID_Reset_SumErr(&PID_CV_ENEMY_X);
		PID_Reset_SumErr(&PID_CV_ENEMY_Y);
		Ramp_ReInit(&CloudPitch_Ramp);
		CV_ClearEnemy();
		RightSwitchFlag = 0x05; //1 0 1
	}
	
	
}

/*待机模式*/
void STProcess(void)
{
	if (RightSwitchFlag & 0x01)  //第一次进入待机模式
	{
		//各种初始化
		RightSwitchFlag = 0x06; //1 1 0
	}
	//各种停机
	FricStatus = 0;
	Send2Cloud(0,0);
	SetFricMotor(0);
	Send2Chasis(0,0,0,0);
}

