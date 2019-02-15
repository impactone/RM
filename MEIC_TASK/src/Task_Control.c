#include "Task_Control.h"

/*
	RightSwitchFlag��������λ
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
			case RMMode://ң����ģʽ				
				RMProcess();
				break;
			case KMMode://����ģʽ
				KMProcess();
//				CVProcess();						
				break;
			case STMode://ͣ��ģʽ
//				STProcess();
				break;
			default:
				
				break;
		}
	  DotShootControl(FricStatus,0);           //�������	0Ϊ˫�ջ� 1Ϊ�ٶȱջ�	
		OSTimeDly(10,OS_OPT_TIME_DLY,&err);  
	}
}

/*ң��������*/
void RMProcess(void)
{
	int16_t PitchCurrent;
	int16_t YawCurrent;
	int16_t ChasisCurrent[4];
	
	if (RightSwitchFlag & 0x04)  //��һ�ν���ң����ģʽ��ң����������
	{
		PID_Pitch_P.Expect = 0;
		PID_Yaw_P.Expect   = -CloudAttitude.yaw; 
		FricStatus         = 0;
		RightSwitchFlag    = 0x03;  //0 1 1
		RM_SetLeftSwtichFlag(0x02); //0 1 0		
		Ramp_ReInit(&CloudPitch_Ramp);
	}
	
	RM_FricControl(&FricStatus);              //Ħ���ֿ����Լ�״̬����
	RM_YawControl(&YawCurrent);               //Yaw����̨�ջ�
	RM_PitchControl(&PitchCurrent);           //Pitch����̨�ջ�
	RM_ChasisControl(ChasisCurrent);          //���̱ջ�	
	
//	Send2Cloud(PitchCurrent,YawCurrent);     					//��̨��Ϣ����	
//	Send2Chasis(ChasisCurrent[0],ChasisCurrent[1],ChasisCurrent[2],ChasisCurrent[3]);  //������Ϣ����

}


/*�������*/
void KMProcess(void)
{
	if (RightSwitchFlag & 0x02)  //��һ�ν������ģʽ
	{
		//���ֳ�ʼ��
		Scan_Cnt = 500;
		PID_Reset_SumErr(&PID_CV_ENEMY_X);
		PID_Reset_SumErr(&PID_CV_ENEMY_Y);
		Ramp_ReInit(&CloudPitch_Ramp);
		CV_ClearEnemy();
		RightSwitchFlag = 0x05; //1 0 1
	}
	
	
}

/*����ģʽ*/
void STProcess(void)
{
	if (RightSwitchFlag & 0x01)  //��һ�ν������ģʽ
	{
		//���ֳ�ʼ��
		RightSwitchFlag = 0x06; //1 1 0
	}
	//����ͣ��
	FricStatus = 0;
	Send2Cloud(0,0);
	SetFricMotor(0);
	Send2Chasis(0,0,0,0);
}

