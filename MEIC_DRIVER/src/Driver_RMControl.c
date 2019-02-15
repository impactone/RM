#include "Driver_Ramp.h"
#include "Driver_RMControl.h"

/*
	LeftSwitchFlag��������λ
		 2      |    1     |     0
	FricMotor |   MID    |  DotShooter
*/
uint8_t LeftSwitchFlag  = 0x02;  //0  1  0

/*���ⲿһ������LeftSwitchFlag��ͨ��*/
void RM_SetLeftSwtichFlag(uint8_t code)
{
	LeftSwitchFlag = code;
}

/*��������Ŀ��ƣ�Ħ����+�����֣�*/
void RM_FricControl(uint8_t *fric_status)
{
	switch (GetRemoterMsg(LEFT_SWITCH))
	{
		case 1: //������
			if (LeftSwitchFlag & 0x04)
			{
				if (!(*fric_status)) 
					*fric_status = 1;
				else
					*fric_status = 0;
				LeftSwitchFlag = 0x03; //0  1  1
			}  
			break;
		case 2: //������
			if (LeftSwitchFlag & 0x01)
			{
				if ((*fric_status))  //��Ħ�����Ѿ������������
				{
					if ((*fric_status) == 2)
						*fric_status = 1;
					else
						*fric_status = 2;
				}
				LeftSwitchFlag = 0x06; //1 1 0
			}
			break;
		case 3: //���м�
			if (LeftSwitchFlag & 0x02)
			{
				LeftSwitchFlag = 0x05; //1 0 1
			}
			break;
		default:
			break;
	}
	
	if ((*fric_status))
	{
		SetFricMotor(1);
	}else
	{
		SetFricMotor(0);
	}   
}

/*�����޷�*/
void DataLimit(float *a,float Max,float Min)
{
	if (*a >= Max) 
		*a = Max;
	else if (*a <= Min)
		*a = Min;
}

/*Pitch����*/
void RM_PitchControl(int16_t *current)
{
	PID_Pitch_P.Expect   += (1024 - GetRemoterMsg(CH3)) * ANGLE_INC_FAC;
	DataLimit(&PID_Pitch_P.Expect,MAX_PITCH,MIN_PITCH);
	PID_Pitch_P.Measured  = CloudAttitude.ENC_Pitch * 0.0439453125f; //360 / 8192
	//PID_Pitch_P.Measured  = CloudAttitude.pitch - 33.0f;
	PID_Pitch_P.Measured  *=  Ramp_Calc(&CloudPitch_Ramp);
	PID_Pitch_V.Measured  = -CloudAttitude.gy;	
	PID_Pitch_V.Expect    = PID_Calc(&PID_Pitch_P);           //�⻷λ�û�����
	*current              = (int16_t)PID_Calc(&PID_Pitch_V);  //�ڻ��ٶȻ�����
}

/*Yaw����*/
void RM_YawControl(int16_t *current)
{
	PID_Yaw_P.Expect    += (1024 - GetRemoterMsg(CH2)) * ANGLE_INC_FAC;
	//DataLimit(&PID_Yaw_P.Expect,MAX_YAW,MIN_YAW);
	//PID_Yaw_P.Measured   = CloudAttitude.ENC_Yaw * 0.0439453125f; //360 / 8192
	PID_Yaw_P.Measured   = -CloudAttitude.yaw;
	PID_Yaw_V.Measured   = CloudAttitude.gz;
	PID_Yaw_V.Expect     = PID_Calc(&PID_Yaw_P);            //�⻷λ�û�����
	*current             = (int16_t)PID_Calc(&PID_Yaw_V);   //�ڻ��ٶȻ�����	
}

/*���̿���*/
void RM_ChasisControl(int16_t *current)
{
	int16_t vx = (1024 - GetRemoterMsg(CH1)) * RM_CHASIS_SPEED_FAC;
	int16_t vy = (1024 - GetRemoterMsg(CH0)) * RM_CHASIS_SPEED_FAC;
	int16_t w = 0;
	 
	//���̸���ջ�
	PID_Servo.Measured = CloudAttitude.ENC_Yaw * 0.0439453125f; //360 / 8192
	w = PID_Calc(&PID_Servo);
	
	//�����ķ�ֽ���
	PID_ChasisMotor1.Expect = vx + vy + w;
	PID_ChasisMotor2.Expect = vx - vy - w;
	PID_ChasisMotor3.Expect = vx - vy + w;
	PID_ChasisMotor4.Expect = vx + vy - w;
	
	current[0] = PID_Calc(&PID_ChasisMotor1);
	current[1] = PID_Calc(&PID_ChasisMotor2);
	current[2] = PID_Calc(&PID_ChasisMotor3);
	current[3] = PID_Calc(&PID_ChasisMotor4);
	
}



