#include "Driver_PID.h"


POS_PID_Struct PID_ChasisMotor1;
POS_PID_Struct PID_ChasisMotor2;
POS_PID_Struct PID_ChasisMotor3;
POS_PID_Struct PID_ChasisMotor4;
POS_PID_Struct PID_CV_ENEMY_X;
POS_PID_Struct PID_CV_ENEMY_Y;
POS_PID_Struct PID_DotShoot_VO;
POS_PID_Struct PID_DotShoot_P;
POS_PID_Struct PID_DotShoot_V;
POS_PID_Struct PID_Pitch_P;
POS_PID_Struct PID_Pitch_V;
POS_PID_Struct PID_Yaw_P;
POS_PID_Struct PID_Yaw_V;
POS_PID_Struct PID_Servo;


/*PID结构体初始化*/
void PIDStruct_Init(POS_PID_Struct *pid)
{
	pid->P          = 0;
	pid->I          = 0;
	pid->D          = 0;
	pid->Imax       = 0;
	pid->NowErr     = 0;
	pid->LstErr     = 0;
	pid->SumErr     = 0;
	pid->Expect     = 0;
	pid->Measured   = 0;
	pid->OutPut     = 0;
	pid->MaxOutput  = 0;
}

/*PID参数初始化*/
void PID_Param_Init(POS_PID_Struct *pid,\
	                  float p,float i,float d,\
										float im,float output)
{
	pid->P          = p;
	pid->I          = i;
	pid->D          = d;
	pid->Imax       = im;
	pid->MaxOutput  = output;
}

/*清空积分项*/
void PID_Reset_SumErr(POS_PID_Struct *pid)
{
	pid->SumErr = 0;
}

/*PID初始化*/
void PID_Init(void)
{
	PIDStruct_Init(&PID_ChasisMotor1);
	PIDStruct_Init(&PID_ChasisMotor2);
	PIDStruct_Init(&PID_ChasisMotor3);
	PIDStruct_Init(&PID_ChasisMotor4);
	PIDStruct_Init(&PID_DotShoot_VO);
	PIDStruct_Init(&PID_DotShoot_P);
	PIDStruct_Init(&PID_DotShoot_V);
	PIDStruct_Init(&PID_Pitch_P);
	PIDStruct_Init(&PID_Pitch_V);
	PIDStruct_Init(&PID_Yaw_P);
	PIDStruct_Init(&PID_Yaw_V);
	PIDStruct_Init(&PID_Servo);
	
	PID_Param_Init(&PID_ChasisMotor1,5,0,5,500,MAX_CHASISI_CURRENT);
	PID_Param_Init(&PID_ChasisMotor2,5,0,5,500,MAX_CHASISI_CURRENT);
	PID_Param_Init(&PID_ChasisMotor3,5,0,5,500,MAX_CHASISI_CURRENT);
	PID_Param_Init(&PID_ChasisMotor4,5,0,5,500,MAX_CHASISI_CURRENT);
	
	
	PID_Param_Init(&PID_CV_ENEMY_X,0.013,0.0003,0.0001,2000,15); // 0.008  0.0004
	PID_Param_Init(&PID_CV_ENEMY_Y,0.01,0.0002,0,40000,15);
	
	
	PID_Param_Init(&PID_DotShoot_VO,1,0.08,0,5000,1000);
	PID_Param_Init(&PID_DotShoot_P,0,0,0,500,1000);//6 0 2
	PID_Param_Init(&PID_DotShoot_V,1.3,0,0,500,1000);//1.3  0 5
	
	PID_Param_Init(&PID_Pitch_P,20,0,0,200,5000);  //20  13
	PID_Param_Init(&PID_Pitch_V,19,0,0,200,5000); //19
	PID_Param_Init(&PID_Yaw_P,25,0,0,1000,5000);  //25
	PID_Param_Init(&PID_Yaw_V,35,0,0,1000,5000);  //35
	
	PID_Param_Init(&PID_Servo,90,0,0,500,MAX_CHASISI_CURRENT);
}

/*PID计算函数*/
float PID_Calc(POS_PID_Struct *pid)
{
	volatile float Pout = 0,Iout = 0,Dout = 0;
	/*误差计算*/
	pid->NowErr  = pid->Measured - pid->Expect;
	pid->SumErr += (pid->I == 0) ? 0 : pid->NowErr;
	
	/*P-I-D*/
	Pout = pid->P * pid->NowErr;
	Iout = pid->I * pid->SumErr;
	Dout = pid->D * (pid->NowErr - pid->LstErr);
	
	/*积分限幅*/
	if (pid->SumErr >= pid->Imax)
	{
		pid->SumErr  = pid->Imax; 
	}
	else if (pid->SumErr <= -pid->Imax)
	{
		pid->SumErr  = -pid->Imax;
	}
	
	/*PID输出*/
	pid->OutPut = Pout + Iout + Dout;

	/*输出限幅*/
	if (pid->OutPut >= pid->MaxOutput) 
		pid->OutPut = pid->MaxOutput;
	else if (pid->OutPut <= -pid->MaxOutput)
	  pid->OutPut = -pid->MaxOutput;
	
	/*误差更新*/
	pid->LstErr = pid->NowErr;
		
	return pid->OutPut;
}

