#include "Driver_Ramp.h"

Ramp_Struct CloudPitch_Ramp;
Ramp_Struct CloudYaw_Ramp;
Ramp_Struct Chasis1_Ramp;
Ramp_Struct Chasis2_Ramp;
Ramp_Struct Chasis3_Ramp;
Ramp_Struct Chasis4_Ramp;

/* ��ʼ������б�� */
void Ramp_Init(Ramp_Struct *ramp,uint32_t tick_cnt)
{
	ramp->CNT       = 0;
	ramp->TICK_CNT  = tick_cnt;
	ramp->Output    = 0.0f;
}

/* ��ʼ������б�� */
void Ramp_InitAllRamp(void)
{
	Ramp_Init(&CloudPitch_Ramp,CLOUD_RAMP_TICK_TO_INC);
	Ramp_Init(&CloudYaw_Ramp,CLOUD_RAMP_TICK_TO_INC);
	
	Ramp_Init(&Chasis1_Ramp,CHASIS_RAMP_TICK_TO_INC);
	Ramp_Init(&Chasis2_Ramp,CHASIS_RAMP_TICK_TO_INC);
	Ramp_Init(&Chasis3_Ramp,CHASIS_RAMP_TICK_TO_INC);
	Ramp_Init(&Chasis4_Ramp,CHASIS_RAMP_TICK_TO_INC);
}

/* б������ */
void Ramp_ReInit(Ramp_Struct *ramp)
{
	ramp->CNT    = 0;
	ramp->Output = 0.0f;
}

/* б�¼��� */
#define SAFE_MAX_UINT32_T 0xFFFFFFF  /* ע����7��F������8����Ϊ�˰�ȫ */
float Ramp_Calc(Ramp_Struct *ramp)
{	
	ramp->CNT      += ramp->TICK_CNT;                                                   /*����*/	
	ramp->CNT      = ramp->CNT >= SAFE_MAX_UINT32_T ? SAFE_MAX_UINT32_T : ramp->CNT;    /*�޷�*/	
  ramp->Output   = 1.0f * ramp->CNT / SAFE_MAX_UINT32_T;                              /*����б��*/	
	return ramp->Output;                                                                /*���*/
}

