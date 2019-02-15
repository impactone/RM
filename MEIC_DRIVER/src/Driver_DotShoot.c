#include "Driver_DotShoot.h"
#include "BSP_TIM.h"
#include "Driver_CAN.h"
#include "math.h"

DotShootType DotShoot;

/*编码器初始化*/
void DotShoot_Init(void)
{
   DotShoot.Position = 0;
	 DotShoot.Speed = 0;
	 DotShoot.Status = Stop;
}

/*获取编码器*/
void GetEncoderFeedBack(DotShootType *dot_shoot)
{
	int16_t err;
	err = TIM3->CNT - 0X7FFF;
	dot_shoot->Speed        = err;
	dot_shoot->Position    += err;
	TIM_SetCounter(TIM3,0X7FFF);
}

void DotShootMotorControl(int16_t pwm)
{
	if (pwm >= 0) //正转
	{
    GPIO_ResetBits(GPIOC, GPIO_Pin_1);
		GPIO_SetBits(GPIOC, GPIO_Pin_2);
		TIM9->CCR1 = pwm;
	}else         //反转
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_2);
		GPIO_SetBits(GPIOC, GPIO_Pin_1);
		TIM9->CCR1 = -pwm;
	}
}


/*点射的时钟*/
OS_TICK PreT4Frequ = 0;
OS_TICK PreT4Stuck = 0;
void DotShootTickInit(void)
{
	OS_ERR err;
	PreT4Stuck = PreT4Frequ = OSTimeGet(&err);
}

/*检测卡弹*/
void CheckStuckBullet(OS_TICK now_t,long RealTimePos)
{
	static long pre_pos = 0;
	
	if (now_t - PreT4Stuck >= BULLET_STUCK_TIME)  //最小卡弹时间内
	{		
		if (abs(RealTimePos - pre_pos) <= BULLET_STUCK_ANGLE) //转过的角度小于最小角度
		{
			PID_DotShoot_P.Expect  -= 60;
			PID_DotShoot_VO.Expect = -BULLET_FREQUENCY_VO;
			DotShoot.Status = Stuck;	
		}
		pre_pos    = RealTimePos;
		PreT4Stuck = now_t;
	}			
}

/*拨弹电机旋转量计算*/
float RotateValue(OS_TICK now_t)
{
	if (DotShoot.Status != Stuck)
	{
		if (now_t - PreT4Frequ >= BULLET_FREQUENCY_V)
		{
			PreT4Frequ = now_t;
			return 60;  //360 / 6
		}else
		{
			PreT4Frequ = now_t;
			return 0;
		}
	}else
	{		
		if (now_t - PreT4Frequ >= BULLET_FREQUENCY_V)
		{
			DotShoot.Status = Moving;			
		}		
	}
	return 0;
}

/*单速度环给定值切换*/
int32_t VelocitySwitch(OS_TICK now_t)
{
	if (DotShoot.Status != Stuck)
		return BULLET_FREQUENCY_VO;
	else
	{
		if (now_t - PreT4Frequ >= BULLET_FREQUENCY_V)
		{
			DotShoot.Status = Moving;			
		}				
	}
	return -BULLET_FREQUENCY_VO;
}

void ContinuousControl(uint8_t fric_status,uint8_t mode,OS_TICK now_t,int16_t *pwm)
{
	 if (fric_status == 2)  //开拨弹轮
	 {
		 if (mode)  //速度和位置闭环
		 {
			 PID_DotShoot_P.Expect += RotateValue(now_t); 
			 PID_DotShoot_V.Expect = PID_Calc(&PID_DotShoot_P);   //外环位置环
			 *pwm = (int16_t)PID_Calc(&PID_DotShoot_V);            //内环速度环
		 }else      //只闭速度环
		 {
			 //PID_DotShoot_VO.Expect = VelocitySwitch(now_t);
			 PID_DotShoot_VO.Expect = 80;
			 *pwm = (int16_t)PID_Calc(&PID_DotShoot_VO);
		 }		 
	 }else        //关拨弹轮
	 {
		 if (mode)  //速度和位置闭环
		 {
			 PID_DotShoot_V.Expect = PID_Calc(&PID_DotShoot_P);   //外环位置环
			 *pwm = (int16_t)PID_Calc(&PID_DotShoot_V);            //内环速度环			 		 
		 }else      //只闭速度环
		 {
			 PID_DotShoot_VO.Expect = 0;
			 *pwm = (int16_t)PID_Calc(&PID_DotShoot_VO);	
			 *pwm = 0;
		 }
	 }	
}

/*点射控制*/
void DotShootControl(uint8_t fric_status,uint8_t mode)
{
	 int16_t pwm;
	 OS_TICK now_t;	
	 OS_ERR err;
	
	 now_t = OSTimeGet(&err);	
	 GetEncoderFeedBack(&DotShoot);
	 PID_DotShoot_P.Measured = DotShoot.Position;
	 PID_DotShoot_VO.Measured = PID_DotShoot_V.Measured = DotShoot.Speed;

	 //检查是否卡弹
//	 CheckStuckBullet(now_t,DotShoot.Position);
	
	 //连续旋转
	 ContinuousControl(fric_status,mode,now_t,&pwm);
	 
	 //输出控制
	 DotShootMotorControl(pwm);
}
