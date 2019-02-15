#include "Driver_ComputerVision.h"

DataTransType VisionPackage[VISION_DATA_LENGTH+2];
uint16_t      Enemy_Index = 0;
EnemyType     Enemy[FITTING_SAMPLE_NUM];

uint16_t rec = 0;
uint16_t  buff_index = 0;
uint32_t  pc_buff[100] = {0};

/*数据接收和处理*/
void MiniPCRec(uint8_t res)
{
	pc_buff[buff_index] = res;
	buff_index++;
	
	/*数据过长，传输失败*/
	if (buff_index > PC_BUFF_LENGTH)
	{
		buff_index = 0;
	}
	/*判断帧头和帧尾*/
	if (buff_index >= 2)
	{
		if (pc_buff[0] == 0x0D)
		{
			if (pc_buff[1] == 0x0A)
			{
				if (pc_buff[PC_BUFF_LENGTH - 1] == 0x0D)
				{
					if (pc_buff[PC_BUFF_LENGTH - 2] == 0x0A)
					{
						rec = buff_index;
						MiniPCFrameRate++;
						MiniPCDecode();
						buff_index = 0;
					}
				}
			}else
			{
				pc_buff[0] = pc_buff[1];
				buff_index = 1;
			}
		}else
		{
			pc_buff[0] = pc_buff[1];
			buff_index = 1;
		}
	}
}

/*数据解码*/
void MiniPCDecode(void)
{
	for (uint8_t i = 0;i < VISION_DATA_LENGTH;i++)
	{
		for (uint8_t j = 0;j < 4;j++)
		{
			VisionPackage[i].data[j] = pc_buff[i*4+j+2];
			pc_buff[i*4+j+2] = 0;
		}
		pc_buff[PC_BUFF_LENGTH - 1] = 0;
		pc_buff[PC_BUFF_LENGTH - 2] = 0;
	}		
}

/*坐标转换*/
void Rotation2Angle(EnemyType src,AngleType *des)
{
	float x = src.X;
	float y = src.Y;
	float z = src.Z;
	
	des->Pitch = atan2(x,z);
	des->Yaw   = atan2(y,sqrt(x * x + z * z));
	
}


#define INF_F 10000000
void CV_YawControl(int16_t *current,float inc_yaw,uint8_t mode)
{
	static uint8_t dir = 0;
	PID_Yaw_P.Measured   = -CloudAttitude.yaw;
	PID_Yaw_V.Measured   = CloudAttitude.gz;
	
	if (inc_yaw != INF_F)
	{
		if (mode)  //角度增量模式
		{
  		PID_Yaw_P.Expect     = PID_Yaw_P.Measured + inc_yaw;
		}
		PID_Yaw_V.Expect     = PID_Calc(&PID_Yaw_P);            //外环位置环计算		
	}else  
	{ //警戒
		if (dir)
		{
			PID_Yaw_V.Expect     = GUARD_AGAINST_SPEED; //以一定速度扫描           
			if (CloudAttitude.ENC_Yaw * 0.0439453125f <= -60)
			{
				dir = !dir;   //转向
			}
		}else
		{
			PID_Yaw_V.Expect     = -GUARD_AGAINST_SPEED; //以一定速度扫描  
			if (CloudAttitude.ENC_Yaw * 0.0439453125f >= 60)
			{
				dir = !dir;   //转向
			}
		}		
	}
	
	*current             = (int16_t)PID_Calc(&PID_Yaw_V);   //内环速度环计算		
}

void CV_PitchControl(int16_t *current,float inc_pitch,uint8_t mode)
{
	PID_Pitch_P.Measured  = CloudAttitude.ENC_Pitch * 0.0439453125f; //360 / 8192
	PID_Pitch_V.Measured  = -CloudAttitude.gy;
	
	if (inc_pitch != INF_F)
	{
		Ramp_ReInit(&CloudPitch_Ramp);
		if (mode)
		{
			PID_Pitch_P.Expect    = PID_Pitch_P.Measured + inc_pitch;			
		}
	}else
	{ //警戒
		PID_Pitch_P.Measured  *=  Ramp_Calc(&CloudPitch_Ramp);
		PID_Pitch_P.Expect     =  0;		
	}

	PID_Pitch_V.Expect    = PID_Calc(&PID_Pitch_P);           //外环位置环计算
	*current              = (int16_t)PID_Calc(&PID_Pitch_V);  //内环速度环计算
}

/*清空敌人数据*/
void CV_ClearEnemy(void)
{
	for (uint8_t i = 0;i < FITTING_SAMPLE_NUM;i++)
	{
		Enemy[i].X = 0;
		Enemy[i].Y = 0;
		Enemy[i].Z = 0;
		Enemy[i].T = 0;
	}
	Enemy_Index = 0;
}

/*线性拟合*/
uint8_t CV_Coordinate_1Fitting(EnemyType *fitting)
{
	float m = FITTING_SAMPLE_NUM;
	float A = 0,B = 0;
	
	float Cx = 0,Dx = 0;
	float Cy = 0,Dy = 0;
	
	float delta;
	
	float a_x[2];  //拟合出的X坐标系数
	float a_y[2];  //拟合出的Y坐标系数
	
	float T = Enemy[Enemy_Index].T + 10;  //拟合出10ms以后的数据

	/*判断数据长度是否足够*/
	for (uint8_t i = 0; i < FITTING_SAMPLE_NUM; i++)
	{
		if (Enemy[i].T == 0)
		{
			return 0;
		}
	}
	
	/*系数计算*/
	for (uint8_t i = 0; i < FITTING_SAMPLE_NUM; i++)
	{
		uint8_t j = (i + Enemy_Index) % FITTING_SAMPLE_NUM;  //环形数据的结构
		
		A += Enemy[j].T;
		B += Enemy[j].T * Enemy[j].T;

		Cx += Enemy[j].X;
		Dx += Enemy[j].X * Enemy[j].T;
		
		Cy += Enemy[j].Y;
		Dy += Enemy[j].Y * Enemy[j].T;
	}

	/*预计算*/
	delta = -A * A + B * m;
	if (delta == 0)
	{
		return 0;
	}
	
	/*拟合X坐标*/
	a_x[0] = (B * Cx - A * Dx) / delta;
	a_x[1] = (Dx * m - A * Cx) / delta;
	fitting->X = a_x[0] + a_x[1] * T ;

	/*拟合Y坐标*/
	a_y[0] = (B * Cy - A * Dy) / delta;
	a_y[1] = (Dy * m - A * Cy) / delta;
	fitting->Y = a_y[0] + a_y[1] * T;

	/*拟合数据的滑动*/
	Enemy_Index = (Enemy_Index + 1) % FITTING_SAMPLE_NUM;
	Enemy[Enemy_Index].X = fitting->X;
	Enemy[Enemy_Index].Y = fitting->Y;
	Enemy[Enemy_Index].T = T;
	
	return 1;
}

/*二项式(抛物线)拟合*/
uint8_t CV_Coordinate_2Fitting(EnemyType *fitting)
{
	float m = FITTING_SAMPLE_NUM;
	float A = 0,B = 0,C = 0,D = 0;
	
	float Ex = 0,Fx = 0,Gx = 0;
	float Ey = 0,Fy = 0,Gy = 0;
	
	float A2,B2,B3,C2;
	
	float delta_x;
	float delta_y;
	
	float a_x[3];  //拟合出的X坐标系数
	float a_y[3];  //拟合出的Y坐标系数
	
	float T = Enemy[Enemy_Index].T + 10;  //拟合出10ms以后的数据

	/*判断数据长度是否足够*/
	for (uint8_t i = 0; i < FITTING_SAMPLE_NUM; i++)
	{
		if (Enemy[i].T == 0)
		{
			return 0;
		}
	}
	
	/*系数计算*/
	for (uint8_t i = 0; i < FITTING_SAMPLE_NUM; i++)
	{
		uint8_t j = (i + Enemy_Index) % FITTING_SAMPLE_NUM;  //环形数据的结构
		
		A += Enemy[j].T;
		B += Enemy[j].T * Enemy[j].T;
		C += Enemy[j].T * Enemy[j].T * Enemy[j].T;
		D += Enemy[j].T * Enemy[j].T * Enemy[j].T * Enemy[j].T;
		
		Ex += Enemy[j].X;
		Fx += Enemy[j].X * Enemy[j].T;
		Gx += Enemy[j].X * Enemy[j].T * Enemy[j].T;
		
		Ey += Enemy[j].Y;
		Fy += Enemy[j].Y * Enemy[j].T;
		Gy += Enemy[j].Y * Enemy[j].T * Enemy[j].T;
	}

	/*预计算*/
	A2 = A * A;
	B2 = B * B;
	B3 = B * B * B;
	C2 = C * C;	
	
	/*拟合X坐标*/
	delta_x = (D * A2 - 2.0f * A * B * C + B3 - D * m * B + m * C2);	
	if (delta_x == 0)
	{
		return 0;
	}
	a_x[0] = (Gx * B2 - Fx * B * C - D * Ex * B + Ex * C2 - A * Gx * C + A * D * Fx) / delta_x;
	a_x[1] = (B2 * Fx - A * B * Gx - B * C * Ex + A * D * Ex + C * Gx * m - D * Fx * m) / delta_x;
	a_x[2] = (Gx * A2 - Fx * A * B - C * Ex * A + Ex * B2 - Gx * m * B + C * Fx * m) / delta_x;
	fitting->X = a_x[0] + a_x[1] * T + a_x[2] * T * T;

	/*拟合Y坐标*/
	delta_y = (D * A2 - 2.0f * A * B * C + B3 - D * m * B + m * C2);	
	if (delta_y == 0)
	{
		return 0;
	}		
	a_y[0] = (Gy * B2 - Fy * B * C - D * Ey * B + Ey * C2 - A * Gy * C + A * D * Fy) / delta_y;
	a_y[1] = (B2 * Fy - A * B * Gy - B * C * Ey + A * D * Ey + C * Gy * m - D * Fy * m) / delta_y;
	a_y[2] = (Gy * A2 - Fy * A * B - C * Ey * A + Ey * B2 - Gy * m * B + C * Fy * m) / delta_y;	
	fitting->Y = a_y[0] + a_y[1] * T + a_y[2] * T * T;

	/*拟合数据的滑动*/
//	Enemy_Index = (Enemy_Index + 1) % FITTING_SAMPLE_NUM;
//	Enemy[Enemy_Index].X = fitting->X;
//	Enemy[Enemy_Index].Y = fitting->Y;
//	Enemy[Enemy_Index].T = T;
	
	return 1;
}

/*视觉模式*/
uint16_t Scan_Cnt = 0;
void CVProcess(void)
{
	int16_t PitchCurrent;
	int16_t YawCurrent;
	EnemyType Fitting;
	AngleType IncAngle = {INF_F,INF_F};
	
	SetFricMotor(1);
	
	if (rec)
	{
		Enemy[Enemy_Index].X  = VisionPackage[0].Trans;
		Enemy[Enemy_Index].Y  = VisionPackage[1].Trans;
		Enemy[Enemy_Index].Z  = VisionPackage[2].Trans;
		Enemy[Enemy_Index].T  = VisionPackage[3].Trans;
		Enemy_Index = (Enemy_Index + 1) % FITTING_SAMPLE_NUM;
		Scan_Cnt = 0;
		rec = 0;
	}else
	{
		Scan_Cnt++;
	}
	
	if (Scan_Cnt >= 100)
	{
		FricStatus     = 0;
		
		IncAngle.Yaw   = INF_F;
		IncAngle.Pitch = INF_F;
		
		CV_ClearEnemy();
		PID_Reset_SumErr(&PID_CV_ENEMY_X);
		PID_Reset_SumErr(&PID_CV_ENEMY_Y);
	}else
	{
		FricStatus = 2;
		
		//if (CV_Coordinate_1Fitting(&Fitting))
		if (0)
		{
			PID_CV_ENEMY_X.Measured = -Fitting.X;
			PID_CV_ENEMY_Y.Measured = -Fitting.Y;						
		}else
		{
			PID_CV_ENEMY_X.Measured = -Enemy[Enemy_Index - 1 >= 0 ? Enemy_Index - 1 : 0].X + 20;
			PID_CV_ENEMY_Y.Measured = -Enemy[Enemy_Index - 1 >= 0 ? Enemy_Index - 1 : 0].Y - 130;				
		}
		
//			PID_CV_ENEMY_X.Measured = -Enemy[0].X;
//			PID_CV_ENEMY_Y.Measured = -Enemy[0].Y;	
		
		IncAngle.Yaw   = PID_Calc(&PID_CV_ENEMY_X);
		IncAngle.Pitch = PID_Calc(&PID_CV_ENEMY_Y);		
	}	

	CV_YawControl(&YawCurrent,IncAngle.Yaw,!Scan_Cnt);
	CV_PitchControl(&PitchCurrent,IncAngle.Pitch,!Scan_Cnt);
	Send2Cloud(PitchCurrent,YawCurrent);
}
