#include "Task_IMU.h"

volatile float q0 = 1,q1 = 0,q2 = 0,q3 = 0;

/*��̬��������*/
void Task_IMU(void *p_arg)
{
	OS_ERR err;
	(void )p_arg;	
//	float last_pitch = 0;
//  float last_roll  = 0;
//  float last_yaw   = 0;	

	/********��������ʼ�Ƕ�***********
	for (;;)
	{
		OSSemPend(&GyroDRY_Sem,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		IMU_GetPitchRollYaw(&CloudAttitude);		
		
		if (fabs(last_pitch - CloudAttitude.pitch) <= 0.001 && \
			  fabs(last_roll  - CloudAttitude.roll)  <= 0.001 && \
		    fabs(last_yaw   - CloudAttitude.yaw)   <= 0.001)
		{
			printf("Init --> pitch: %f roll: %f yaw: %f\r\n",CloudAttitude.pitch,CloudAttitude.roll,CloudAttitude.yaw);
			CloudAttitude.init_yaw = CloudAttitude.raw_yaw = CloudAttitude.yaw;
			break;
		}
		
		last_pitch = CloudAttitude.pitch;
		last_roll  = CloudAttitude.roll;
		last_yaw   = CloudAttitude.yaw;
		
	}
	OSTaskResume(&Task_SysInitTCB,&err);
	********��������ʼ�Ƕ�***********/
	
	CloudAttitude.yaw = 0;
 
	for (;;)
	{
	  IMU_GetPitchRollYaw(&CloudAttitude);		
		OSTimeDly(2,OS_OPT_TIME_DLY,&err);		
	}
}

uint8_t isInARange(float data,float min,float max)
{
	if (data >= min && \
		  data <= max)
	  return 0;
	else
		return 1;
}

float Rad2Degree(float data)
{
	return data * 57.3f;
}

//��������invSqrt(void)
//��������ƽ�����ĵ���
//�ú����Ǿ����Carmack��ƽ�����㷨��Ч�ʼ��ߣ�ʹ��ħ��0x5f375a86
static float invSqrt(float number) 
{
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );
    return y;
}

/*��ʼ����Ԫ��,�ӿ������ٶ�*/

void InitQ(Attitude *InitAng)
{
	uint8_t cnt = 10;
	uint8_t temp_cnt = 0;
	int acc_mag[6] = {0};
	int16_t fifo[9];
  float InitPitch,InitRoll,InitYaw;
	float Hx,Hy;
	float sinP,sinR,sinY;
	float cosP,cosR,cosY;
	
	OS_ERR err;
  
  /*�ɼ���ʼ����*/
	temp_cnt = cnt;
	while(cnt--)
	{
//		OSSemPend(&GyroDRY_Sem,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		Get_9Motion_Data(fifo);
		acc_mag[0] += fifo[0];
		acc_mag[1] += fifo[1];
		acc_mag[2] += fifo[2];
		acc_mag[3] += fifo[6];
		acc_mag[4] += fifo[7];
		acc_mag[5] += fifo[8];
	}
	acc_mag[0] /= temp_cnt;
	acc_mag[1] /= temp_cnt;
	acc_mag[2] /= temp_cnt;
	acc_mag[3] /= temp_cnt;
	acc_mag[4] /= temp_cnt;
	acc_mag[5] /= temp_cnt;
	
	/*�����ʼ�����Ǻͷ�����*/
	InitPitch = -atan2f(acc_mag[0], acc_mag[2]);
	InitRoll = atan2f(acc_mag[1],acc_mag[2]);
	
	/*Ԥ����*/
	sinP = sinf(InitPitch);
	cosP = cosf(InitPitch);
	sinR = sinf(InitRoll);
	cosR = cosf(InitRoll);
	
	/*��ǲ��������ˮƽ�����ϴ�������*/
	Hx = acc_mag[3] * cosP + acc_mag[4] * sinR * sinP - acc_mag[5] * cosR * sinP;
	Hy = acc_mag[4] * cosR + acc_mag[5] * sinR;
	
	/*�����ʼ�����*/
	InitYaw = atan2f(Hy,Hx);
	InitAng->pitch = Rad2Degree(InitPitch);
	InitAng->roll  = Rad2Degree(InitRoll);	
  InitAng->yaw   = Rad2Degree(InitYaw);
	
	/*�Ƕȼ���*/
	InitPitch *= 0.5f;
	InitRoll  *= 0.5f;
	InitYaw   *= 0.5f;
	
	/*Ԥ����*/
	sinP = sinf(InitPitch);
	sinR = sinf(InitRoll);
	sinY = sinf(InitYaw);
	cosP = cosf(InitPitch);
	cosR = cosf(InitRoll);
	cosY = cosf(InitYaw);
	
	/*ŷ����ת��Ԫ��*/
	q0 = cosY*cosP*cosR + sinY*sinP*sinR;
	q1 = cosY*cosP*sinR - sinY*sinP*cosR;
	q2 = cosY*sinP*cosR + sinY*cosP*sinR;
	q3 = sinY*cosP*cosR - cosY*sinP*sinR;

}

#define Kp 10.0f
#define Ki 0.02f
float UpdateQ(int16_t *fifo)
{
	  static uint32_t PreT = 0;
	  uint32_t NowT;
	  static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f; 
    float ax,ay,az;               
	  float gx,gy,gz;            
	  float mx,my,mz;
		
		OS_ERR err;

    float norm;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez,halfT;
    float tempq0,tempq1,tempq2,tempq3;

    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;   
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;   

		ax = fifo[0];
		ay = fifo[1];
		az = fifo[2];
		gx = fifo[3] * 0.000532f;   //ת���ɻ���ÿ��
    gy = fifo[4] * 0.000532f;	  //ת���ɻ���ÿ��
		gz = -fifo[5] * 0.000532f;   //ת���ɻ���ÿ��
		mx = fifo[6];
		my = fifo[7];
		mz = fifo[8];

  	  //ʱ�����
    NowT = OSTimeGet(&err);
		halfT = (NowT - PreT) / 2000.0f;
		PreT = NowT;
	
    //������ƽ�����㷨
    norm = invSqrt(ax*ax + ay*ay + az*az);       
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    //�ѼӼƵ���ά����ת�ɵ�λ������
    norm = invSqrt(mx*mx + my*my + mz*mz);          
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm; 
    // compute reference direction of flux
    hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
    hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
    hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);         
    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz; 
    // estimated direction of gravity and flux (v and w)
    vx = 2.0f*(q1q3 - q0q2);
    vy = 2.0f*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
    wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
    wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
        exInt = exInt + ex * Ki * halfT;
        eyInt = eyInt + ey * Ki * halfT;	
        ezInt = ezInt + ez * Ki * halfT;
        // �ò���������PI����������ƫ
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
    }
    // ��Ԫ��΢�ַ���
    tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

    // ��Ԫ���淶��
    norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;
		
		//���ؼ��ʱ��
		return halfT;
}

//���û�����
void IMU_GetPitchRoll(Attitude *angles)
{
	angles->pitch = -asin(2.0f * (q0 * q2 -   q1 * q3));  //Pitch
	angles->roll = atan2(2.0f * (q2 * q3 + q0 * q1), 1.0f - 2 * (q1 * q1 + q2 * q2)); //Roll
//	angles->yaw =  -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1);  //Yaw
}

//�������������������Ƿ�����
//���룺��������������
//�����0   ����
//      !0  ������
uint8_t checkMag(int16_t *mag)
{
	uint8_t res = 0;
//	res |= isInARange(mag[0],MagMinX,MagMaxX) << 1;
//	res |= isInARange(mag[1],MagMinY,MagMaxY) << 2;
//	res |= isInARange(mag[2],MagMinZ,MagMaxZ) << 3;
	
	return res;
	
}

/*��Yaw�����������*/
void YawProcess(float *yaw)
{
	static float last_yaw = 0.0f;
	static float now_yaw = 0.0f;
  static int8_t rount_cnt = 0;
	
	last_yaw = now_yaw;
	now_yaw = (*yaw);   //�ɽ�����ν������֮��̫�����ж�ʧ�������
	
	if (now_yaw - last_yaw >= 170.0f)
	{
		rount_cnt--;
	}else if (now_yaw - last_yaw <= -170.0f)
	{
		rount_cnt++;
	}
	
	(*yaw) = now_yaw + rount_cnt * 360;

}

int16_t fifo[9];
void IMU_GetPitchRollYaw(Attitude *angles)
{
	  
	  static float yaw = 0;
	  float halfT;
//	  float Mag_Yaw;
	  float New_Yaw = angles->raw_yaw;
//	  float Hx,Hy;
//    float sin_P,cos_P;
//    float sin_R,cos_R;

	 //Step1 ��ȡ9������
		Get_9Motion_Data(fifo);
	 //Step2 ������Ԫ�� 
		halfT = UpdateQ(fifo);
	
	 //Step3 ��ȡ��Ԫ��������
		IMU_GetPitchRoll(angles);
	
//	 //Steop4 �����ںϴ�����
//	  if (!checkMag(&fifo[6]))
//	  //if (0)
//		{
//			sin_P = sin(angles->pitch);
//			cos_P = cos(angles->pitch);
//			sin_R = sin(angles->roll);
//			cos_R = cos(angles->roll);
//			
//			//��ǲ��������ˮƽ�����ϴ�������
//			Hx = fifo[6] * cos_P + fifo[7] * sin_R * sin_P - fifo[8] * cos_R * sin_P;
//			Hy = fifo[7] * cos_R + fifo[8] * sin_R;
//			
//			//���ô����Ƽ���������
//			Mag_Yaw = Rad2Degree(atan2(Hy,Hx));	
//			
//      //����ֱ�ӻ��ּ���Ƕ�
//      //angles->yaw += fifo[5] * GYRO_SCALE * halfT * 2.0f;			
//			New_Yaw += fifo[5] * GYRO_SCALE * halfT * 2.0f;		

//			//�����ں�
//			if((Mag_Yaw>90 && New_Yaw<-90) || (Mag_Yaw<-90 && New_Yaw>90)) /*��180���ת��*/
//				New_Yaw = -New_Yaw * 0.99f + Mag_Yaw * 0.01f;
//			else
//				New_Yaw = New_Yaw * 0.99f + Mag_Yaw * 0.01f;			
//		}
//		else
//		{ 
//			New_Yaw += fifo[5] * GYRO_SCALE * halfT * 2.0f;
//		}	
		
//		angles->raw_yaw = New_Yaw;	
//		YawProcess(&New_Yaw);
//		angles->yaw = New_Yaw - angles->init_yaw;
//		angles->yaw = New_Yaw;

		/*����ת���ɽǶ�*/
		angles->pitch  = Rad2Degree(angles->pitch);  //Pitch
		angles->roll   = Rad2Degree(angles->roll);   //Roll	
//		angles->yaw    = Rad2Degree(angles->yaw);    //Yaw
		angles->yaw    += fifo[5] * halfT * 2.0f * 0.030481f;
		
		/*��ȡ���ٶ�*/
    angles->gx     = fifo[3] * GYRO_SCALE;
		angles->gy     = fifo[4] * GYRO_SCALE;
		angles->gz     = fifo[5] * GYRO_SCALE;
	}
