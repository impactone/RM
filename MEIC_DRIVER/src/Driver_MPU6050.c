#include "Driver_MPU6050.h"


int GyroOffset[3] = {0};   //���������
float MagOffset[6] = {0};  //���������
Attitude CloudAttitude;

/*  @brief ��ʼ�������Ǻʹ�����
 *  @return 0  ��ʼ���ɹ�
 *          !0 ��ʼ��ʧ��
 */
uint16_t MPU6050_5883L_Init(void)
{
	u16 res = 0x00;
	OS_ERR err;
	
	/*��������*/
	res |= IIC_WriteData(MPU6050_DEVICE_ADDRESS,PWR_MGMT_1,0x01)   << 1;         //�������״̬
	res |= IIC_WriteData(MPU6050_DEVICE_ADDRESS,CONFIG,0x03)       << 2;         //���õ�ͨ�˲���
	res |= IIC_WriteData(MPU6050_DEVICE_ADDRESS,GYRO_CONFIG,0x10)  << 3;         //����������  ��1000dps
	res |= IIC_WriteData(MPU6050_DEVICE_ADDRESS,ACCEL_CONFIG,0x00) << 4;         //���ٶȼ�����  ��2g
	res |= IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_PIN_CFG,0x02)  << 5;  
	res |= IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_ENABLE,0x00)   << 6;         //�ر������ж�
  res |= IIC_WriteData(MPU6050_DEVICE_ADDRESS,MPU6050_RA_USER_CTRL,0x00) << 7; //�ر�Masterģʽ
	res |= IIC_WriteData(MPU6050_DEVICE_ADDRESS,SMPLRT_DIV,0x01)   << 8;         //�����ǲ�����	
	res |= IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_ENABLE,0x01)   << 9;         //��INT
	
	/*��������*/
  res |= IIC_WriteData(HMC5883_ADDRESS, HMC58X3_R_CONFA,0x70) << 10;
	res |= IIC_WriteData(HMC5883_ADDRESS, HMC58X3_R_CONFB,0xA0) << 11;
	res |= IIC_WriteData(HMC5883_ADDRESS, HMC58X3_R_MODE,0x00)  << 12;
	res |= IIC_WriteData(HMC5883_ADDRESS, HMC58X3_R_CONFA,0x18) << 13; 
	OSTimeDly(500,OS_OPT_TIME_DLY,&err); /*����ʱ���ɹأ����ȷ��ȫ������*/

  /*�Լ�ģʽ*/
//  res |= IIC_SingleSend(HMC5883_ADDRESS, HMC58X3_R_CONFA,0x71) << 9;
//	res |= IIC_SingleSend(HMC5883_ADDRESS, HMC58X3_R_CONFB,0xA0) << 10;
//	res |= IIC_SingleSend(HMC5883_ADDRESS, HMC58X3_R_MODE,0x00)  << 11;
//	OSTimeDly(500,OS_OPT_TIME_DLY,&err);
	
	/*�Ƿ���ʧ��*/
	if (res)
	{
	  printf("MPU Error Code:%d\r\n",res);
		//return res;
	}

	/*������ȷ�����*/
	Gyro_Cali();

	/*�����Ƽ������*/
//	Mag_Calc();
	
	/*��ʼ����Ԫ��*/
//  InitQ(&CloudAttitude);
	
	return 0;
}

/*  @brief ȷ�����������
 *  @return ��
 */
void Mag_Cali(void)
{
	static s16 MagMaxMinX[2] = {0};
	static s16 MagMaxMinY[2] = {0};
	static s16 MagMaxMinZ[2] = {0};	

  int16_t mx,my,mz;
	
  Get_HML5883L_Data(&mx,&my,&mz);

  /*���˲�*/
	if (mx >= MagMaxMinX[0]) MagMaxMinX[0] = 0.2*MagMaxMinX[0] + 0.8*mx;
	if (mx <= MagMaxMinX[1]) MagMaxMinX[1] = 0.2*MagMaxMinX[1] + 0.8*mx;
	if (my >= MagMaxMinY[0]) MagMaxMinY[0] = 0.2*MagMaxMinY[0] + 0.8*my;
	if (my <= MagMaxMinY[1]) MagMaxMinY[1] = 0.2*MagMaxMinY[1] + 0.8*my;
	if (mz >= MagMaxMinZ[0]) MagMaxMinZ[0] = 0.2*MagMaxMinZ[0] + 0.8*mz;
	if (mz <= MagMaxMinZ[1]) MagMaxMinZ[1] = 0.2*MagMaxMinZ[1] + 0.8*mz;

  /*�����˲�*/
//	if (mx >= MagMaxMinX[0]) MagMaxMinX[0] = mx;
//	if (mx <= MagMaxMinX[1]) MagMaxMinX[1] = mx;
//	if (my >= MagMaxMinY[0]) MagMaxMinY[0] = my;
//	if (my <= MagMaxMinY[1]) MagMaxMinY[1] = my;
//	if (mz >= MagMaxMinZ[0]) MagMaxMinZ[0] = mz;
//	if (mz <= MagMaxMinZ[1]) MagMaxMinZ[1] = mz;

	printf("MaxX = %d MinX = %d MaxY = %d MinY = %d MaxZ = %d MinZ = %d\r\n",
	MagMaxMinX[0],MagMaxMinX[1],MagMaxMinY[0],MagMaxMinY[1],MagMaxMinZ[0],MagMaxMinZ[1]);

}

/*  @brief ������������
 *  @return ��
 */
void Mag_Calc(void)
{
  MagOffset[0] = (MagMaxX + MagMinX) / 2.0f;
	MagOffset[1] = (MagMaxY + MagMinY) / 2.0f;
	MagOffset[2] = (MagMaxZ + MagMinZ) / 2.0f;	
	MagOffset[3] = 1.0f;
	MagOffset[4] = (MagMaxX - MagMinX) / (MagMaxY - MagMinY);
	MagOffset[5] = (MagMaxX - MagMinX) / (MagMaxZ - MagMinZ);
}


/*  @brief ȷ�����������
 *  @return ��
 */
void Gyro_Cali(void)
{
	int16_t ax,ay,az,gx,gy,gz;
	int16_t cnt = 600;    //��cnt�����ֵ
	int16_t tmp_cnt = 0;
	OS_ERR err;
	
	while (cnt--)
	{
		if (!Get_MPU6050_Data(&ax,&ay,&az,&gx,&gy,&gz))
		{
			tmp_cnt++;
			GyroOffset[0] += gx;
			GyroOffset[1] += gy;
			GyroOffset[2] += gz;	
			OSTimeDly(2,OS_OPT_TIME_DLY,&err);			
		}
	}				
	
	GyroOffset[0] /= tmp_cnt;
	GyroOffset[1] /= tmp_cnt;
	GyroOffset[2] /= tmp_cnt;
	
}

/*  @brief ��ȡ6������������
 *  @return 0  ��ȡ�ɹ�
 *          1  ��ȡʧ��
 */
int16_t temp = 0;
uint8_t Get_MPU6050_Data(int16_t *ax,int16_t *ay,int16_t *az,int16_t *gx,int16_t *gy,int16_t *gz)
{
	uint8_t buf[14];
	if (!IIC_ReadData(MPU6050_DEVICE_ADDRESS,MPU6050_DATA_START,buf,14))
	{
		*ax = ((u16)buf[0] << 8) | buf[1];
		*ay = ((u16)buf[2] << 8) | buf[3];
		*az = ((u16)buf[4] << 8) | buf[5];
		temp = ((u16)buf[6] << 8) | buf[7]; /*�¶�*/
		*gx = ((u16)buf[8]  << 8) | buf[9];
		*gy = ((u16)buf[10] << 8) | buf[11];
		*gz = ((u16)buf[12] << 8) | buf[13];	
    IIC_ReadData(HMC5883_ADDRESS,HMC58X3_R_XM,buf,0);
    return 0;		
	}else
	{
		return 1;
	}
}

/*  @brief ��ȡ�������������
 *  @return 0  ��ȡ�ɹ�
 *          1  ��ȡʧ��
 */
uint8_t Get_HML5883L_Data(int16_t *mx,int16_t *my,int16_t *mz)
{
	uint8_t buf[10];
	if (!IIC_ReadData(HMC5883_ADDRESS,HMC58X3_R_XM,buf,6))
	{
		*mx = ((u16)buf[4] << 8) | buf[5];
		*my = ((u16)buf[0] << 8) | buf[1];
		*mz = ((u16)buf[2] << 8) | buf[3];
		IIC_ReadData(HMC5883_ADDRESS,HMC58X3_R_XM,buf,0);
		return 0;
	}else
	{
		return 1;
	}
}

/*�����˲���*/
void DataFilter(int16_t *fifo)
{
	//Digtal Filter!
	static int16_t acc_buf[3][10] = {0};
	int16_t sum[3] = {0};
	for (uint8_t i = 0;i < 9;i++)
	{
		acc_buf[0][i] = acc_buf[0][i+1];
		acc_buf[1][i] = acc_buf[1][i+1];
		acc_buf[2][i] = acc_buf[2][i+1];
	}
	acc_buf[0][9] = fifo[3];
	acc_buf[1][9] = fifo[4];
	acc_buf[2][9] = fifo[5];	
	
	for (uint8_t i = 0;i < 10;i++)
	{
		sum[0] += acc_buf[0][i];
		sum[1] += acc_buf[1][i];
		sum[2] += acc_buf[2][i];
	}
	
	fifo[0] = sum[0] / 10;
	fifo[1] = sum[1] / 10;
	fifo[2] = sum[2] / 10;
	
}

/*��ȡ9������*/
void Get_9Motion_Data(int16_t *fifo)
{
		int16_t mx,my,mz;
		int16_t ax,ay,az,gx,gy,gz;
		Get_MPU6050_Data(&ax,&ay,&az,&gx,&gy,&gz);
		Get_HML5883L_Data(&mx,&my,&mz);
	  
		fifo[0] = (s16)ax;
		fifo[1] = (s16)ay;
		fifo[2] = (s16)az;
		fifo[3] = (s16)gx - GyroOffset[0];
		fifo[4] = (s16)gy - GyroOffset[1];
		fifo[5] = (s16)gz - GyroOffset[2];
//	  fifo[5] = -fifo[5];    //ȥ���õķ��ŷ�T�Ͱ壬FUCK��
		fifo[6] = ((s16)mx - MagOffset[0]) * MagOffset[3];
		fifo[7] = ((s16)my - MagOffset[1]) * MagOffset[4];
		fifo[8] = ((s16)mz - MagOffset[2]) * MagOffset[5];

	  //DataFilter(fifo);
}


