#include "Driver_IIC.h"
#include <stdio.h>

#define IIC_DELAY_4 IIC_Delay(10)  //4US
#define IIC_DELAY_1 IIC_Delay(10)  //4US//1US
#define IIC_DELAY_2 IIC_Delay(10)  //4US//2US



//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA_H;	  	  
	IIC_SCL_H;
	IIC_DELAY_4;
 	IIC_SDA_L;//START:when CLK is high,DATA change form high to low 
	IIC_DELAY_4;
	IIC_SCL_L;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL_L;
	IIC_SDA_L;//STOP:when CLK is high DATA change form low to high
 	IIC_DELAY_4;
	IIC_SCL_H; 
	IIC_SDA_H;//����I2C���߽����ź�
	IIC_DELAY_4;							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA_H;IIC_DELAY_1;	   
	IIC_SCL_H;IIC_DELAY_1;	 
	while(IIC_SDA_Read)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_L;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCL_L;
	SDA_OUT();
	IIC_SDA_L;
	IIC_DELAY_2;
	IIC_SCL_H;
	IIC_DELAY_2;
	IIC_SCL_L;
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_SCL_L;
	SDA_OUT();
	IIC_SDA_H;
	IIC_DELAY_2;
	IIC_SCL_H;
	IIC_DELAY_2;
	IIC_SCL_L;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	  SDA_OUT(); 	    
    IIC_SCL_L;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
			if ((txd&0x80)>>7)
				IIC_SDA_H;
			else
				IIC_SDA_L;
      txd<<=1; 	  
		  IIC_DELAY_2;   //��TEA5767��������ʱ���Ǳ����
		  IIC_SCL_H;
		  IIC_DELAY_2; 
		  IIC_SCL_L;	
		  IIC_DELAY_2;
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	  {
			IIC_SCL_L; 
			IIC_DELAY_2;
			IIC_SCL_H;
			receive<<=1;
			if(IIC_SDA_Read)receive++;   
			IIC_DELAY_1; 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}

//1 ʧ��
//0 �ɹ�
u8 IIC_WriteData(u8 dev_addr,u8 reg_addr,u8 data)
{
	IIC_Start();
	IIC_Send_Byte(dev_addr);
	if (IIC_Wait_Ack())
		return 1;
  IIC_Send_Byte(reg_addr);
 	if (IIC_Wait_Ack())
		return 1;
  IIC_Send_Byte(data);
 	if (IIC_Wait_Ack())
		return 1;
  IIC_Stop();
  return 0;	
}

//1 ʧ��
//0 �ɹ�
u8 IIC_ReadData(u8 dev_addr,u8 reg_addr,u8 *pdata,u8 count)
{
	u8 i;
	IIC_Start();
	IIC_Send_Byte(dev_addr);
	if (IIC_Wait_Ack())
		return 1;
	IIC_Send_Byte(reg_addr);
	if (IIC_Wait_Ack())
		return 1;
	IIC_Start();
	IIC_Send_Byte(dev_addr+1);
	if (IIC_Wait_Ack())
		return 1;
  for(i=0;i<(count-1);i++)
  {
		*pdata=IIC_Read_Byte(1);
		pdata++;
  }
  *pdata=IIC_Read_Byte(1);
  IIC_Stop(); 	
	return 0;
}


//����us����ʱ
void IIC_Delay(u32 time)
{
	while(time--);
}
