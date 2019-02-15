#ifndef __DRIVER_IIC_H
#define __DRIVER_IIC_H

#include "stm32f4xx.h"


#define IIC_SCL_L		  GPIO_ResetBits(GPIOB, GPIO_Pin_8)
#define IIC_SCL_H		  GPIO_SetBits(GPIOB, GPIO_Pin_8)
#define IIC_SDA_L		  GPIO_ResetBits(GPIOB, GPIO_Pin_9)
#define IIC_SDA_H		  GPIO_SetBits(GPIOB, GPIO_Pin_9)
#define	IIC_SDA_Read	GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)

//IO��������
#define SDA_IN()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//PB9����ģʽ
#define SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9���ģʽ

//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�
void IIC_Delay(u32 time);

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  
u8 IIC_WriteData(u8 dev_addr,u8 reg_addr,u8 data);
u8 IIC_ReadData(u8 dev_addr,u8 reg_addr,u8 *pdata,u8 count);


#endif
