#ifndef __DRIVER_IIC_H
#define __DRIVER_IIC_H

#include "stm32f4xx.h"


#define IIC_SCL_L		  GPIO_ResetBits(GPIOB, GPIO_Pin_8)
#define IIC_SCL_H		  GPIO_SetBits(GPIOB, GPIO_Pin_8)
#define IIC_SDA_L		  GPIO_ResetBits(GPIOB, GPIO_Pin_9)
#define IIC_SDA_H		  GPIO_SetBits(GPIOB, GPIO_Pin_9)
#define	IIC_SDA_Read	GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)

//IO方向设置
#define SDA_IN()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//PB9输入模式
#define SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9输出模式

//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号
void IIC_Delay(u32 time);

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  
u8 IIC_WriteData(u8 dev_addr,u8 reg_addr,u8 data);
u8 IIC_ReadData(u8 dev_addr,u8 reg_addr,u8 *pdata,u8 count);


#endif
