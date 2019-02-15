#include "Driver_IIC.h"
#include <stdio.h>

#define IIC_DELAY_4 IIC_Delay(10)  //4US
#define IIC_DELAY_1 IIC_Delay(10)  //4US//1US
#define IIC_DELAY_2 IIC_Delay(10)  //4US//2US



//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA_H;	  	  
	IIC_SCL_H;
	IIC_DELAY_4;
 	IIC_SDA_L;//START:when CLK is high,DATA change form high to low 
	IIC_DELAY_4;
	IIC_SCL_L;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL_L;
	IIC_SDA_L;//STOP:when CLK is high DATA change form low to high
 	IIC_DELAY_4;
	IIC_SCL_H; 
	IIC_SDA_H;//发送I2C总线结束信号
	IIC_DELAY_4;							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
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
	IIC_SCL_L;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
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
//不产生ACK应答		    
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
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	  SDA_OUT(); 	    
    IIC_SCL_L;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
			if ((txd&0x80)>>7)
				IIC_SDA_H;
			else
				IIC_SDA_L;
      txd<<=1; 	  
		  IIC_DELAY_2;   //对TEA5767这三个延时都是必须的
		  IIC_SCL_H;
		  IIC_DELAY_2; 
		  IIC_SCL_L;	
		  IIC_DELAY_2;
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
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
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}

//1 失败
//0 成功
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

//1 失败
//0 成功
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


//用于us级延时
void IIC_Delay(u32 time)
{
	while(time--);
}
