/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "includes.h"
#include "stm32f4xx_it.h"
#include "Task_SysInit.h"

#include "Driver_CAN.h"
#include "Driver_DBUS.h"
#include "Driver_ComputerVision.h"

#include "Task_Monitor.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

/** 
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	  OSIntEnter();		
    OSTimeTick();                     
    OSIntExit();    
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/*�����������Ƿ�׼����*/
void EXTI4_IRQHandler(void)
{
	 CPU_SR_ALLOC();
	 OS_CRITICAL_ENTER();
	 OSIntNestingCtr++;
	 OS_CRITICAL_EXIT();
	
	 EXTI->PR = EXTI_Line4;  //�����־
	
	 OSIntExit();
}

/*ң�������ݽ���*/
void USART1_IRQHandler(void)
{
	 CPU_SR_ALLOC();
	 OS_CRITICAL_ENTER();
	 OSIntNestingCtr++;
	 OS_CRITICAL_EXIT();
	
	/*�����־*/
	(void)USART1->SR;
	(void)USART1->DR;			
	DMA_Cmd(DMA2_Stream2, DISABLE);
	
	/*���ݳ�����ȷ*/
	if (DMA2_Stream2->NDTR == DBUS_LENGTH)
	{
		RemoterFrameRate++;
		/*ң��������*/
		DBUS_Decode();
	}
	
	//����DMA
	DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
	while(DMA_GetCmdStatus(DMA2_Stream2) != DISABLE);
	DMA_SetCurrDataCounter(DMA2_Stream2, DBUS_LENGTH);
	DMA_Cmd(DMA2_Stream2, ENABLE);
	
	 OSIntExit();

}

void USART3_IRQHandler(void)
{
	uint8_t res;
	 CPU_SR_ALLOC();
	 OS_CRITICAL_ENTER();
	 OSIntNestingCtr++;
	 OS_CRITICAL_EXIT();

	(void)USART3->SR;
	(void)USART3->DR;
	
	res = USART_ReceiveData(USART3);
	MiniPCRec(res);

	 OSIntExit();
}

/*CAN1��������ж�*/
void CAN1_TX_IRQHandler(void)
{
	 CPU_SR_ALLOC();
	 OS_CRITICAL_ENTER();
	 OSIntNestingCtr++;
	 OS_CRITICAL_EXIT();
	
  CAN_ClearITPendingBit(CAN1,CAN_IT_TME);  
	
	 OSIntExit();
}

/*CAN1�����ж�*/
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg rx_message;
	
	 CPU_SR_ALLOC();
	 OS_CRITICAL_ENTER();
	 OSIntNestingCtr++;
	 OS_CRITICAL_EXIT();
	
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
    CAN1_Process(rx_message); //CAN���մ���
	
	 OSIntExit();
}

/*CAN2��������ж�*/
u8 suc = 0;
void CAN2_TX_IRQHandler(void)
{
	 CPU_SR_ALLOC();
	 OS_CRITICAL_ENTER();
	 OSIntNestingCtr++;
	 OS_CRITICAL_EXIT();
	
  CAN_ClearITPendingBit(CAN2,CAN_IT_TME);   
  suc = 1;

	 OSIntExit();
}

/*CAN2�����ж�*/
void CAN2_RX0_IRQHandler(void)
{
    CanRxMsg rx_message;
	
	 CPU_SR_ALLOC();
	 OS_CRITICAL_ENTER();
	 OSIntNestingCtr++;
	 OS_CRITICAL_EXIT();
	
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
		CAN_Receive(CAN2, CAN_FIFO0, &rx_message);
    CAN2_Process(rx_message); //CAN���մ���
	
	 OSIntExit(); 
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
