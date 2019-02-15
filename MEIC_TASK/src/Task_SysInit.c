#include "Task_IMU.h"
#include "Task_CanTx.h"
#include "Task_Monitor.h"
#include "Task_Control.h"
#include "Task_SysInit.h"
#include "Task_DebugMsg.h"

#include "Driver_DBUS.h"
#include "Driver_Ramp.h"
#include "Driver_Remoter.h"
#include "Driver_DotShoot.h"
#include "Driver_FricMotor.h"


/*�������ȼ�����*/
#define  TASK_IMU_PRIO            5u
#define  TASK_CANTX_PRIO          7u
#define  TASK_CONTROL_PRIO        6u 
#define  TASK_MONITOR_PRIO        9u
#define  TASK_DEBUGMSG_PRIO       12u

/*�����ջ����С*/
#define  TASK_IMU_STK_SIZE        512u
#define  TASK_CANTX_STK_SIZE      128u
#define  TASK_BLINKY_STK_SIZE     128u
#define  TASK_CONTROL_STK_SIZE    256u
#define  TASK_MONITOR_STK_SIZE    128u
#define  TASK_DEBUGMSG_STK_SIZE   256u
CPU_STK  Task_IMUStk[TASK_IMU_STK_SIZE];
CPU_STK  Task_CanTxStk[TASK_CANTX_STK_SIZE];
CPU_STK  Task_ControlStk[TASK_CONTROL_STK_SIZE];
CPU_STK  Task_MonitorStk[TASK_MONITOR_STK_SIZE];
CPU_STK  Task_DebugMsgStk[TASK_DEBUGMSG_STK_SIZE];

/*������ƿ�*/
OS_TCB  Task_IMUTCB;
OS_TCB  Task_CanTxTCB;
OS_TCB  Task_ControlTCB;
OS_TCB  Task_MonitorTCB;
OS_TCB  Task_DebugMsgTCB;

/*�ź���*/
OS_SEM GyroDRY_Sem;

/*CAN��Ϣ����*/
OS_Q CanMsgQue;

/*��ʼ����*/
void  Task_SysInit (void *p_arg)
{
    OS_ERR  err;
	  CPU_SR_ALLOC();
	
   (void)p_arg;
	
	  /*������Ϣ����*/
	  OSQCreate(&CanMsgQue,"CanMsgQue",10,&err);
	
	  /*ϵͳ��ģ���ʼ��*/
	  SysModule_Init();  
    
	  /*���ֲ�����ʼ��*/
	  PID_Init();
	  Mode_Init();
	  DBUS_Init();  
	  DotShoot_Init();
		GMEncoder_Init();
		Ramp_InitAllRamp();
	
	  /*��Ƭ�������ʼ��*/
		BSP_Init();
	  OSTimeDly(500,OS_OPT_TIME_DLY,&err);		

	  /*����ģ���ʼ��*/  
		ZGYRO_Reset();
	  MPU6050_5883L_Init();
		
	
	  /*�����ٽ���*/
	  OS_CRITICAL_ENTER();
	  OSSemSet(&GyroDRY_Sem,0,&err);
	   	   
		/*������������*/
		OSTaskCreate((OS_TCB       *)&Task_DebugMsgTCB,         
							 (CPU_CHAR     *)"DebugMsg Task",
							 (OS_TASK_PTR   )Task_DebugMsg,
							 (void         *)0u,
							 (OS_PRIO       )TASK_DEBUGMSG_PRIO,
							 (CPU_STK      *)Task_DebugMsgStk,
							 (CPU_STK_SIZE  )TASK_DEBUGMSG_STK_SIZE / 10u,
							 (CPU_STK_SIZE  )TASK_DEBUGMSG_STK_SIZE,
							 (OS_MSG_QTY    )0u,
							 (OS_TICK       )0u,
							 (void         *)0u,
							 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
							 (OS_ERR       *)&err);
							 
		/*������̬��������*/					
		OSTaskCreate((OS_TCB       *)&Task_IMUTCB,         
							 (CPU_CHAR     *)"IMU Task",
							 (OS_TASK_PTR   )Task_IMU,
							 (void         *)0u,
							 (OS_PRIO       )TASK_IMU_PRIO,
							 (CPU_STK      *)Task_IMUStk,
							 (CPU_STK_SIZE  )TASK_IMU_STK_SIZE / 10u,
							 (CPU_STK_SIZE  )TASK_IMU_STK_SIZE,
							 (OS_MSG_QTY    )0u,
							 (OS_TICK       )0u,
							 (void         *)0u,
							 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
							 (OS_ERR       *)&err);
							 
//		 /*�˳��ٽ���*/
//		 OS_CRITICAL_EXIT();
//		
//    /*����̬��������֮���ٽ�����һ��*/							 
//		OS_TaskSuspend(NULL,&err);
//					
//	  /*�����ٽ���*/
//	  OS_CRITICAL_ENTER();
//	  OSSemSet(&GyroDRY_Sem,0,&err);
							 
		/*������������*/					
		OSTaskCreate((OS_TCB       *)&Task_ControlTCB,         
							 (CPU_CHAR     *)"Control Task",
							 (OS_TASK_PTR   )Task_Control,
							 (void         *)0u,
							 (OS_PRIO       )TASK_CONTROL_PRIO,
							 (CPU_STK      *)Task_ControlStk,
							 (CPU_STK_SIZE  )TASK_CONTROL_STK_SIZE / 10u,
							 (CPU_STK_SIZE  )TASK_CONTROL_STK_SIZE,
							 (OS_MSG_QTY    )0u,
							 (OS_TICK       )0u,
							 (void         *)0u,
							 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
							 (OS_ERR       *)&err);
							 
		/*����CAN��������*/					
		OSTaskCreate((OS_TCB       *)&Task_CanTxTCB,         
							 (CPU_CHAR     *)"CanTx Task",
							 (OS_TASK_PTR   )Task_CanTx,
							 (void         *)0u,
							 (OS_PRIO       )TASK_CANTX_PRIO,
							 (CPU_STK      *)Task_CanTxStk,
							 (CPU_STK_SIZE  )TASK_CANTX_STK_SIZE / 10u,
							 (CPU_STK_SIZE  )TASK_CANTX_STK_SIZE,
							 (OS_MSG_QTY    )0u,
							 (OS_TICK       )0u,
							 (void         *)0u,
							 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
							 (OS_ERR       *)&err);					

		/*�����������*/					
		OSTaskCreate((OS_TCB       *)&Task_MonitorTCB,         
							 (CPU_CHAR     *)"Monitor Task",
							 (OS_TASK_PTR   )Task_Monitor,
							 (void         *)0u,
							 (OS_PRIO       )TASK_MONITOR_PRIO,
							 (CPU_STK      *)Task_MonitorStk,
							 (CPU_STK_SIZE  )TASK_MONITOR_STK_SIZE / 10u,
							 (CPU_STK_SIZE  )TASK_MONITOR_STK_SIZE,
							 (OS_MSG_QTY    )0u,
							 (OS_TICK       )0u,
							 (void         *)0u,
							 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
							 (OS_ERR       *)&err);		
							 
		 /*�˳��ٽ���*/
		 OS_CRITICAL_EXIT();
							 
	   /*�ɵ��Լ�*/
	   OSTaskDel(NULL , & err);
}

/*ϵͳ����ܳ�ʼ��*/
void SysModule_Init(void)
{
	OS_ERR  err;
	  
	  /*CPU���ó�ʼ��*/
	  CPU_Init();   
	
    /*��ʼ��SysTick��ʱ��*/	
    OS_CPU_SysTickInit(BSP_CPU_ClkFreq()
	                     /(CPU_INT32U)OSCfg_TickRate_Hz);   
	
	  /*�ڴ�����ʼ��*/
//	  Mem_Init();     
	
#if OS_CFG_STAT_TASK_EN > 0u
	  /*����ͳ������*/
    OSStatTaskCPUUsageInit(&err);                       
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();
#endif
	
#if OS_CFG_SCHED_ROUND_ROBIN_EN
    /*����ʱ��Ƭ����*/
		OSSchedRoundRobinCfg((CPU_BOOLEAN )DEF_ENABLED,
												 (OS_TICK     )0,
												 (OS_ERR     *)&err);
#endif	
	
}
