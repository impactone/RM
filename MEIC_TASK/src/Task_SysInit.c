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


/*任务优先级定义*/
#define  TASK_IMU_PRIO            5u
#define  TASK_CANTX_PRIO          7u
#define  TASK_CONTROL_PRIO        6u 
#define  TASK_MONITOR_PRIO        9u
#define  TASK_DEBUGMSG_PRIO       12u

/*任务堆栈及大小*/
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

/*任务控制块*/
OS_TCB  Task_IMUTCB;
OS_TCB  Task_CanTxTCB;
OS_TCB  Task_ControlTCB;
OS_TCB  Task_MonitorTCB;
OS_TCB  Task_DebugMsgTCB;

/*信号量*/
OS_SEM GyroDRY_Sem;

/*CAN消息队列*/
OS_Q CanMsgQue;

/*开始任务*/
void  Task_SysInit (void *p_arg)
{
    OS_ERR  err;
	  CPU_SR_ALLOC();
	
   (void)p_arg;
	
	  /*创建消息队列*/
	  OSQCreate(&CanMsgQue,"CanMsgQue",10,&err);
	
	  /*系统各模块初始化*/
	  SysModule_Init();  
    
	  /*各种参数初始化*/
	  PID_Init();
	  Mode_Init();
	  DBUS_Init();  
	  DotShoot_Init();
		GMEncoder_Init();
		Ramp_InitAllRamp();
	
	  /*单片机外设初始化*/
		BSP_Init();
	  OSTimeDly(500,OS_OPT_TIME_DLY,&err);		

	  /*板载模块初始化*/  
		ZGYRO_Reset();
	  MPU6050_5883L_Init();
		
	
	  /*进入临界区*/
	  OS_CRITICAL_ENTER();
	  OSSemSet(&GyroDRY_Sem,0,&err);
	   	   
		/*创建调试任务*/
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
							 
		/*创建姿态解算任务*/					
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
							 
//		 /*退出临界区*/
//		 OS_CRITICAL_EXIT();
//		
//    /*等姿态迭代好了之后再进行下一步*/							 
//		OS_TaskSuspend(NULL,&err);
//					
//	  /*进入临界区*/
//	  OS_CRITICAL_ENTER();
//	  OSSemSet(&GyroDRY_Sem,0,&err);
							 
		/*创建控制任务*/					
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
							 
		/*创建CAN发送任务*/					
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

		/*创建监控任务*/					
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
							 
		 /*退出临界区*/
		 OS_CRITICAL_EXIT();
							 
	   /*干掉自己*/
	   OSTaskDel(NULL , & err);
}

/*系统各项功能初始化*/
void SysModule_Init(void)
{
	OS_ERR  err;
	  
	  /*CPU配置初始化*/
	  CPU_Init();   
	
    /*初始化SysTick定时器*/	
    OS_CPU_SysTickInit(BSP_CPU_ClkFreq()
	                     /(CPU_INT32U)OSCfg_TickRate_Hz);   
	
	  /*内存管理初始化*/
//	  Mem_Init();     
	
#if OS_CFG_STAT_TASK_EN > 0u
	  /*开启统计任务*/
    OSStatTaskCPUUsageInit(&err);                       
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();
#endif
	
#if OS_CFG_SCHED_ROUND_ROBIN_EN
    /*开启时间片调度*/
		OSSchedRoundRobinCfg((CPU_BOOLEAN )DEF_ENABLED,
												 (OS_TICK     )0,
												 (OS_ERR     *)&err);
#endif	
	
}
