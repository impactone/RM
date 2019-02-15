#include "includes.h"
#include "Task_SysInit.h"

#define  TASK_SYSINIT_PRIO        2u
#define  TASK_SYSINIT_STK_SIZE    128u

OS_TCB  Task_SysInitTCB;
CPU_STK  Task_SysInitStk[TASK_SYSINIT_STK_SIZE];

int main(void)
{
    OS_ERR  err;
	
    /*��ʼ��UCOSIII*/
    OSInit(&err);                                     
	
	  /*����ϵͳ��ʼ������*/
    OSTaskCreate((OS_TCB       *)&Task_SysInitTCB,         
                 (CPU_CHAR     *)"SysInit Task",
                 (OS_TASK_PTR   )Task_SysInit,
                 (void         *)0u,
                 (OS_PRIO       )TASK_SYSINIT_PRIO,
                 (CPU_STK      *)Task_SysInitStk,
                 (CPU_STK_SIZE  )TASK_SYSINIT_STK_SIZE / 10u,
                 (CPU_STK_SIZE  )TASK_SYSINIT_STK_SIZE,
                 (OS_MSG_QTY    )0u,
                 (OS_TICK       )0u,
                 (void         *)0u,
                 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR       *)&err);
								 
		/*��������*/
    OSStart(&err);                                        
		
		for (;;)
		{
			
		}
}

