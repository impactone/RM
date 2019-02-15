#ifndef __TASK_SYSINIT_H
#define __TASK_SYSINIT_H

#include  "includes.h"
#include  "BSP_ALL.h"

extern OS_Q CanMsgQue;

void  Task_SysInit (void *p_arg);
void  SysModule_Init(void);

#endif
