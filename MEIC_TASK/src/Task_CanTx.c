#include "Task_CanTx.h"

extern u8 suc;


void Task_CanTx(void *p_arg)
{
	(void )p_arg;
	OS_ERR err;
	OS_MSG_SIZE size;
	CanTxMsg *TxMsg;

	for (;;)
	{
		TxMsg = (CanTxMsg *)OSQPend(&CanMsgQue,0,OS_OPT_PEND_BLOCKING,&size,NULL,&err);	
		suc = 0;
		CAN_Transmit(CAN2,TxMsg);
		while (suc == 0);
	}
}

