#include "BSP_NVIC.h"


/**
  * @brief  NVIC��ʼ��
  * @param  void
  * @retval void
  */
void BSP_NVIC_InitConfig(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}

