#include "BSP_NVIC.h"


/**
  * @brief  NVIC≥ı ºªØ
  * @param  void
  * @retval void
  */
void BSP_NVIC_InitConfig(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}

