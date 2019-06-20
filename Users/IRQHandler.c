#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "TimeBase.h"
#include "OV2640.h"
/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim8);
}
void DCMI_IRQHandler(void)
{
	HAL_DCMI_FrameEventCallback(&hdcmi);
}