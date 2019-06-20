#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "TimeBase.h"
TIM_HandleTypeDef htim8;

/**
  * @brief  This function configures the TIM8 as a time base source. 
  *         The time source is configured  to have 1ms time base with a dedicated 
  *         Tick interrupt priority. 
  * @note   This function is called  automatically at the beginning of program after
  *         reset by HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig(). 
  * @param  TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClkInitTypeDef    clkconfig;
  uint32_t              uwTimclock = 0;
  uint32_t              uwPrescalerValue = 0;
  uint32_t              pFLatency;
  
  /*Configure the TIM8 IRQ priority */
  //HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, TickPriority ,0); 
  
  /* Enable the TIM8 global Interrupt */
  HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn); 
  
  /* Enable TIM8 clock */
  __HAL_RCC_TIM8_CLK_ENABLE();
  
  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
  
  /* Compute TIM8 clock */
  uwTimclock = 2*HAL_RCC_GetPCLK2Freq();
   
  /* Compute the prescaler value to have TIM8 counter clock equal to 1MHz */
  uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000) - 1);
  
  /* Initialize TIM8 */
  htim8.Instance = TIM8;
  
  /* Initialize TIMx peripheral as follow:
  + Period = [(TIM8CLK/1000) - 1]. to have a (1/1000) s time base.
  + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
  + ClockDivision = 0
  + Counter direction = Up
  */
  htim8.Init.Period = (1000000 / 1000) - 1;
  htim8.Init.Prescaler = uwPrescalerValue;
  htim8.Init.ClockDivision = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&htim8) == HAL_OK)
  {
    /* Start the TIM time Base generation in interrupt mode */
    return HAL_TIM_Base_Start_IT(&htim8);
  }
  
  /* Return function status */
  return HAL_ERROR;
}

/**
  * @brief  Suspend Tick increment.
  * @note   Disable the tick increment by disabling TIM8 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_SuspendTick(void)
{
  /* Disable TIM8 update Interrupt */
  __HAL_TIM_DISABLE_IT(&htim8, TIM_IT_UPDATE);                                                  
}

/**
  * @brief  Resume Tick increment.
  * @note   Enable the tick increment by Enabling TIM8 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_ResumeTick(void)
{
  /* Enable TIM8 Update interrupt */
  __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);
}
///**
//  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
//  */
//void TIM8_UP_TIM13_IRQHandler(void)
//{
//  HAL_TIM_IRQHandler(&htim8);
//}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM8) {
    HAL_IncTick();
  }
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
