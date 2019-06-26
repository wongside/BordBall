#include "MOTO.h" 
TIM_HandleTypeDef htim2;
uint16_t period=1000;
void Error_Handler()
{
	while(1);
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
	//设置PWM输出引脚
  GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	/**TIM2 GPIO Configuration    
	PA0-WKUP     ------> TIM2_CH1
	PA2     ------> TIM2_CH3 
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
	//启动时钟
	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 210;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = period;
	htim2.Init.RepetitionCounter=10;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim2);
}
void Moto_Init()
{
	MX_TIM2_Init();
	HAL_TIM_MspPostInit(&htim2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,200);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,200);
}
void Moto1_Set(float per)
{
	//93
	//20
	if(per>180)
		per=180;
	if(per<0)
		per=0;
	per=per*0.406f+20;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,period*(per/100));
}
void Moto2_Set(float per)
{
	if(per>180)
		per=180;
	if(per<0)
		per=0;
	per=per*0.406f+20;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,period*(per/100));
}