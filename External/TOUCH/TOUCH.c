#include "TOUCH.h"
SPI_HandleTypeDef hspi2;
uint8_t t_start=1;
uint8_t t_Yaddr=1;//0~7
uint8_t t_Xaddr=5;//0~7
uint8_t t_mode=0;//1 8位 0 12位
uint8_t t_SER_DFR=0;//1 单端 0差分
uint8_t t_PD1=0;//总是使能
uint8_t t_PD0=0;//总是使能
uint16_t GetADC(bool isx)
{
	uint8_t error=0;
	uint16_t buffer=0,value[4],xyerror=0;
	uint8_t rxdata=0,txdata=0;
	uint8_t time=4,t=time;
	
	txdata=t_start<<7;
	if(isx) txdata|=t_Xaddr<<4;
	else txdata|=t_Yaddr<<4;
	txdata|=t_mode<<3;
	txdata|=t_SER_DFR<<2;
	txdata|=t_PD1<<1;
	txdata|=t_PD0;
	while(t--)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
		if(HAL_SPI_Transmit(&hspi2,&txdata,1,300)!=HAL_OK)
			error++;
		if(HAL_SPI_Receive(&hspi2,&rxdata,1,300)!=HAL_OK)
			error++;
		buffer=rxdata<<8;
		if(HAL_SPI_Receive(&hspi2,&rxdata,1,300)!=HAL_OK)
			error++;
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
		buffer|=rxdata;
		buffer>>=3;
		value[t]=buffer;
		//HAL_Delay(10);
	}
	t=time;
	while((t--)>1)
	{
		xyerror+=abs(value[t]-value[t-1]);
	}
	return xyerror>60?0:(value[0]+value[1]+value[2]+value[3])/4;
}
bool GetTouch(uint16_t *x,uint16_t *y)
{
	uint16_t tx=GetADC(true);
	uint16_t ty=GetADC(false);
	*x=tx?tx:*x;
	*y=ty?ty:*y;
	return true;
}
bool Touch_init()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_SPI2_CLK_ENABLE();

	__HAL_RCC_GPIOB_CLK_ENABLE();
	/**SPI2 GPIO Configuration    
	PB12     ------> SPI2_NSS
	PB13     ------> SPI2_SCK
	PB14     ------> SPI2_MISO
	PB15     ------> SPI2_MOSI 
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
		return false;
	else
		return true;
}