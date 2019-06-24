#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "Task.h"
#include "event_groups.h"
#include "semphr.h" 

#include "ImageProcess.h"
#include "MOTO.h"
#include "OV2640.h"
#include "LCD.h"
#include "pid.h"


xSemaphoreHandle xSemaphorezz;

PID_IncTypeDef pid1, pid2;

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // Configure the main internal regulator output voltage 
  
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  //Initializes the CPU, AHB and APB busses clocks 
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    while(1);
  }
  // Initializes the CPU, AHB and APB busses clocks 
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    while(1);
  }
  // Enables the Clock Security System 
  HAL_RCC_EnableCSS();
}
void IRQHandleTask()
{

	
	
	while(1){
		xSemaphoreTake(xSemaphorezz, portMAX_DELAY);

		OV2640_FrameReady();

	}
	
}

void Task()
{
	float A=0,B=0;
	bool AF=false,BF=false;
	while(1)
	{
		Moto1_Set(A);
		Moto2_Set(B);
		if(AF)
			A+=0.1f;
		else
			A-=0.2f;
		if(BF)
			B+=0.2f;
		else
			B-=0.3f;
		if(A<0)
			AF=true;
		if(A>100)
			AF=false;
		if(B<0)
			BF=true;
		if(B>100)
			BF=false;
		vTaskDelay(3);
	}

}
#define GRAY_BACKGROUND
//#define VOTE_POINT
#define TRACT
#define CONFIDENCE
void OV2640_FrameReady(void)
{
	float v1, v2;
	
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
	//转换为灰度图
	RGB_to_gray();//4.4ms

	//寻找圆心
	Center p=find_circle();//14ms
	//画摄像头背景
//	unsigned char * str = {"ddd"};
//	LCD_ShowxNum(200,300, p.x, 3, 12, 0);
//	LCD_ShowxNum(200,250, p.x, 3, 12, 0);
//	LCD_ShowString(200, 200, 50, 50, 12, str);
	v1 = PID_Inc(232/2, p.x, &pid1);
//	Moto2_Set(v1);
//	v2 = PID_Inc(0, p.y, &pid1);
	
#ifdef GRAY_BACKGROUND
	LCD_Set_Window(0,0,IMAGE_W,IMAGE_W);
	//LCD_SetCursor(0,0);
	LCD_WriteRAM_Prepare();		//开始写入GRAM
	for(uint16_t y=0;y<IMAGE_W;y++)//7ms
		for(uint16_t x=0;x<IMAGE_W;x++)
			LCD->LCD_RAM=Gray8toGary16(GrayImage[y][x]);
#else
	LCD_Fill(0,0,232,232,0x0000);//5.6ms
#endif
#ifdef TRACT
	//画圆心跟踪标记
	POINT_COLOR=0x7e0;
	LCD_DrawLine(0,p.y,IMAGE_W,p.y);
	LCD_DrawLine(p.x,0,p.x,IMAGE_W);
#endif
#ifdef VOTE_POINT
	//画投票点
	for(uint16_t i=0;i<p.CentersNumber;i++)
	{
		uint16_t Color=p.Centers[i].confidence<<13;
		LCD_Fast_DrawPoint(p.Centers[i].x,p.Centers[i].y,Color);
	}
#endif
#ifdef CONFIDENCE
	//画信心条
	uint16_t Conf=p.confidence!=0?p.confidence>20?IMAGE_W:(float)p.confidence/20*IMAGE_W:0;
	if(Conf<IMAGE_W-1)
		LCD_Fill(IMAGE_W,Conf,239,IMAGE_W-1,0x0000);
	if(Conf>2)
		LCD_Fill(IMAGE_W,0,239,Conf-1,0xffff);
#endif
	//LCD_Fast_DrawPoint(p.x,p.y,0x7e0);
	/*
	LCD_SetCursor(4,0);
	LCD_WriteRAM_Prepare();		//开始写入GRAM
	for(uint16_t y=0;y<IMAGE_W;y++)
		for(uint16_t x=0;x<IMAGE_W/8;x++)
		{
			uint8_t imagepart=TowValueImage[y][x];
			for(int indexx=7;indexx>=0;indexx--)
			{
				LCD->LCD_RAM=(imagepart&(1<<indexx))?0xffff:0;
			}
		}
*/
	/*
	for(uint16_t y=0;y<IMAGE_W;y++)//7ms
		for(uint16_t x=0;x<IMAGE_W;x++)
			LCD->LCD_RAM=Gray8toGary16(GrayImage[y][x]);
			*/
	//OV2640_OutStart((uint32_t)&LCD->LCD_RAM, 1);//启动传输
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
}

int main ()
{
	uint16_t mai;
	HAL_Init();
	SystemClock_Config();
	
	LCD_Init();
	OV2640_Init();
	Moto_Init();
	
	LCD_Display_Dir(0);
	LCD_Scan_Dir(L2R_U2D);
	OV2640_LED(0);
	OV2640_RGB565_Mode();	//RGB565模式
	//OV2640_Window_Set(0,0,800,600);
	OV2640_ImageSize_Set(800,600);
	OV2640_ImageWin_Set(200,0,600,600);
	OV2640_OutSize_Set(IMAGE_W,IMAGE_W);
	//LCD_Set_Window(0,0,IMAGE_W,IMAGE_W);

	//OV2640_DMAOutput(DMA_MDATAALIGN_WORD, DMA_MINC_DISABLE); //DCMI DMA配置
	//OV2640_OutStart((uint32_t)&LCD->LCD_RAM, 1); 		//启动传输
	OV2640_DMAOutput(DMA_MDATAALIGN_WORD, DMA_MINC_ENABLE); //DCMI DMA配置
	OV2640_OutStart((uint32_t)RGBYUVImage,26912); 		//启动传输
	//OV2640_LED(1);
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//	vSemaphoreCreateBinary(xSemaphorezz);
	xSemaphorezz = xSemaphoreCreateCounting(10, 0);
	
	PID_Init(&pid1);
	PID_Init(&pid2);
	PID_Set_Value(&pid1, 10, 0, 0);
	Moto1_Set(45);
	if( xSemaphorezz != NULL ){

		xTaskCreate((TaskFunction_t)IRQHandleTask,"keyscan",300,NULL,3,NULL);
//		xTaskCreate((TaskFunction_t)Task,"keyscan",300,NULL,1,NULL);
		vTaskStartScheduler();		
	
	}
	while(1);

}