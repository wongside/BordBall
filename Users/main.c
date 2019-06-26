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
static PID_IncTypeDef SpeedPIDX, SpeedPIDY,AngelPIDX,AngelPIDY;	

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
void PIDUpdate(uint8_t x,uint8_t y,Center p)
{
	static float AngleX = 90, AngleY = 90;
	static float SpeedX = 0,SpeedY=0;
	static unsigned char str[20];
	
	LCD_Scan_Dir(R2L_D2U);
	LCD_Set_Window(0,0,240,320);
	POINT_COLOR=0x0000;
	sprintf((char *)str, "x:%0.1f y:%0.1f", p.x, p.y);
	LCD_ShowString(0, 0, 100, 50, 16, str);

	SpeedX = PID_Inc(x, p.x, &SpeedPIDX);
	SpeedY = PID_Inc(y, p.y, &SpeedPIDY);
	
	sprintf((char *)str, "TSPEED_X: %0.5f", SpeedX);
	LCD_ShowString(0, 16, 300, 50, 16, str);
	sprintf((char *)str, "TSPEED_Y: %0.5f", SpeedY);
	LCD_ShowString(0, 32, 300, 50, 16, str);
	
	AngleX =70+ PID_Inc(SpeedX, p.speedx, &AngelPIDX);
	AngleY =60+ PID_Inc(SpeedY, p.speedy, &AngelPIDY);
	
	if(AngleX < 0){
		AngleX = 0;
	}
	if(AngleX > 180){
		AngleX = 180;
	}
	
	if(AngleY < 0){
		AngleY = 0;
	}
	if(AngleY > 180){
		AngleY = 180;
	}
	Moto1_Set(AngleX);
	Moto2_Set(AngleY);
	
	sprintf((char *)str, "xcal: %6.2f", AngleX);
	LCD_ShowString(0, 48, 100, 50, 16, str);
	sprintf((char *)str, "ycal: %6.2f", AngleY);
	LCD_ShowString(0, 64, 100, 50, 16, str);	
	LCD_Scan_Dir(L2R_U2D);
}
#define GRAY_BACKGROUND
#define VOTE_POINT
#define TRACT
#define SPEED
#define TARGET_POSTION
#define CONFIDENCE
#define CX 115
#define CY 120
void Task_FrameReady(void)
{
	static Center p;
	uint8_t px=CX,py=CY;
	while(1)
	{
		xSemaphoreTake(xSemaphorezz, portMAX_DELAY);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
		//转换为灰度图
		RGB_to_gray();//4.4ms

		//寻找圆心
		p=find_circle(13,3,5);//14ms
		//计算PID并控制电机
		PIDUpdate(px,py,p);
		//画摄像头背景
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
	#ifdef SPEED
		//画速度矢量
		POINT_COLOR=0xF800;
		int ENDX=p.x+p.speedx*10;
		int ENDY=p.y+p.speedy*10;
		if(ENDX>235) ENDX=235;
		else if(ENDX<5) ENDX=5;
		if(ENDY>235) ENDY=235;
		else if(ENDY<5) ENDY=5;
		LCD_DrawLine(p.x,p.y,ENDX,ENDY);
	#endif
	#ifdef TARGET_POSTION
		//画目标点
		POINT_COLOR=LCD_RGB_24_2_565(0x66,0xcc,0xff);
		LCD_Draw_Circle(px,py,13);
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
		POINT_COLOR=LCD_RGB_24_2_565(0xFF,0xFF,0x00);
		uint16_t Conf=p.confidence!=0?p.confidence>20?IMAGE_W:(float)p.confidence/20*IMAGE_W:0;
		if(Conf<IMAGE_W-1)
			LCD_Fill(IMAGE_W,Conf,239,IMAGE_W-1,0x0000);
		if(Conf>2)
			LCD_Fill(IMAGE_W,0,239,Conf-1,POINT_COLOR);
	#endif
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
	}
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
	
	PID_Init(&SpeedPIDX);
	PID_Init(&SpeedPIDY);
	PID_Set_Value(&SpeedPIDX, 0.012, -0.011, 0.035);
	PID_Set_Value(&SpeedPIDY, 0.012, -0.011, 0.035);
	PID_Init(&AngelPIDX);
	PID_Init(&AngelPIDY);
	PID_Set_Value(&AngelPIDX, 7.2, 0, 2.5);
	PID_Set_Value(&AngelPIDY, 7.2, 0, 2.5);
	Moto1_Set(70);
	Moto2_Set(60);
	if( xSemaphorezz != NULL )
	{
		xTaskCreate((TaskFunction_t)Task_FrameReady,"keyscan",300,NULL,3,NULL);
//		xTaskCreate((TaskFunction_t)Task,"keyscan",300,NULL,1,NULL);
		vTaskStartScheduler();		
	}
}