#include "stm32f4xx_hal.h"
#include "stm32f407xx.h"
#include "FreeRTOS.h"
#include "Task.h"
#include "event_groups.h"
#include "semphr.h" 

#include "ImageProcess.h"
#include "MOTO.h"
#include "OV2640.h"
#include "LCD.h"
#include "TOUCH.h"
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
void Task_MotoTest()
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
void PIDUpdate(point targetpoint,Center p)
{
	static float AngleX = 90, AngleY = 90;
	static float SpeedX = 0,SpeedY=0;
	static unsigned char str[50];
	
//	LCD_Scan_Dir(R2L_D2U);
//	LCD_Set_Window(0,0,240,320);
//	POINT_COLOR=0x0000;
	
//	sprintf((char *)str, "x:%0.1f y:%0.1f", p.x, p.y);
//	LCD_ShowString(0, 0, 120, 50, 16, str);

	SpeedX = PID_Inc(targetpoint.x, p.x, &SpeedPIDX);
	SpeedY = PID_Inc(targetpoint.y, p.y, &SpeedPIDY);
	
//	sprintf((char *)str, "TS_X: %0.1fP:%0.1fI:%0.1fD:%0.1f", SpeedX,SpeedPIDX.Ek,SpeedPIDX.Ek1,SpeedPIDX.Ek2);
//	LCD_ShowString(0, 16, 300, 50, 16, str);
//	sprintf((char *)str, "TS_Y: %0.1fP:%0.1fI:%0.1fD:%0.1f", SpeedY,SpeedPIDY.Ek,SpeedPIDY.Ek1,SpeedPIDY.Ek2);
//	LCD_ShowString(0, 32, 300, 50, 16, str);
	
	AngleX =70+ PID_Inc(SpeedX, p.speedx, &AngelPIDX);
	AngleY =59+ PID_Inc(SpeedY, p.speedy, &AngelPIDY);
	
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
	static float LastAngleX=0;
	static float LastAngleY=0;
	AngleX=LastAngleX*0.6f+AngleX*0.4f;
	AngleY=LastAngleY*0.6f+AngleY*0.4f;
	LastAngleX=AngleX;
	LastAngleY=AngleY;
	Moto1_Set(AngleX);
	Moto2_Set(AngleY);
	
	static uint8_t drawx=0;
//	LCD_Fast_DrawPoint(drawx,AngleX/180*32,0xffff);
//	LCD_Fast_DrawPoint(drawx,AngleY/180*32+32,0xffff);
//	drawx++;
//	if(drawx==240)
//	{
//		drawx=0;
//		LCD_Fill(0,0,239,64,0);
//	}
	
//	sprintf((char *)str, "xcal: %6.2f", AngleX);
//	LCD_ShowString(0, 48, 100, 50, 16, str);
//	sprintf((char *)str, "ycal: %6.2f", AngleY);
//	LCD_ShowString(0, 64, 100, 50, 16, str);	
//	LCD_Scan_Dir(L2R_U2D);
}
#define CX 115
#define CY 120
static Center p;
point TargetPoint={CX,CY};
point Regions[9];
void Regions_init()
{
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			Regions[i*3+j].x=50+j*66;
			Regions[i*3+j].y=50+i*66;
		}
	}
}
bool isBollInCircular(point tarpoint,float rad)
{
	float absx=fabs(p.x-tarpoint.x);
	float absy=fabs(p.y-tarpoint.y);
	float r=__sqrtf(absx*absx+absy*absy);
	if(r<rad) return true;
	else return false;
}
void Task_Mission_2(bool *FLAG_MissionRunning)
{
	TargetPoint=Regions[1-1];
	while(!isBollInCircular(Regions[1-1],20));
	vTaskDelay(3000);
	TargetPoint=Regions[5-1];
	while(!isBollInCircular(Regions[5-1],20));
	vTaskDelay(3000);
	FLAG_MissionRunning=false;
	vTaskDelete(NULL);
}
void Task_Mission_3(bool *FLAG_MissionRunning)
{
	TargetPoint=Regions[1-1];
	while(!isBollInCircular(Regions[1-1],20));
	vTaskDelay(3000);
	TargetPoint=Regions[4-1];
	while(!isBollInCircular(Regions[4-1],20));
	vTaskDelay(3000);
	TargetPoint=Regions[5-1];
	while(!isBollInCircular(Regions[5-1],20));
	vTaskDelay(3000);
	FLAG_MissionRunning=false;
	vTaskDelete(NULL);
}
void Task_Mission_4(bool *FLAG_MissionRunning)
{
	TargetPoint=Regions[1-1];
	while(!isBollInCircular(Regions[1-1],20));
	vTaskDelay(3000);
	TargetPoint=Regions[9-1];
	while(!isBollInCircular(Regions[9-1],20));
	vTaskDelay(3000);
	FLAG_MissionRunning=false;
	vTaskDelete(NULL);
}
void Task_Mission_5(bool *FLAG_MissionRunning)
{
	TargetPoint=Regions[1-1];
	while(!isBollInCircular(Regions[1-1],20));
	vTaskDelay(3000);
	TargetPoint=Regions[2-1];
	while(!isBollInCircular(Regions[2-1],20));
	vTaskDelay(3000);
	TargetPoint=Regions[6-1];
	while(!isBollInCircular(Regions[6-1],20));
	vTaskDelay(3000);
	FLAG_MissionRunning=false;
	vTaskDelete(NULL);
}
void get_circular(uint8_t* outx,uint8_t* outy,uint8_t x,uint8_t y,uint8_t r,float rad)
{
	float K=sinf(rad);
	*outy=K*r+y;
	K=cosf(rad);
	*outx=K*r+x;
}
void Task_Mission_7(bool *FLAG_MissionRunning)
{
	TargetPoint=Regions[4-1];
	while(!isBollInCircular(Regions[4-1],20));
	vTaskDelay(3000);
	
	uint8_t count=0;
	float rad=0;
	while(count<4)
	{
		get_circular(&TargetPoint.x,&TargetPoint.y,CX,CY,60,rad);
		rad+=0.002f;
		vTaskDelay(1);
		if(rad>6.28f)
		{
			rad=0;
			count++;
		}
	}
	TargetPoint=Regions[9-1];
	while(!isBollInCircular(Regions[9-1],20));
	vTaskDelay(3000);
	FLAG_MissionRunning=false;
	vTaskDelete(NULL);
}
#define GRAY_BACKGROUND
#define VOTE_POINT
#define TRACT
#define SPEED
#define TARGET_POSTION
#define CONFIDENCE
void Draw()
{
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
		LCD_Draw_Circle(TargetPoint.x,TargetPoint.y,13);
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
}
bool FLAG_NeedDraw=false;
void OV2640_FrameReady()
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
	//转换为灰度图
	RGB_to_gray();//4.4ms
	//寻找圆心
	p=find_circle(4,2,3);//14ms  4  13
	//计算PID并控制电机
	PIDUpdate(TargetPoint,p);
	FLAG_NeedDraw=true;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
}
const static Rectangle Mission_1rec={0,0,120,40};
const static Rectangle Mission_2rec={121,0,239,40};
const static Rectangle Mission_3rec={0,41,120,80};
const static Rectangle Mission_4rec={121,41,239,80};
const static Rectangle Mission_5rec={0,81,120,120};
const static Rectangle Mission_6rec={121,81,239,120};
const static Rectangle Mission_7rec={0,121,120,160};
const static Rectangle Mission_8rec={121,121,239,160};
void Task_ChangePosition(void)
{
	uint16_t tx,ty;
	xTaskHandle MissionHandle;
	bool FLAG_MissionRunning=false;
	uint16_t count=0;
	char str[40];
	Regions_init();
	while(1)
	{
		LCD_Scan_Dir(R2L_D2U);
		POINT_COLOR=0;
		if(!GetTouch(&tx,&ty))//如果没有得到 count++
		{
			if(count<21)
				count++;
			if(count==20)
			{
				if(IsPointInRec(Mission_1rec,tx,ty))
				{
					strcpy(str,"mission:1");
					LCD_ShowString(120, 64, 300, 50, 16, (uint8_t *)str);
					if(FLAG_MissionRunning) vTaskDelete(MissionHandle);
					FLAG_MissionRunning=false;
					TargetPoint=Regions[1];
				}
				if(IsPointInRec(Mission_2rec,tx,ty))
				{
					LCD_ShowString(120, 64, 300, 50, 16, "mission:2");
					if(FLAG_MissionRunning) vTaskDelete(MissionHandle);
					FLAG_MissionRunning=true;
					xTaskCreate((TaskFunction_t)Task_Mission_2,"Mission_2",200,(void*)&FLAG_MissionRunning,1,&MissionHandle);
				}
				if(IsPointInRec(Mission_3rec,tx,ty))
				{
					LCD_ShowString(120, 64, 300, 50, 16, "mission:3");
					if(FLAG_MissionRunning) vTaskDelete(MissionHandle);
					FLAG_MissionRunning=true;
					xTaskCreate((TaskFunction_t)Task_Mission_3,"Mission_3",200,(void*)&FLAG_MissionRunning,1,&MissionHandle);
				}
				if(IsPointInRec(Mission_4rec,tx,ty))
				{
					LCD_ShowString(120, 64, 300, 50, 16, "mission:4");
					if(FLAG_MissionRunning) vTaskDelete(MissionHandle);
					FLAG_MissionRunning=true;
					xTaskCreate((TaskFunction_t)Task_Mission_4,"Mission_4",200,(void*)&FLAG_MissionRunning,1,&MissionHandle);
				}
				if(IsPointInRec(Mission_5rec,tx,ty))
				{
					LCD_ShowString(120, 64, 300, 50, 16, "mission:5");
					if(FLAG_MissionRunning) vTaskDelete(MissionHandle);
					FLAG_MissionRunning=true;
					xTaskCreate((TaskFunction_t)Task_Mission_5,"Mission_5",200,(void*)&FLAG_MissionRunning,1,&MissionHandle);
				}
				if(IsPointInRec(Mission_7rec,tx,ty))
				{
					LCD_ShowString(120, 64, 300, 50, 16, "mission:7");
					if(FLAG_MissionRunning) vTaskDelete(MissionHandle);
					FLAG_MissionRunning=true;
					xTaskCreate((TaskFunction_t)Task_Mission_7,"Mission_7",200,(void*)&FLAG_MissionRunning,1,&MissionHandle);
				}
			}
		}
		else //得到了
		{
//x 400~3750 0~3350
//y 3800~250 3550
			if(tx<400) tx=400;
			if(ty<250) ty=250;
			tx=(tx-400)/14;
			ty=340-(ty-250)/11.1f;
			sprintf(str,"x:%d,y:%d     ",tx,ty);
			LCD_ShowString(0, 64, 300, 50, 16, (uint8_t *)str);
			count=0;
		}
		LCD_Scan_Dir(L2R_U2D);
		if(FLAG_NeedDraw)
		{
			FLAG_NeedDraw=false;
			Draw();
		}
		vTaskDelay(1);
	}
}
int main ()
{
	uint16_t mai;
	HAL_Init();
	SystemClock_Config();
	LCD_Init();
	Touch_init();
	
//	uint16_t tx,ty;
//	LCD_Display_Dir(0);
//	LCD_Scan_Dir(R2L_D2U);
//	while(1)
//	{
//		char str[20];
//		if(GetTouch(&tx,&ty))
//		{
//			sprintf((char *)str, "tx:%d     ", tx);
//			LCD_ShowString(0, 64, 200, 50, 16, (uint8_t*)str);
//			sprintf((char *)str, "ty:%d     ",ty);
//			LCD_ShowString(0, 80, 200, 50, 16, (uint8_t*)str);
//		}
//		else
//		{
//			tx=0;
//			ty=0;
//			sprintf((char *)str, "tx:%d     ", tx);
//			LCD_ShowString(0, 64, 200, 50, 16, (uint8_t*)str);
//			sprintf((char *)str, "ty:%d     ",ty);
//			LCD_ShowString(0, 80, 200, 50, 16, (uint8_t*)str);
//		}
//	}
//	while(1);
	
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
	
	PID_Init(&SpeedPIDX);
	PID_Init(&SpeedPIDY);
	PID_Set_Value(&SpeedPIDX, 0.065, 0.01, 0.36);
	PID_Set_Value(&SpeedPIDY, 0.065, 0.01, 0.36);
	PID_Init(&AngelPIDX);
	PID_Init(&AngelPIDY);
	PID_Set_Value(&AngelPIDX, 4, 0, 2.7);
	PID_Set_Value(&AngelPIDY, 4, 0, 2.7);
	Moto1_Set(70);
	Moto2_Set(75);
	
	//xTaskCreate((TaskFunction_t)Task_FrameReady,"keyscan",300,NULL,2,NULL);
	xTaskCreate((TaskFunction_t)Task_ChangePosition,"keyscan",400,NULL,1,NULL);
	vTaskStartScheduler();
}