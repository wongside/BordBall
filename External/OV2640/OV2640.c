#include "stm32f4xx_hal.h"
#include "OV2640.h"
#include "SCCB.h"	
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;
void HAL_DCMI_MspInit(DCMI_HandleTypeDef* hdcmi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hdcmi->Instance==DCMI)
  {
    /* Peripheral clock enable */
    __HAL_RCC_DCMI_CLK_ENABLE();
  
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**DCMI GPIO Configuration    
    PE4     ------> DCMI_D4
    PE5     ------> DCMI_D6
    PE6     ------> DCMI_D7
    PA4     ------> DCMI_HSYNC
    PA6     ------> DCMI_PIXCK
    PC6     ------> DCMI_D0
    PC7     ------> DCMI_D1
    PB6     ------> DCMI_D5
    PB7     ------> DCMI_VSYNC
    PE0     ------> DCMI_D2
    PE1     ------> DCMI_D3 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_0 
                          |GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }

}
void OV2640_DMAOutput(uint32_t DMA_MemoryDataSize,uint32_t DMA_MemoryInc)
{
	hdma_dcmi.Instance = DMA2_Stream1;
	hdma_dcmi.Init.Channel = DMA_CHANNEL_1;
	hdma_dcmi.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_dcmi.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_dcmi.Init.MemInc = DMA_MemoryInc;
	hdma_dcmi.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	hdma_dcmi.Init.MemDataAlignment = DMA_MemoryDataSize;
	hdma_dcmi.Init.Mode = DMA_CIRCULAR;
	hdma_dcmi.Init.Priority = DMA_PRIORITY_VERY_HIGH;
	hdma_dcmi.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
	hdma_dcmi.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	hdma_dcmi.Init.MemBurst = DMA_MBURST_SINGLE;
	hdma_dcmi.Init.PeriphBurst = DMA_PBURST_SINGLE;
	if (HAL_DMA_Init(&hdma_dcmi) != HAL_OK)
	{
		while(1);
	}
	__HAL_LINKDMA(&hdcmi,DMA_Handle,hdma_dcmi);
}
static void MX_DCMI_Init(void)
{
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_LOW;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
		while(1);
  }
	__HAL_DCMI_DISABLE_IT(&hdcmi, DCMI_IT_LINE);
	__HAL_DCMI_DISABLE_IT(&hdcmi, DCMI_IT_ERR);
	__HAL_DCMI_DISABLE_IT(&hdcmi, DCMI_IT_VSYNC);
}
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  //HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
}
//DCMI,��������
void OV2640_OutStart(uint32_t DMA_Memory0BaseAddr,uint16_t DMA_BufferSize)
{
	__HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_FRAME);
	HAL_NVIC_SetPriority(DCMI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DCMI_IRQn);//
	HAL_DCMI_Start_DMA(&hdcmi,DCMI_MODE_CONTINUOUS,DMA_Memory0BaseAddr,DMA_BufferSize);
	//HAL_DCMI_Start_DMA(&hdcmi,DCMI_MODE_SNAPSHOT,DMA_Memory0BaseAddr,DMA_BufferSize);
}
void OV2640_OutSuspend(void)
{
	HAL_DCMI_Suspend(&hdcmi);
}
void OV2640_OutResume(void)
{
	HAL_DCMI_Resume(&hdcmi);
}
//DCMI,�رմ���
void OV2640_OutStop(void)
{
	HAL_DCMI_Stop(&hdcmi); //DCMI����ʹ�ر�	
}
__weak void OV2640_FrameReady(void)
{
	
}
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	//OV2640_OutSuspend();
	__HAL_DCMI_ENABLE_IT(hdcmi, DCMI_IT_FRAME);
	DCMI->ICR=0x1;
	OV2640_FrameReady();
	
	//OV2640_OutResume();
}
//��ʼ��OV2640 
//�������Ժ�,Ĭ�������1600*1200�ߴ��ͼƬ!! 
//����ֵ:0,�ɹ�
//    ����,�������
uint8_t OV2640_Init(void)
{ 
	uint16_t i=0;
	uint16_t reg;
	//����IO
	GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DCMI_PWDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DCMI_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DCMI_LED_Pin, GPIO_PIN_RESET);
	/*Configure GPIO pins : DCMI_XCLK_Pin */
  GPIO_InitStruct.Pin = DCMI_XCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  /*Configure GPIO pins : DCMI_PWDN_Pin */
  GPIO_InitStruct.Pin = DCMI_PWDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_RESET_Pin */
  GPIO_InitStruct.Pin = DCMI_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_LED_Pin */
  GPIO_InitStruct.Pin = DCMI_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

 	OV2640_POWER_ON;	//POWER ON
	HAL_Delay(10);
	OV2640_RESET_L;	//��λOV2640
	HAL_Delay(10);
	OV2640_RESET_H;	//������λ 
  SCCB_Init();//��ʼ��SCCB ��IO��	 
	SCCB_WR_Reg(OV2640_DSP_RA_DLMT, 0x01);	//����sensor�Ĵ���
 	SCCB_WR_Reg(OV2640_SENSOR_COM7, 0x80);	//��λOV2640
	HAL_Delay(50); 
	reg=SCCB_RD_Reg(OV2640_SENSOR_MIDH);	//��ȡ����ID �߰�λ
	reg<<=8;
	reg|=SCCB_RD_Reg(OV2640_SENSOR_MIDL);	//��ȡ����ID �Ͱ�λ
	if(reg!=OV2640_MID)
	{
		//printf("MID:%d\r\n",reg);
		return 1;
	}
	reg=SCCB_RD_Reg(OV2640_SENSOR_PIDH);	//��ȡ����ID �߰�λ
	reg<<=8;
	reg|=SCCB_RD_Reg(OV2640_SENSOR_PIDL);	//��ȡ����ID �Ͱ�λ

 	//��ʼ�� OV2640 
	for(i=0;i<sizeof(ov2640_svga_init_reg_tbl)/2;i++)
	{
	   	SCCB_WR_Reg(ov2640_svga_init_reg_tbl[i][0],ov2640_svga_init_reg_tbl[i][1]);
		  if(i<10)HAL_Delay(5);//д���ʼ�������ݵ�ʱ�򣬼���ʱ���������ò�����
 	}
	MX_DCMI_Init();
	MX_DMA_Init();
	HAL_DCMI_MspInit(&hdcmi);
	return 0;//ok
}
void OV2640_LED(uint8_t onoff)
{
	if(onoff) OV2640_LED_ON;//��������LED
	else OV2640_LED_OFF;
}
//OV2640�л�ΪJPEGģʽ
void OV2640_JPEG_Mode(void) 
{
	uint16_t i=0;
	//����:YUV422��ʽ
	for(i=0;i<(sizeof(ov2640_yuv422_reg_tbl)/2);i++)
	{
		SCCB_WR_Reg(ov2640_yuv422_reg_tbl[i][0],ov2640_yuv422_reg_tbl[i][1]); 
	} 
	
	//����:���JPEG����
	for(i=0;i<(sizeof(ov2640_jpeg_reg_tbl)/2);i++)
	{
		SCCB_WR_Reg(ov2640_jpeg_reg_tbl[i][0],ov2640_jpeg_reg_tbl[i][1]);  
	}  
}
//OV2640�л�ΪRGB565ģʽ
void OV2640_RGB565_Mode(void) 
{
	uint16_t i=0;
	//����:RGB565���
	for(i=0;i<(sizeof(ov2640_rgb565_reg_tbl)/2);i++)
	{
		SCCB_WR_Reg(ov2640_rgb565_reg_tbl[i][0],ov2640_rgb565_reg_tbl[i][1]); 
	}
} 
//�Զ��ع����ò�����,֧��5���ȼ�
const static uint8_t OV2640_AUTOEXPOSURE_LEVEL[5][8]=
{
	{
		0xFF,0x01,
		0x24,0x20,
		0x25,0x18,
		0x26,0x60,
	},
	{
		0xFF,0x01,
		0x24,0x34,
		0x25,0x1c,
		0x26,0x00,
	},
	{
		0xFF,0x01,	
		0x24,0x3e,	
		0x25,0x38,
		0x26,0x81,
	},
	{
		0xFF,0x01,
		0x24,0x48,
		0x25,0x40,
		0x26,0x81,
	},
	{
		0xFF,0x01,	
		0x24,0x58,	
		0x25,0x50,	
		0x26,0x92,	
	},
}; 
//OV2640�Զ��ع�ȼ�����
//level:0~4
void OV2640_Auto_Exposure(uint8_t level)
{  
	uint8_t i;
	uint8_t *p=(uint8_t*)OV2640_AUTOEXPOSURE_LEVEL[level];
	for(i=0;i<4;i++)
	{ 
		SCCB_WR_Reg(p[i*2],p[i*2+1]); 
	} 
}  
//��ƽ������
//0:�Զ�
//1:̫��sunny
//2,����cloudy
//3,�칫��office
//4,����home
void OV2640_Light_Mode(uint8_t mode)
{
	uint8_t regccval=0X5E;//Sunny 
	uint8_t regcdval=0X41;
	uint8_t regceval=0X54;
	switch(mode)
	{ 
		case 0://auto 
			SCCB_WR_Reg(0XFF,0X00);	 
			SCCB_WR_Reg(0XC7,0X00);//AWB ON 
			return;  	
		case 2://cloudy
			regccval=0X65;
			regcdval=0X41;
			regceval=0X4F;
			break;	
		case 3://office
			regccval=0X52;
			regcdval=0X41;
			regceval=0X66;
			break;	
		case 4://home
			regccval=0X42;
			regcdval=0X3F;
			regceval=0X71;
			break;	
	}
	SCCB_WR_Reg(0XFF,0X00);	 
	SCCB_WR_Reg(0XC7,0X40);	//AWB OFF 
	SCCB_WR_Reg(0XCC,regccval); 
	SCCB_WR_Reg(0XCD,regcdval); 
	SCCB_WR_Reg(0XCE,regceval);  
}
//ɫ������
//0:-2
//1:-1
//2,0
//3,+1
//4,+2
void OV2640_Color_Saturation(uint8_t sat)
{ 
	uint8_t reg7dval=((sat+2)<<4)|0X08;
	SCCB_WR_Reg(0XFF,0X00);		
	SCCB_WR_Reg(0X7C,0X00);		
	SCCB_WR_Reg(0X7D,0X02);				
	SCCB_WR_Reg(0X7C,0X03);			
	SCCB_WR_Reg(0X7D,reg7dval);			
	SCCB_WR_Reg(0X7D,reg7dval); 		
}
//��������
//0:(0X00)-2
//1:(0X10)-1
//2,(0X20) 0
//3,(0X30)+1
//4,(0X40)+2
void OV2640_Brightness(uint8_t bright)
{
  SCCB_WR_Reg(0xff, 0x00);
  SCCB_WR_Reg(0x7c, 0x00);
  SCCB_WR_Reg(0x7d, 0x04);
  SCCB_WR_Reg(0x7c, 0x09);
  SCCB_WR_Reg(0x7d, bright<<4); 
  SCCB_WR_Reg(0x7d, 0x00); 
}
//�Աȶ�����
//0:-2
//1:-1
//2,0
//3,+1
//4,+2
void OV2640_Contrast(uint8_t contrast)
{
	uint8_t reg7d0val=0X20;//Ĭ��Ϊ��ͨģʽ
	uint8_t reg7d1val=0X20;
  	switch(contrast)
	{
		case 0://-2
			reg7d0val=0X18;	 	 
			reg7d1val=0X34;	 	 
			break;	
		case 1://-1
			reg7d0val=0X1C;	 	 
			reg7d1val=0X2A;	 	 
			break;	
		case 3://1
			reg7d0val=0X24;	 	 
			reg7d1val=0X16;	 	 
			break;	
		case 4://2
			reg7d0val=0X28;	 	 
			reg7d1val=0X0C;	 	 
			break;	
	}
	SCCB_WR_Reg(0xff,0x00);
	SCCB_WR_Reg(0x7c,0x00);
	SCCB_WR_Reg(0x7d,0x04);
	SCCB_WR_Reg(0x7c,0x07);
	SCCB_WR_Reg(0x7d,0x20);
	SCCB_WR_Reg(0x7d,reg7d0val);
	SCCB_WR_Reg(0x7d,reg7d1val);
	SCCB_WR_Reg(0x7d,0x06);
}
//��Ч����
//0:��ͨģʽ    
//1,��Ƭ
//2,�ڰ�   
//3,ƫ��ɫ
//4,ƫ��ɫ
//5,ƫ��ɫ
//6,����	    
void OV2640_Special_Effects(uint8_t eft)
{
	uint8_t reg7d0val=0X00;//Ĭ��Ϊ��ͨģʽ
	uint8_t reg7d1val=0X80;
	uint8_t reg7d2val=0X80; 
	switch(eft)
	{
		case 1://��Ƭ
			reg7d0val=0X40; 
			break;	
		case 2://�ڰ�
			reg7d0val=0X18; 
			break;	 
		case 3://ƫ��ɫ
			reg7d0val=0X18; 
			reg7d1val=0X40;
			reg7d2val=0XC0; 
			break;	
		case 4://ƫ��ɫ
			reg7d0val=0X18; 
			reg7d1val=0X40;
			reg7d2val=0X40; 
			break;	
		case 5://ƫ��ɫ
			reg7d0val=0X18; 
			reg7d1val=0XA0;
			reg7d2val=0X40; 
			break;	
		case 6://����
			reg7d0val=0X18; 
			reg7d1val=0X40;
			reg7d2val=0XA6; 
			break;	 
	}
	SCCB_WR_Reg(0xff,0x00);
	SCCB_WR_Reg(0x7c,0x00);
	SCCB_WR_Reg(0x7d,reg7d0val);
	SCCB_WR_Reg(0x7c,0x05);
	SCCB_WR_Reg(0x7d,reg7d1val);
	SCCB_WR_Reg(0x7d,reg7d2val); 
}
//��������
//sw:0,�رղ���
//   1,��������(ע��OV2640�Ĳ����ǵ�����ͼ�������)
void OV2640_Color_Bar(uint8_t sw)
{
	uint8_t reg;
	SCCB_WR_Reg(0XFF,0X01);
	reg=SCCB_RD_Reg(0X12);
	reg&=~(1<<1);
	if(sw)reg|=1<<1; 
	SCCB_WR_Reg(0X12,reg);
}
//����ͼ��������� 
//sx,sy,��ʼ��ַ
//width,height:���(��Ӧ:horizontal)�͸߶�(��Ӧ:vertical)
void OV2640_Window_Set(uint16_t sx,uint16_t sy,uint16_t width,uint16_t height)
{
	uint16_t endx;
	uint16_t endy;
	uint8_t temp; 
	endx=sx+width/2;	//V*2
 	endy=sy+height/2;
	
	SCCB_WR_Reg(0XFF,0X01);			
	temp=SCCB_RD_Reg(0X03);				//��ȡVref֮ǰ��ֵ
	temp&=0XF0;
	temp|=((endy&0X03)<<2)|(sy&0X03);
	SCCB_WR_Reg(0X03,temp);				//����Vref��start��end�����2λ
	SCCB_WR_Reg(0X19,sy>>2);			//����Vref��start��8λ
	SCCB_WR_Reg(0X1A,endy>>2);			//����Vref��end�ĸ�8λ
	
	temp=SCCB_RD_Reg(0X32);				//��ȡHref֮ǰ��ֵ
	temp&=0XC0;
	temp|=((endx&0X07)<<3)|(sx&0X07);
	SCCB_WR_Reg(0X32,temp);				//����Href��start��end�����3λ
	SCCB_WR_Reg(0X17,sx>>3);			//����Href��start��8λ
	SCCB_WR_Reg(0X18,endx>>3);			//����Href��end�ĸ�8λ
}
//����ͼ�������С

//OV2640���ͼ��Ĵ�С(�ֱ���),��ȫ�ɸĺ���ȷ��
//width,height:���(��Ӧ:horizontal)�͸߶�(��Ӧ:vertical),width��height������4�ı���
//����ֵ:0,���óɹ�
//    ����,����ʧ��
uint8_t OV2640_OutSize_Set(uint16_t width,uint16_t height)
{
	uint16_t outh;
	uint16_t outw;
	uint8_t temp; 
	
	if(width%4)return 1;
	if(height%4)return 2;
	
	outw=width/4;
	outh=height/4; 
	
	SCCB_WR_Reg(0XFF,0X00);	
	SCCB_WR_Reg(0XE0,0X04);			
	SCCB_WR_Reg(0X5A,outw&0XFF);		//����OUTW�ĵͰ�λ
	SCCB_WR_Reg(0X5B,outh&0XFF);		//����OUTH�ĵͰ�λ
	temp=(outw>>8)&0X03;
	temp|=(outh>>6)&0X04;
	SCCB_WR_Reg(0X5C,temp);				//����OUTH/OUTW�ĸ�λ 
	SCCB_WR_Reg(0XE0,0X00);	
	return 0;
}
//����ͼ�񿪴���С
//��:OV2640_ImageSize_Setȷ������������ֱ��ʴӴ�С.
//�ú������������Χ������п���,����OV2640_OutSize_Set�����
//ע��:�������Ŀ�Ⱥ͸߶�,������ڵ���OV2640_OutSize_Set�����Ŀ�Ⱥ͸߶�
//     OV2640_OutSize_Set���õĿ�Ⱥ͸߶�,���ݱ��������õĿ�Ⱥ͸߶�,��DSP
//     �Զ��������ű���,������ⲿ�豸.
//width,height:���(��Ӧ:horizontal)�͸߶�(��Ӧ:vertical),width��height������4�ı���
//����ֵ:0,���óɹ�
//    ����,����ʧ��
uint8_t OV2640_ImageWin_Set(uint16_t offx,uint16_t offy,uint16_t width,uint16_t height)
{
	uint16_t hsize;
	uint16_t vsize;
	uint8_t temp; 
	if(width%4)return 1;
	if(height%4)return 2;
	hsize=width/4;
	vsize=height/4;
	SCCB_WR_Reg(0XFF,0X00);	
	SCCB_WR_Reg(0XE0,0X04);					
	SCCB_WR_Reg(0X51,hsize&0XFF);		//����H_SIZE�ĵͰ�λ
	SCCB_WR_Reg(0X52,vsize&0XFF);		//����V_SIZE�ĵͰ�λ
	SCCB_WR_Reg(0X53,offx&0XFF);		//����offx�ĵͰ�λ
	SCCB_WR_Reg(0X54,offy&0XFF);		//����offy�ĵͰ�λ
	temp=(vsize>>1)&0X80;
	temp|=(offy>>4)&0X70;
	temp|=(hsize>>5)&0X08;
	temp|=(offx>>8)&0X07; 
	SCCB_WR_Reg(0X55,temp);				//����H_SIZE/V_SIZE/OFFX,OFFY�ĸ�λ
	SCCB_WR_Reg(0X57,(hsize>>2)&0X80);	//����H_SIZE/V_SIZE/OFFX,OFFY�ĸ�λ
	SCCB_WR_Reg(0XE0,0X00);	
	return 0;
} 
//�ú�������ͼ��ߴ��С,Ҳ������ѡ��ʽ������ֱ���
//UXGA:1600*1200,SVGA:800*600,CIF:352*288
//width,height:ͼ���Ⱥ�ͼ��߶�
//����ֵ:0,���óɹ�
//    ����,����ʧ��
uint8_t OV2640_ImageSize_Set(uint16_t width,uint16_t height)
{ 
	uint8_t temp; 
	SCCB_WR_Reg(0XFF,0X00);			
	SCCB_WR_Reg(0XE0,0X04);			
	SCCB_WR_Reg(0XC0,(width)>>3&0XFF);		//����HSIZE��10:3λ
	SCCB_WR_Reg(0XC1,(height)>>3&0XFF);		//����VSIZE��10:3λ
	temp=(width&0X07)<<3;
	temp|=height&0X07;
	temp|=(width>>4)&0X80; 
	SCCB_WR_Reg(0X8C,temp);	
	SCCB_WR_Reg(0XE0,0X00);				 
	return 0;
}
