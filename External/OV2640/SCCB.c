#include "SCCB.h"
void SCCB_Delay(uint8_t us)
{
	for(;us>0;us--)
	{
		volatile uint8_t consttime=10;
		while(consttime--);
	}
}
//��ʼ��SCCB�ӿ� 
void SCCB_Init(void)
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	HAL_GPIO_WritePin(GPIOB, SCCB_SDA|SCCB_SCL, GPIO_PIN_RESET);
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.Pin = SCCB_SDA|SCCB_SCL;
	GPIO_InitStructure.Mode=GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull=GPIO_PULLUP;
	GPIO_InitStructure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	SCCB_SDA_OUT;
}
//SCCB��ʼ�ź�
//��ʱ��Ϊ�ߵ�ʱ��,�����ߵĸߵ���,ΪSCCB��ʼ�ź�
//�ڼ���״̬��,SDA��SCL��Ϊ�͵�ƽ
void SCCB_Start(void)
{
    SCCB_SDA_H;     //�����߸ߵ�ƽ	   
    SCCB_SCL_H;	    //��ʱ���߸ߵ�ʱ���������ɸ�����
    SCCB_Delay(50);  
    SCCB_SDA_L;
    SCCB_Delay(50);	 
    SCCB_SCL_L;	    //�����߻ָ��͵�ƽ��������������Ҫ	  
}

//SCCBֹͣ�ź�
//��ʱ��Ϊ�ߵ�ʱ��,�����ߵĵ͵���,ΪSCCBֹͣ�ź�
//����״����,SDA,SCL��Ϊ�ߵ�ƽ
void SCCB_Stop(void)
{
    SCCB_SDA_L;
    SCCB_Delay(50);	 
    SCCB_SCL_H;	
    SCCB_Delay(50); 
    SCCB_SDA_H;	
    SCCB_Delay(50);
}  
//����NA�ź�
void SCCB_No_Ack(void)
{
	SCCB_Delay(50);
	SCCB_SDA_H;	
	SCCB_SCL_H;	
	SCCB_Delay(50);
	SCCB_SCL_L;	
	SCCB_Delay(50);
	SCCB_SDA_L;	
	SCCB_Delay(50);
}
//SCCB,д��һ���ֽ�
//����ֵ:0,�ɹ�;1,ʧ��. 
uint8_t SCCB_WR_Byte(uint8_t dat)
{
	uint8_t j,res=0;	 
	for(j=0;j<8;j++) //ѭ��8�η�������
	{
		if(dat&0x80)SCCB_SDA_H;	
		else SCCB_SDA_L;
		dat<<=1;
		SCCB_Delay(50);
		SCCB_SCL_H;	
		SCCB_Delay(50);
		SCCB_SCL_L;		   
	}
	SCCB_SDA_IN;//����SDAΪ���� 
	SCCB_Delay(50);
	SCCB_SCL_H;//���յھ�λ,���ж��Ƿ��ͳɹ�
	SCCB_Delay(50);
	res=SCCB_READ_SDA;//SDA_H����ʧ�ܣ�����1 SDA_L���ͳɹ�������0
	SCCB_SCL_L;
	SCCB_SDA_OUT;//����SDAΪ���    
	return res;
}	 
//SCCB ��ȡһ���ֽ�
//��SCL��������,��������
//����ֵ:����������
uint8_t SCCB_RD_Byte(void)
{
	uint8_t temp=0,j;    
	SCCB_SDA_IN;		//����SDAΪ����  
	for(j=8;j>0;j--) 	//ѭ��8�ν�������
	{		     	  
		SCCB_Delay(50);
		SCCB_SCL_H;
		temp=temp<<1;
		if(SCCB_READ_SDA)temp++;   
		SCCB_Delay(50);
		SCCB_SCL_L;
	}	
	SCCB_SDA_OUT;		//����SDAΪ���    
	return temp;
} 							    
//д�Ĵ���
//����ֵ:0,�ɹ�;1,ʧ��.
uint8_t SCCB_WR_Reg(uint8_t reg,uint8_t data)
{
	uint8_t res=0;
	SCCB_Start(); 					//����SCCB����
	if(SCCB_WR_Byte(SCCB_ID))res=1;	//д����ID	  
	SCCB_Delay(100);
	if(SCCB_WR_Byte(reg))res=1;		//д�Ĵ�����ַ	  
	SCCB_Delay(100);
	if(SCCB_WR_Byte(data))res=1; 	//д����	 
	SCCB_Stop();	  
	return	res;
}		  					    
//���Ĵ���
//����ֵ:�����ļĴ���ֵ
uint8_t SCCB_RD_Reg(uint8_t reg)
{
	uint8_t val=0;
	SCCB_Start(); 				//����SCCB����
	SCCB_WR_Byte(SCCB_ID);		//д����ID	  
	SCCB_Delay(100);	 
	SCCB_WR_Byte(reg);			//д�Ĵ�����ַ	  
	SCCB_Delay(100);	  
	SCCB_Stop();   
	SCCB_Delay(100);	   
	//���üĴ�����ַ�󣬲��Ƕ�
	SCCB_Start();
	SCCB_WR_Byte(SCCB_ID|0X01);	//���Ͷ�����	  
	SCCB_Delay(100);
	val=SCCB_RD_Byte();		 	//��ȡ����
	SCCB_No_Ack();
	SCCB_Stop();
	return val;
}
