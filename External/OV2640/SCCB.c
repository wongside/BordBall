#include "SCCB.h"
void SCCB_Delay(uint8_t us)
{
	for(;us>0;us--)
	{
		volatile uint8_t consttime=10;
		while(consttime--);
	}
}
//初始化SCCB接口 
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
//SCCB起始信号
//当时钟为高的时候,数据线的高到低,为SCCB起始信号
//在激活状态下,SDA和SCL均为低电平
void SCCB_Start(void)
{
    SCCB_SDA_H;     //数据线高电平	   
    SCCB_SCL_H;	    //在时钟线高的时候数据线由高至低
    SCCB_Delay(50);  
    SCCB_SDA_L;
    SCCB_Delay(50);	 
    SCCB_SCL_L;	    //数据线恢复低电平，单操作函数必要	  
}

//SCCB停止信号
//当时钟为高的时候,数据线的低到高,为SCCB停止信号
//空闲状况下,SDA,SCL均为高电平
void SCCB_Stop(void)
{
    SCCB_SDA_L;
    SCCB_Delay(50);	 
    SCCB_SCL_H;	
    SCCB_Delay(50); 
    SCCB_SDA_H;	
    SCCB_Delay(50);
}  
//产生NA信号
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
//SCCB,写入一个字节
//返回值:0,成功;1,失败. 
uint8_t SCCB_WR_Byte(uint8_t dat)
{
	uint8_t j,res=0;	 
	for(j=0;j<8;j++) //循环8次发送数据
	{
		if(dat&0x80)SCCB_SDA_H;	
		else SCCB_SDA_L;
		dat<<=1;
		SCCB_Delay(50);
		SCCB_SCL_H;	
		SCCB_Delay(50);
		SCCB_SCL_L;		   
	}
	SCCB_SDA_IN;//设置SDA为输入 
	SCCB_Delay(50);
	SCCB_SCL_H;//接收第九位,以判断是否发送成功
	SCCB_Delay(50);
	res=SCCB_READ_SDA;//SDA_H发送失败，返回1 SDA_L发送成功，返回0
	SCCB_SCL_L;
	SCCB_SDA_OUT;//设置SDA为输出    
	return res;
}	 
//SCCB 读取一个字节
//在SCL的上升沿,数据锁存
//返回值:读到的数据
uint8_t SCCB_RD_Byte(void)
{
	uint8_t temp=0,j;    
	SCCB_SDA_IN;		//设置SDA为输入  
	for(j=8;j>0;j--) 	//循环8次接收数据
	{		     	  
		SCCB_Delay(50);
		SCCB_SCL_H;
		temp=temp<<1;
		if(SCCB_READ_SDA)temp++;   
		SCCB_Delay(50);
		SCCB_SCL_L;
	}	
	SCCB_SDA_OUT;		//设置SDA为输出    
	return temp;
} 							    
//写寄存器
//返回值:0,成功;1,失败.
uint8_t SCCB_WR_Reg(uint8_t reg,uint8_t data)
{
	uint8_t res=0;
	SCCB_Start(); 					//启动SCCB传输
	if(SCCB_WR_Byte(SCCB_ID))res=1;	//写器件ID	  
	SCCB_Delay(100);
	if(SCCB_WR_Byte(reg))res=1;		//写寄存器地址	  
	SCCB_Delay(100);
	if(SCCB_WR_Byte(data))res=1; 	//写数据	 
	SCCB_Stop();	  
	return	res;
}		  					    
//读寄存器
//返回值:读到的寄存器值
uint8_t SCCB_RD_Reg(uint8_t reg)
{
	uint8_t val=0;
	SCCB_Start(); 				//启动SCCB传输
	SCCB_WR_Byte(SCCB_ID);		//写器件ID	  
	SCCB_Delay(100);	 
	SCCB_WR_Byte(reg);			//写寄存器地址	  
	SCCB_Delay(100);	  
	SCCB_Stop();   
	SCCB_Delay(100);	   
	//设置寄存器地址后，才是读
	SCCB_Start();
	SCCB_WR_Byte(SCCB_ID|0X01);	//发送读命令	  
	SCCB_Delay(100);
	val=SCCB_RD_Byte();		 	//读取数据
	SCCB_No_Ack();
	SCCB_Stop();
	return val;
}
