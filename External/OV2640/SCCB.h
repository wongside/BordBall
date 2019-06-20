#ifndef __SCCB_H
#define __SCCB_H
#include "stm32f4xx_hal.h"
#include "stdint.h"


//GPIOB 
#define SCCB_SDA GPIO_PIN_11
#define SCCB_SCL GPIO_PIN_10

//IO方向设置
#define SCCB_SDA_IN  GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=0<<11*2	//PB11 输入
#define SCCB_SDA_OUT GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=1<<11*2 	//PB11 输出

//IO操作函数
#define SCCB_READ_SDA GPIOB->IDR&SCCB_SDA
#define SCCB_SDA_H HAL_GPIO_WritePin(GPIOB,SCCB_SDA,GPIO_PIN_SET)
#define SCCB_SDA_L HAL_GPIO_WritePin(GPIOB,SCCB_SDA,GPIO_PIN_RESET)
#define SCCB_SCL_H HAL_GPIO_WritePin(GPIOB,SCCB_SCL,GPIO_PIN_SET)
#define SCCB_SCL_L HAL_GPIO_WritePin(GPIOB,SCCB_SCL,GPIO_PIN_RESET)

#define SCCB_ID   			0X60  			//OV2640的ID

///////////////////////////////////////////
void SCCB_Init(void);
void SCCB_Start(void);
void SCCB_Stop(void);
void SCCB_No_Ack(void);
uint8_t SCCB_WR_Byte(uint8_t dat);
uint8_t SCCB_RD_Byte(void);
uint8_t SCCB_WR_Reg(uint8_t reg,uint8_t data);
uint8_t SCCB_RD_Reg(uint8_t reg);
#endif