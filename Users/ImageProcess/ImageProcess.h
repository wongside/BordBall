#ifndef __IMAGEPROCESS_H_
#define __IMAGEPROCESS_H_
#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "math.h"
#define IMAGE_W 232
#define VOTENUMBER 600
typedef struct point
{
	uint8_t x;
	uint8_t y;
	uint8_t confidence;
}point;
typedef struct Center
{
	float x;
	float y;
	float speedx;
	float speedy;
	uint8_t confidence;
	point* Centers;
	uint16_t CentersNumber;
}Center;
typedef struct Centerf
{
	uint8_t x;
	uint8_t y;
	float value;
}Centerf;
typedef struct Gradient
{
	uint16_t lenth;
	int X;
	int Y;
}Gradient;
extern uint16_t RGBYUVImage[IMAGE_W][IMAGE_W];
extern uint8_t TowValueImage[IMAGE_W][IMAGE_W/8];
extern uint8_t GrayImage[IMAGE_W][IMAGE_W];
uint16_t Gray8toGary16(uint8_t Gray8);
void RGB_to_gray(void);
void RGB_to_tow_value(void);
Center find_circle(uint8_t radius,uint8_t r_range,uint8_t Minconfidence);
#endif