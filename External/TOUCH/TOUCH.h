#ifndef __TOUCH_H_
#define __TOUCH_H_
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "math.h"
bool Touch_init();
bool GetTouch(uint16_t *x,uint16_t *y);
#endif