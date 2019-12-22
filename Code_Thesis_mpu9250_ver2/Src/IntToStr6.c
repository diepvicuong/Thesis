#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <math.h>

/* 	u : number in type int
		y : buffer in string
		i : position in buffer
*/

void IntToStr6(uint32_t u, uint8_t *y,int i)
{
	uint32_t a;
	a=u;
	y[i+4] = a % 10 +0x30;
	a= a/10;
	y[i+3] = a % 10 +0x30;
	a= a/10;
	y[i+2] = a % 10 +0x30;
	a= a/10;
	y[i+1] = a % 10 +0x30;
	a= a/10;
	y[i] = a +0x30;
}
