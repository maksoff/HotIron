/*
 * delay.c
 *
 *  Created on: Aug 6, 2021
 *      Author: makso
 */

// Start Timer
#include "main.h"

TIM_HandleTypeDef * htim;

void delay_init(TIM_HandleTypeDef * htim_i)
{
	htim = htim_i;
	HAL_TIM_Base_Start(htim);
}

// delays for us count
void delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(htim,0);
	while ((uint16_t)__HAL_TIM_GET_COUNTER(htim) < us);
}
