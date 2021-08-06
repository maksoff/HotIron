/*
 * delay.h
 *
 *  Created on: Aug 6, 2021
 *      Author: makso
 */

#ifndef INC_DELAY_H_
#define INC_DELAY_H_

#include "main.h"

void delay_init(TIM_HandleTypeDef * htim_i);
void delay_us(uint16_t us);


#endif /* INC_DELAY_H_ */
