/*
 * delay.h
 *
 *  Created on: Jun 28, 2023
 *      Author: shobhit
 */

#ifndef DELAY_H_
#define DELAY_H_

#include "main.h"
extern TIM_HandleTypeDef htim6;
#include "stm32g0xx.h"

void delay_ms_polling(uint16_t time);

void delay_ms_interrupt(uint16_t time);


#endif /* DELAY_H_ */
