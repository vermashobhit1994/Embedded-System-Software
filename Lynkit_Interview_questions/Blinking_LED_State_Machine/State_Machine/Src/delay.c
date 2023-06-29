/*
 * delay.c
 *
 *  Created on: Jun 28, 2023
 *      Author: shobhit
 */

#ifndef DELAY_C_
#define DELAY_C_

#include "delay.h"

uint8_t timer_state_running = -1;

void delay_ms_polling(uint16_t time)
{
	//remove old value from SR register
	WRITE_REG(htim6.Instance -> SR, 0);

	//initial value of counter i.e. count from value - 65535
	WRITE_REG(htim6.Instance -> CNT, htim6.Instance->ARR-time-1);

	//start timer
	HAL_TIM_Base_Start(&htim6);


	//wait until update event occured.
	while( READ_REG(htim6.Instance -> SR) == 0);


	//reset update event.
	WRITE_REG(htim6.Instance -> SR, 0);


	//stop timer
	HAL_TIM_Base_Stop(&htim6);

/*
	if (time == 1000)
		for(int i=0; i<500000;i++);
	else if (time == 500)
		for(int i=0; i<500000/2;i++);
*/
}


void delay_ms_interrupt(uint16_t time)
{
	//remove old value from SR register
		WRITE_REG(htim6.Instance -> SR, 0);

	//initial value of counter i.e. count from value - 65535
		WRITE_REG(htim6.Instance -> CNT, htim6.Instance->ARR-time-1);

		//start timer
		HAL_TIM_Base_Start(&htim6);
		timer_state_running = 1;

		while(timer_state_running == 1);
}


#endif /* DELAY_C_ */
