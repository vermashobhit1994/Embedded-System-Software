/*
 * delay.h
 *
 *  Created on: 22-Jan-2021
 *      Author: vermas
 */

#ifndef INC_DELAY_H_
#define INC_DELAY_H_


#include"stm32f446xx.h"

#include<stdbool.h>//for bool

/* macro for timer interrupt enable or disable in interrupt handler*/
#define TIM7_INT_STATUS_ENABLE              1
#define TIM7_INT_STATUS_DISABLE             0


/**** Prototype of API's used ******/
void TIM6_Init(void);
void delay_us(uint16_t time_us);
void delay_ms(uint16_t time_ms);

/* Function pointer to store the address of delay_us and delay_ms function*/
//void(*funcptr_delay_ms)(uint16_t);
//void(*funcptr_delay_us)(uint16_t);

void mdelay_blocking(uint32_t delay);
void udelay_blocking(uint32_t delay);



/* delay implemented using interrupt */
void TIM7_Init_Interrupt(uint32_t TIM7_Priority);
void TIM7_IRQHandling(bool TIM7_IntStatus);


void delay_us_Interrupt(uint16_t ticks_us);
void delay_ms_Interrupt(uint16_t ticks_ms);


#endif /* INC_DELAY_H_ */
