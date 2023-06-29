/*
 * led_blink_state_machine.c
 *
 *  Created on: Jun 28, 2023
 *      Author: shobhit
 */


/* program to implement state machine for led blink program.


 */
#include "led_blink_state_machine.h"


unsigned char led_blink_rate = -1;

BLINKING_LED_STATEMACHINE_EVENTS_t btn_State = -1;
static BLINKING_LED_STATEMACHINE_STATES_t currentState =  IDLE;


void BlinkingLED_StateMachine(void)
{

	//when button is pressed for first time then goes into below condition
	if (currentState == IDLE)
	{
		blink_rate_twice_state_handler();
	}

	//when button is released for second time
	else if (currentState == BLINK_RATE_TWICE)
	{
		blink_rate_no_change_state_handler();
	}

	//when button is again pressed
	else if (currentState == BLINK_RATE_NO_CHANGE)
	{
		blink_rate_single_state_handler();
	}

	else if (currentState == BLINK_RATE_SINGLE)
	{
		idle_state_handler();
	}


}





void BlinkingLED_StateMachine_Init(void)
{
	idle_state_handler();
	//led_blink_rate = LED_BLINK_RATE_ONCE_PER_SEC;

	//LED_OFF(LED_STATUS_BUTTON_PRESS_PORT, LED_STATUS_BUTTON_PRESS_PIN_NO);
}







/* handler functions for different states */
void blink_rate_twice_state_handler(void)
{
	currentState = BLINK_RATE_TWICE;
	led_blink_rate = LED_BLINK_RATE_TWICE_PER_SEC;
	btn_State = BUTTON_PRESSED;
	LED_ON(LED_STATUS_BUTTON_PRESS_PORT, LED_STATUS_BUTTON_PRESS_PIN_NO);



}

void blink_rate_no_change_state_handler(void)
{
	currentState = BLINK_RATE_NO_CHANGE;
	led_blink_rate = LED_BLINK_RATE_TWICE_PER_SEC;
	LED_OFF(LED_STATUS_BUTTON_PRESS_PORT, LED_STATUS_BUTTON_PRESS_PIN_NO);

	btn_State =BUTTON_RELEASED;

}


void blink_rate_single_state_handler(void)
{
	currentState = BLINK_RATE_SINGLE;
	led_blink_rate = LED_BLINK_RATE_ONCE_PER_SEC;
	LED_ON(LED_STATUS_BUTTON_PRESS_PORT, LED_STATUS_BUTTON_PRESS_PIN_NO);

	btn_State =BUTTON_PRESSED;

}

void idle_state_handler(void)
{
	currentState = IDLE;
	led_blink_rate = LED_BLINK_RATE_ONCE_PER_SEC;

	btn_State = BUTTON_RELEASED;
	LED_OFF(LED_STATUS_BUTTON_PRESS_PORT, LED_STATUS_BUTTON_PRESS_PIN_NO);

}




void LED_ON(GPIO_TypeDef* GPIO_PORT, uint16_t GPIO_PIN)
{
	HAL_GPIO_WritePin( (GPIO_TypeDef*)GPIO_PORT, (uint16_t )GPIO_PIN, GPIO_PIN_SET);
}
void LED_OFF(GPIO_TypeDef* GPIO_PORT, uint16_t GPIO_PIN)
{
	HAL_GPIO_WritePin((GPIO_TypeDef*)GPIO_PORT, (uint16_t )GPIO_PIN, GPIO_PIN_RESET);
}


