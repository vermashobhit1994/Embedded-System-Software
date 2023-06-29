/*
 * led_blink_state_machine.c
 *
 *  Created on: Jun 28, 2023
 *      Author: shobhit
 */

#ifndef LED_BLINK_STATE_MACHINE_C_
#define LED_BLINK_STATE_MACHINE_C_

#include "delay.h"
#include "stm32g0xx.h"
#include "main.h"

/********************* driver specific functionalities *****************************************/
#define LED_BLINK_PIN_NO                  (LED_BLINK_Pin)
#define LED_BLINK_PORT                    ( LED_BLINK_GPIO_Port)

#define LED_STATUS_BUTTON_PRESS_PIN_NO    (LED_STATUS_Pin)
#define LED_STATUS_BUTTON_PRESS_PORT      (LED_STATUS_GPIO_Port)




#define BUTTON_PIN_NO                    (USER_BUTTON_Pin)



#define DELAY_MS(TIME_MS)     delay_ms_polling(TIME_MS)

/**********************************************************************************************/


#define LED_BLINK_RATE_ONCE_PER_SEC (1)
#define LED_BLINK_RATE_TWICE_PER_SEC (2)





typedef enum
{
	IDLE,
	BLINK_RATE_TWICE,
	BLINK_RATE_NO_CHANGE,
	BLINK_RATE_SINGLE,
}BLINKING_LED_STATEMACHINE_STATES_t;




typedef enum
{
	BUTTON_PRESSED,
	BUTTON_RELEASED,
}BLINKING_LED_STATEMACHINE_EVENTS_t;







void BlinkingLED_StateMachine(void);
void BlinkingLED_StateMachine_Init(void);
void blink_rate_twice_state_handler(void);
void blink_rate_no_change_state_handler(void);
void blink_rate_single_state_handler(void);
void idle_state_handler(void);


extern unsigned char led_blink_rate;
extern BLINKING_LED_STATEMACHINE_EVENTS_t btn_State ;



void LED_ON(GPIO_TypeDef* GPIO_PORT, uint16_t GPIO_PIN);
void LED_OFF(GPIO_TypeDef* GPIO_PORT, uint16_t GPIO_PIN);





#endif /* LED_BLINK_STATE_MACHINE_C_ */
