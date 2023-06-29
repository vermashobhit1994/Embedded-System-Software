/*
 * led_blink.c
 *
 *  Created on: Jun 28, 2023
 *      Author: shobhit
 */


#include "led_blink.h"

#define BLINK_ONCE_RATE  (1000)
#define BLINK_TWICE_RATE (200)

void led_blink(void )
{
	//blink LED at once per sec
	if (led_blink_rate == LED_BLINK_RATE_ONCE_PER_SEC)
	{
		LED_ON(LED_BLINK_PORT, LED_BLINK_PIN_NO);
        DELAY_MS(BLINK_ONCE_RATE);
        LED_OFF(LED_BLINK_PORT, LED_BLINK_PIN_NO);
		DELAY_MS(BLINK_ONCE_RATE);

	}
	else if (led_blink_rate == LED_BLINK_RATE_TWICE_PER_SEC) //blink led at twice per sec
	{
		LED_ON(LED_BLINK_PORT, LED_BLINK_PIN_NO);
        DELAY_MS(BLINK_TWICE_RATE);
        LED_OFF(LED_BLINK_PORT, LED_BLINK_PIN_NO);
		DELAY_MS(BLINK_TWICE_RATE);
	}

}



