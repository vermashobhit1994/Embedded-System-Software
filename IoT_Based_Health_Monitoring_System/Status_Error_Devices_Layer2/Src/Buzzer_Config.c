/*
 * Buzzer_ON_OFF.c
 *
 *  Created on: 05-Jul-2021
 *      Author: vermas
 */


/* Description : To Turn ON/OFF Buzzer on gpio pin */

#include"Buzzer_Config.h"

/* *********************************Documentation Section **********************************************
 * @func                         : BUZZER_GPIO_Init
 * @brief                        :
 * @return                       : None
 * @Note                         : Must be done for one time.
 */
void BUZZER_GPIO_Init(void)
{
	GPIO_Handle_t gpiobuzzer;
	gpiobuzzer.pGPIOx = BUZZER_GPIO_PORT  ;
	gpiobuzzer.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpiobuzzer.GPIO_PinConfig.GPIO_PinNumber = BUZZER_GPIO_PIN_NO;
	gpiobuzzer.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PUSH_PULL ;
	gpiobuzzer.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_NO_PUPD ;
	gpiobuzzer.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	GPIO_Init(&gpiobuzzer);

}

/* *********************************Documentation Section **********************************************
 * @func                         : BUZZER_Control
 * @brief                        :
 * @param[in1]                   :
 * @return                       : None
 * @Note                         :
 */
void BUZZER_Control(bool BuzzerState, uint32_t BuzzerDurationTime_ms)
{
	if(BuzzerState == BUZZER_STATE_ON)
	{
		GPIO_WriteToOutputPin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN_NO, BUZZER_STATE_ON);
		delay_ms(BuzzerDurationTime_ms);
		//(*funcptr_delay_ms)(BuzzerDurationTime_ms);
		GPIO_WriteToOutputPin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN_NO, BUZZER_STATE_OFF);

	}
}

