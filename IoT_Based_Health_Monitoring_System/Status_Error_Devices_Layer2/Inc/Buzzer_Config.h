/*
 * Buzzer_Config.h
 *
 *  Created on: 05-Jul-2021
 *      Author: vermas
 */

/* Description : This is used to turn OFF /ON the buzzer */

#ifndef BUZZER_CONFIG_H_
#define BUZZER_CONFIG_H_

/* mcu specific header file */
#include"../../stm32f44xx_driver_Layer1/Inc/stm32f446xx.h"

#include"../../stm32f44xx_driver_Layer1/Inc/stm32f44xx_gpio_driver.h"

#include<stdbool.h>//for bool

#include"../../stm32f44xx_driver_Layer1/Inc/delay.h"

/* macro for buzzer state */
#define BUZZER_STATE_OFF              0
#define BUZZER_STATE_ON               1

/* Default pins and port for buzzer */
#ifndef __BUZZER_GPIO_PIN_INIT__
#define __BUZZER_GPIO_PIN_INIT__

#define BUZZER_GPIO_PORT             GPIOA
#define BUZZER_GPIO_PIN_NO           GPIO_PIN_NO_4

#endif

/* Function prototypes */
void BUZZER_GPIO_Init(void);

void BUZZER_Control(bool BuzzerState, uint32_t BuzzerDurationTime_us);

#endif /* BUZZER_CONFIG_H_ */
