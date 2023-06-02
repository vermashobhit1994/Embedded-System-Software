/*
 * HCSR04.h
 *
 *  Created on: 12-Feb-2021
 *      Author: vermas
 */

/*********************** Documentation ***************
 * Range - 2cm to 400cm or 1”to 13 feet with 3mm interval of distance.
 * Trigger I/P Pulse Width = 10 us.
 * Operating frequency - 40KHz
 * ECHO Pin Pulse width = distance measured.
 */

#ifndef HCSR04_ULTRASONICE_SENSOR_INC_HCSR04_H_
#define HCSR04_ULTRASONICE_SENSOR_INC_HCSR04_H_

#include"stm32f446xx.h"
#include"stm32f44xx_gpio_driver.h"

#include"delay.h"
#include<string.h>//for memset()

/* Default Pins and Port for ECHO and TRIG */
#ifndef __HCSR04_CONFIG__
#define __HCSR04_CONFIG__
#define HCSR04_PORT_ECHO          GPIOC
#define HCSR04_PORT_TRIGGER       GPIOC
#define HCSR04_PIN_TRIGGER        GPIO_PIN_NO_2
#define HCSR04_PIN_ECHO           GPIO_PIN_NO_3
#endif

#ifndef __HCSR04_TIMEOUT__
#define __HCSR04_TIMEOUT__
#define HCSR04_TIMEOUT          1000000//Timeout = 1 us
#endif

//Default temperature value (in *c)
#ifndef DEFAULT_TEMP_VALUE
#define DEFAULT_TEMP_VALUE      25
#endif


#define HCSR04_CONST            ((float)0.0171821)

double HCSR04_Sensor_Read();
uint8_t HCSR04_Init();

#endif /* HCSR04_ULTRASONICE_SENSOR_INC_HCSR04_H_ */
