/*
 * DHT11_Sensor.h
 *
 *  Created on: 04-Jul-2021
 *      Author: vermas
 */

#ifndef DHT11_TEMPERATURE_HUMIDITY_SENSOR_INC_DHT11_SENSOR_H_
#define DHT11_TEMPERATURE_HUMIDITY_SENSOR_INC_DHT11_SENSOR_H_

#include"stm32f446xx.h"

#include<stdbool.h>//for bool
#include"Error_Handling.h"//for error indication led
#include"delay.h"

//structure to hold the DHT11 data
typedef struct
{
	uint8_t Temperature;
	uint8_t Humidity;
}DHT11_Sensor_Data;

/* Default Pins for DHT11 sensor . Can be changed in user application */
#ifndef __DHT11_GPIO_PIN_INIT__
#define __DHT11_GPIO_PIN_INIT__

//must use below macro to override
#define DHT11_GPIO_PORT          GPIOA
#define DHT11_GPIO_PIN_NO        GPIO_PIN_NO_1

#endif

/*function prototypes */
void DHT11_Sensor_Init(void);
void DHT11_Sensor_SetPinOutput(void);
void DHT11_Sensor_SetPinInput(void);
uint8_t DHT11_Sensor_GetData(DHT11_Sensor_Data *pDHTData);

#endif /* DHT11_TEMPERATURE_HUMIDITY_SENSOR_INC_DHT11_SENSOR_H_ */
