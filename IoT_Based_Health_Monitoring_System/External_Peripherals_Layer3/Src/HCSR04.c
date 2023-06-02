/*
 * HCSR04.c
 *
 *  Created on: 12-Feb-2021
 *      Author: vermas
 */

#include"HCSR04.h"

double HCSR04_Sensor_Read()
{
	uint32_t temperature = DEFAULT_TEMP_VALUE;
	(void)temperature;

	//Initialize the TRIG Pin to low
	GPIO_WriteToOutputPin(HCSR04_PORT_TRIGGER, HCSR04_PIN_TRIGGER, 0);
	delay_us(2);

	//Start sending the pulse on trigger pin
	GPIO_WriteToOutputPin(HCSR04_PORT_TRIGGER, HCSR04_PIN_TRIGGER, 1);
	delay_us(10);
	GPIO_WriteToOutputPin(HCSR04_PORT_TRIGGER, HCSR04_PIN_TRIGGER, 0);

	//wait until the ECHO Pin becomes HIGH
	//If ECHO Pin doesn't becomes High it means timeout occurs.
	uint32_t timeout = HCSR04_TIMEOUT;
	while(!GPIO_ReadFromInputPin(HCSR04_PORT_ECHO, HCSR04_PIN_ECHO))
	{
		if(timeout-- == 0x00)
			return -1;

	}

	//Measure the time by checking the ECHO Pin until it remains HIGH
	uint32_t timeinMicroseconds =0;


	//wait until the pin goes low i.e the pulse has been retrieved back
	while(GPIO_ReadFromInputPin(HCSR04_PORT_ECHO, HCSR04_PIN_ECHO))
	{
		timeinMicroseconds++;
		delay_us(1);
	}


	//Convert time into distance
	//Calculate the speed of sound in Centimeter Per microseconds
	//where it depends on temperature.
	//float temperature = DEFAULT_TEMP_VALUE;//default value of temperature
	//double SpeedSoundInCmPerMicroseconds = (((double)331.3*100)/1000000) + (((double)0.606 * 100)/1000000)*temperature;
	double SpeedSoundInCmPerMicroseconds = 29.412;//(340 * 100)/1000000;
	//divided by 2 so as to get the half journey time.
	double DistanceInCm = ((double)timeinMicroseconds/(2.0 * SpeedSoundInCmPerMicroseconds ) );

	//double DistanceInCm = (float)timeinMicroseconds * HCSR04_CONST;

	//If the calculated distance isn't in range then return -1.0
	if(DistanceInCm <= 2.0  && DistanceInCm > 400.000)
	{
		return -1;
	}

	else
		return DistanceInCm;
}


uint8_t HCSR04_Init()
{


	//Initialize the GPIO Port and Pins for TRIGGER and ECHO
	GPIO_Handle_t HCSR04Config = {0};
	(void)HCSR04Config;

	//Initialize the TRIG Pin
	HCSR04Config.pGPIOx = HCSR04_PORT_TRIGGER;
	HCSR04Config.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	HCSR04Config.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PUSH_PULL;

	//Why selected as Pull down?
	HCSR04Config.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_PULL_DOWN;
	HCSR04Config.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	HCSR04Config.GPIO_PinConfig.GPIO_PinNumber = HCSR04_PIN_TRIGGER;
	GPIO_Init(&HCSR04Config);


	//Initialize the ECHO Pin as Input Pin
	GPIO_Handle_t HCSR04Configecho = {0};
	HCSR04Configecho.pGPIOx = HCSR04_PORT_ECHO;
	HCSR04Configecho.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	HCSR04Configecho.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PUSH_PULL;
	//Why selected as Pull down?
	HCSR04Configecho.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_PULL_DOWN;
	HCSR04Configecho.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	HCSR04Configecho.GPIO_PinConfig.GPIO_PinNumber = HCSR04_PIN_ECHO;
	GPIO_Init(&HCSR04Configecho);

	//Setting the TRIG to low by default
	GPIO_WriteToOutputPin(HCSR04_PORT_TRIGGER, HCSR04_PIN_TRIGGER, 0);


	//Read the dummy data to check if sensor is working
	if(HCSR04_Sensor_Read() >= 0)
		return 1;
	else
		return 0;


}


