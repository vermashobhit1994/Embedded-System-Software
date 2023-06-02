/*
 * Peripherals_Driver_Init.c
 *
 *  Created on: 17-Jul-2021
 *      Author: vermas
 */

/* Description : File that initialized all peripherals used in project.
 *
 *
 * */

#include"Peripherals_Driver_init.h"






void Peripherals_Init(void)
{

	
	//Check_Endianess();
	TIM6_Init();

	//declare in Error_Handling.h
        //to init the status devices
	GPIO_Error_LED_Init();
	GPIO_Timeout_LED_Init();
        BUZZER_GPIO_Init();

	//Init the external peripherals
	//Initialize the Temperature sensor
	MCP9808_Param_Init();

	//Initialize the 16x2 LCD
	I2C_LCD_Init();



}
