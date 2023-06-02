/*
 * main.h
 *
 *  Created on: 19-Jul-2021
 *      Author: vermas
 */

#ifndef MAIN_H_
#define MAIN_H_


#include"MCP9808_Temp_Sensor.h"


#include"LCD_HD44780_I2C.h"
/* Peripherals used in project */

/*
typedef struct
{
	MCP9808_Param_Config_t *pTemperature_SensorParam;

	I2C_Handle_t *pTemperature_Sensor_I2CHandle;

	I2C_Handle_t *pI2C_LCD_Handle;

}Peripherals_Struct_Init;
*/




void Peripherals_Init(void);



#include"FPU_Config.h"


#include<stdio.h>



void Check_Endianess(void);


//extern Peripherals_Struct_Init PeripheralsInit;
extern I2C_Handle_t Temperature_Sensor_I2CHandle;//I2C parameter for temperature sensor
extern MCP9808_Param_Config_t Temperature_SensorParam;//parameter for temperature sensor
extern I2C_Handle_t I2C_LCD_Handle;


/* prototye for I2C LCD */
//Functionalities related functions
void LCD_I2C_HD44780_PrintString(char *str);





#endif /* MAIN_H_ */
