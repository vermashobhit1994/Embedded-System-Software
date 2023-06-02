/*
 * main.c
 *
 *  Created on: 17-July-2021
 *      Author: Shobhit Verma
 */

/* Description : This is the main application file for the whole project.
 *               All functions and peripherals are called from here.
 *
 */

#include"main.h"



int main()
{
	//set the SysClock as PLL_P
	SystemClockConfig();

	//enable the FPU
	//Enable_FPU();

	printf("Hello world\n");

	Check_Endianess();

	/*************** Peripherals struct variables ***********************/
	//structure that hold the structure which is to be initialized



	/***********************************************************************/

	//variables for temperature sensor
	/*
	PeripheralsInit.pTemperature_SensorParam = &MCP9808Params;
	PeripheralsInit.pTemperature_Sensor_I2CHandle = &MCP9808_I2C;
	PeripheralsInit.pI2C_LCD_Handle = &I2C_LCD_Handle;
	*/

	//Initialize the parameters used in project
	Peripherals_Init();


	//float TempValue = MCP9808_Read_TempValue_In_Celsius(&Temperature_SensorParam);


	while(1)
	{
		//float TempValue = MCP9808_Calculate_Avg_Temperature_Value(&Temperature_SensorParam, 10);
		float TempValue = MCP9808_Read_TempValue_In_Celsius(&Temperature_SensorParam);

		float farenheitvalue = MCP9808_Convert_Celsius_To_Farenheit(TempValue);


		char buffer[50] = {0};
			sprintf(buffer, "%.2f %.2f",TempValue, farenheitvalue);

			LCD_I2C_HD44780_PrintString(buffer);
			delay_ms(1000);

			LcdI2CDisplayClear() ;

	}
}

