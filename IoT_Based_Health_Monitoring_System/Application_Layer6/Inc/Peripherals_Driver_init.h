/*
 * Peripherals_Driver_init.h
 *
 *  Created on: 19-Jul-2021
 *      Author: vermas
 */




#ifndef PERIPHERALS_DRIVER_INIT_H_
#define PERIPHERALS_DRIVER_INIT_H_

//to Init the TIMER6 and TIMER 7

#define COMMON_HEADER_FILES

#ifdef COMMON_HEADER_FILES
#include"../../stm32f44xx_driver_Layer1/Inc/delay.h"
#endif
#undef COMMON_HEADER_FILES

//include the header to configure the error led and timeout led 
#include"../../Status_Error_Devices_Layer2/Inc/Error_Handling.h"

//to configure buzzer 
#include"../../Status_Error_Devices_Layer2/Inc/Buzzer_Config.h"

//to configure the external devices i.e Temperature sensor
#include"../../External_Peripherals_Layer3/Inc/MCP9808_Temp_Sensor.h"

//to configure the I2C LCD 
#include"../../External_Peripherals_Layer3/Inc/LCD_HD44780_I2C.h"




#endif /* PERIPHERALS_DRIVER_INIT_H_ */
