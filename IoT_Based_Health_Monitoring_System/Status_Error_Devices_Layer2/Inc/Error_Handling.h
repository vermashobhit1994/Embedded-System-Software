/*
 * Error_Handling.h
 *
 *  Created on: 05-Jul-2021
 *      Author: vermas
 */

/* Description : Header file for handling Error in case code execution fails
 *               and any of the peripheral needed in system isn't connected.
 */
#ifndef ERROR_HANDLING_H_
#define ERROR_HANDLING_H_

//including MCU specific header file for function prototypes and macros
//related to GPIOx Peripheral
#include"../../stm32f44xx_driver_Layer1/Inc/stm32f446xx.h"

#include"../../stm32f44xx_driver_Layer1/Inc/stm32f44xx_gpio_driver.h"

//enum to indicate whether the error occured or not
typedef enum
{
	STATUS_ERROR,
	STATUS_OK,
	STATUS_TIMEOUT
}Error_Status;

/*Macro used to Turn ON or OFF Error status led*/
#define ERROR_STATUS_LED_ON          1
#define ERROR_STATUS_LED_OFF         0

//Default Configuration for Pin and Port for Error Handling
#ifndef __ERROR_GPIO_PIN_INIT__
#define __ERROR_GPIO_PIN_INIT__

#define ERROR_HANDLE_PORT           GPIOA
#define ERROR_HANDLE_PIN_NO         GPIO_PIN_NO_0


#define ERROR_HANDLE_TIMOUT_PORT     GPIOA
#define ERROR_HANDLE_TIMOUT_PIN_NO   GPIO_PIN_NO_1

#endif

/* Function Prototypes */
void Error_Handler(Error_Status errstatus);
void GPIO_Error_LED_Init(void);
void GPIO_Timeout_LED_Init(void);


#endif /* ERROR_HANDLING_H_ */
