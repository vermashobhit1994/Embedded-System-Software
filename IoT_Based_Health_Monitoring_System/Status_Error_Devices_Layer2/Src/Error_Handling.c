/*
 * Error_Handling.c
 *
 *  Created on: 05-Jul-2021
 *      Author: vermas
 */
/* Description : This file contains the function definition for handling of
 *               error if occured due to error in code execution or some
 *               of peripheral needed for whole system isn't being connected.
 */

#include"Error_Handling.h"



/* *********************************Documentation Section **********************************************
 * @func                         : GPIO_Error_LED_Init
 * @brief                        : Initialize the Error led
 * @return                       : None
 * @Note                         : Must be done for one time.
 */

void GPIO_Error_LED_Init(void)
{
	GPIO_Handle_t gpioerrorled;
	gpioerrorled.pGPIOx = ERROR_HANDLE_PORT;
	gpioerrorled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioerrorled.GPIO_PinConfig.GPIO_PinNumber = ERROR_HANDLE_PIN_NO;
	gpioerrorled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PUSH_PULL ;
	gpioerrorled.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_NO_PUPD ;
	gpioerrorled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	GPIO_Init(&gpioerrorled);
}


/* *********************************Documentation Section **********************************************
 * @func                         : GPIO_Error_LED_Init
 * @brief                        : Initialize the Error led
 * @return                       : None
 * @Note                         : Must be done for one time.
 */

void GPIO_Timeout_LED_Init(void)
{
	GPIO_Handle_t gpioerrorled;
	gpioerrorled.pGPIOx = ERROR_HANDLE_TIMOUT_PORT;
	gpioerrorled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioerrorled.GPIO_PinConfig.GPIO_PinNumber = ERROR_HANDLE_TIMOUT_PIN_NO;
	gpioerrorled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PUSH_PULL ;
	gpioerrorled.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_NO_PUPD ;
	gpioerrorled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	GPIO_Init(&gpioerrorled);
}


/* *********************************Documentation Section **********************************************
 * @func                         : Error_Handler
 * @brief                        : This function handle the error by turning ON and OFF led
 * @param[in]                    : Flag indicating Error status
 * @return                       : None
 * @Note                         :
 */
void Error_Handler(Error_Status errstatus)
{

	//Turn ON the LED if error has occured
	if(errstatus == STATUS_ERROR)
	{
		GPIO_WriteToOutputPin(ERROR_HANDLE_PORT,ERROR_HANDLE_PIN_NO ,ERROR_STATUS_LED_ON);
	}
	else if (errstatus == STATUS_TIMEOUT)
	{
		GPIO_WriteToOutputPin(ERROR_HANDLE_TIMOUT_PORT,ERROR_HANDLE_TIMOUT_PIN_NO ,ERROR_STATUS_LED_ON);
	}
	//Turn OFF Error and timeout led. Used when error has gone
	else
	{
		GPIO_WriteToOutputPin(ERROR_HANDLE_PORT,ERROR_HANDLE_PIN_NO ,ERROR_STATUS_LED_OFF);
		GPIO_WriteToOutputPin(ERROR_HANDLE_TIMOUT_PORT,ERROR_HANDLE_TIMOUT_PIN_NO ,ERROR_STATUS_LED_OFF);

	}

}
