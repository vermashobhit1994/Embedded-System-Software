/*
 * stm32f44xx_gpio_driver.h
 *
 *  Created on: 15-Jul-2020
 *      Author: vermas
 */

#ifndef INC_STM32F44XX_GPIO_DRIVER_H_
#define INC_STM32F44XX_GPIO_DRIVER_H_


#include"stm32f446xx.h"//including the MCU specific header file

/* here the possible values using the link i.e @*/
typedef struct
{
	uint8_t GPIO_PinNumber;//possible values from @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode; //possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;//possible values from @GPIO_OPSPEED
	//for controlling the pull up and pull down
	uint8_t GPIO_PinPuPdCtrl;//possible values from @GPIO_PULL_UP_PULL_DOWN
	uint8_t GPIO_PinOPType;//possible values from @GPIO_OP_TYPES
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct
{
	//pointer to hold the base address of GPIO peripheral
	 GPIO_RegDef_t *pGPIOx;//pGPIO holds the base address of GPIO port to which pin belongs. Here p means pointer
	 GPIO_PinConfig_t GPIO_PinConfig;//this holds the GPIO pin configuration settings

}GPIO_Handle_t;

/**********************************GPIO PinNumber Macros ****************************************
 *@GPIO_PIN_NUMBERS
 */
#define GPIO_PIN_NO_0                            0
#define GPIO_PIN_NO_1                            1
#define GPIO_PIN_NO_2                            2
#define GPIO_PIN_NO_3                            3
#define GPIO_PIN_NO_4                            4
#define GPIO_PIN_NO_5                            5
#define GPIO_PIN_NO_6                            6
#define GPIO_PIN_NO_7                            7
#define GPIO_PIN_NO_8                            8
#define GPIO_PIN_NO_9                            9
#define GPIO_PIN_NO_10                           10
#define GPIO_PIN_NO_11                           11
#define GPIO_PIN_NO_12                           12
#define GPIO_PIN_NO_13                           13
#define GPIO_PIN_NO_14                           14
#define GPIO_PIN_NO_15                           15


/**********************************GPIO Pin Possible Modes Macros ****************************************
 *Here the Input can be on rising edge or falling edge
 *@GPIO_PIN_MODES
 */
/*If the value is less than 3 then it is non-interrupt mode else it is interrupt mode*/
//Non-Interrupt modes
#define GPIO_MODE_IN                           0
#define GPIO_MODE_OUT                          1
#define GPIO_MODE_ALTFUN                       2
#define GPIO_MODE_ANALOG                       3
//Interrupt modes
#define GPIO_MODE_INT_FALLEDGE_TRIGGER         4//INT is input on falling edge
#define GPIO_MODE_INT_RISINGEDGE_TRIGGER       5//INT is input on rising edge
#define GPIO_MODE_RISINGEDGE_FALLEDGE_TRIGGER  6//INt is input in both rising and falling edge


/*************************************GPIO Pin Possible output types Macros *************************************
 * Here the output can be push pull or open drain for the GPIO Port output type register
 * @GPIO_OP_TYPES
 */
#define GPIO_OPTYPE_PUSH_PULL                      0
#define GPIO_OPTYPE_OPEN_DRAIN                     1

/************************************GPIO Pin Possible output speed modes Macros *************************************
 * Here the modes can be slow, medium,fast and high for the GPIO Port output Speed Register
 * @GPIO_OPSPEED
 */
#define GPIO_SPEED_LOW                        0
#define GPIO_SPEED_MEDIUM                     1
#define GPIO_SPEED_HIGH                       2
#define GPIO_SPEED_VERY_HIGH                  3

/************************************GPIO Pin possible Pull up and Pull Down Configuration Macros *************************************
 *Configuration for the GPIO_PULL_UP_PULL_DOWN_REGISTER
 *@GPIO_PULL_UP_PULL_DOWN
 */
#define GPIO_PIN_NO_PUPD                         0
#define GPIO_PIN_PULL_UP                         1
#define GPIO_PIN_PULL_DOWN                       2



/*******************************************************************************************
 ****************************API's supported by this driver*********************************/

/* init and deint.  */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle); //initialize the GPIO Port and Pin

/* Here the parameter is base address of the peripheral*/
void GPIO_DEInit(GPIO_RegDef_t *pGPIOx);//deinitialize the GPIO Port i.e reset state

/*Peripheral clock setup
 * Enable or Disable the clock for GPIO peripheral
 * Here the first parameter is the pointer to GPIO General structure and second is the Enable
 * or disable the clock*/
void GPIO_PeriClock_Ctrl(GPIO_RegDef_t *pGPIOx,uint8_t ENOrDI);//enable and disable the peripheral clock

/*data read and write*/
/*here the first parameter is the base address of peripheral and second parameter is the pinNumber from which
we want to read . Return value is the state of pin i.e either high or low.*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);//to read from Input Pin

//here the return value is 16 bit wide since port is of 16 bit. First parameter is the base address of peripheral
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);//to read from Input Port

/*here the return value is void since write . first parameter is the base address of the peripheral , second
parameter is the pinNumber from to we want to send data and third paramter is the Value that we want to send*/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value);//to write to output pin

/* second parameter is the 16 bit wide port value taken to write to port*/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value);//to write to output port
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);//to toggle the pin

/*IRQ Configuration and IRQ Handling*/
//Second parameter whether to enable or disable the IRQ
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENOrDI);//to configure the Interrupt i.e IRQ no of GPIO Pin

//for the Interrupt Priority
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);

//here the parameter is the PinNumber upon which interrupt is done
void GPIO_IRQHandling(uint8_t PinNumber);//the actual function that gets called on Interrupt or Interrupt handler

#endif /* INC_STM32F44XX_GPIO_DRIVER_H_ */
