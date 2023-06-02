/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: 15-Jul-2020
 *      Author: vermas
 */

#include"stm32f44xx_gpio_driver.h"//including the Peripheral specific header file

/*******************************************************************************************
 ****************************API's definitions supported by this driver*********************************/

/* *********************************Documentation Section **********************************************
 * Function Name                 : GPIO_Init
 * Brief Description             : This function initialise the GPIO Port
 * Input Parameter 1 description : Base address of the gpio peripheral
 * Return Type                   : None
 * Special Note                  : None
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)//initialize the GPIO Port and Pin
{
	//enable the GPIO Peripheral Clock so no need to configure in application part.
	GPIO_PeriClock_Ctrl(pGPIOHandle ->pGPIOx, ENABLE);

	uint32_t temp = 0;
/* Here if the mode is less than 3 then it is non-interrupt mode else interrupt mode
 * Here we need to change only the Pins in the actual position and not touch other bits*/
	//1. Configure the mode of GPIO Pin
	if(pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode <= (GPIO_MODE_ANALOG) )
	{
		/*temp stores the pinmode input by the user.
		 * Here each pin takes 2 fields of GPIOMode register
		 * PinMode left shift by 2*PinNumber

		 */
		temp = (pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber));
		//reset the MODER Register PinNUmber bits
		//pGPIOHandle -> pGPIOx -> MODER &= ~(0x3 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);//clear the pinNumber

		//putting the value in MODER  register
		pGPIOHandle -> pGPIOx -> MODER |= temp;

	}
	/* Interrupt Mode handling*/
	else
	{
		//STEP 1: Configure the selection register
		//if the mode is falling edge
		if(pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_FALLEDGE_TRIGGER)
		{
			/*1. configure the falling trigger selection register(FTSR)
			 * and setting the pin corresponding to PinNumber
			 */
			EXTI -> EXTI_FTSR |= (1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding bit in RTSR to only enable the RTSR register
			EXTI -> EXTI_RTSR &= ~(1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);

		}
		//if the mode is rising edge
		else if(pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_RISINGEDGE_TRIGGER)
		{
			//1. configure the RTSR(Rising Trigger Selection Register)
			EXTI -> EXTI_RTSR |= (1<<pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding bit in FTSR
			EXTI -> EXTI_FTSR &= ~(1<<pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);

		}
		//if the mode is rising and falling edge
		else if(pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_RISINGEDGE_FALLEDGE_TRIGGER)
		{
			//1. configure both RTSR and FTSR
			//configure the RTSR
			EXTI -> EXTI_RTSR |= (1<<pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
			//configure the FTSR
			EXTI -> EXTI_FTSR |= (1<<pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
		}
		//STEP 2: configure the GPIO Port selection in SYSCFG_EXTICR(SYSConfig EXTI control register)
		uint8_t tmp1 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber/4;//select the EXTI Register
		uint8_t tmp2 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber % 4;//select the EXTI Register bits

		//enable the clock for the SYSCFG register
		SYSCFG_PERI_CLK_EN();

		//Define a Macro that takes the GPIOx base address and return the corresponding port code for the GPIOx
		//Done so as to choose the appropriate EXTI for Port.
		//for Onboard button it is GPIOC and that's why portcode = 2
		uint8_t PortCode = GPIO_BASE_ADDR_TO_CODE(pGPIOHandle -> pGPIOx);


		//putting the PortCode in the EXTICRx register
		SYSCFG -> SYSCFG_EXTICR[tmp1] = PortCode <<(4 * tmp2);

		//STEP 3: Enable the EXTI Interrupt delivery in IMR(Interrupt Mask Register)
		EXTI -> EXTI_IMR |= (1<<pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
	}

	//2. Configure the speed
	temp = 0;//again putting the value temp to zero
	temp = pGPIOHandle -> 	GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle -> pGPIOx -> OSPEEDR &= ~(0x3 <<pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);//clear the pinNumber
	pGPIOHandle -> pGPIOx -> OSPEEDR |= temp;//set the PinNumber

	//3. Configure the pull up or pull down settings
	temp = 0;//again putting the value temp to zero
	temp = pGPIOHandle -> 	GPIO_PinConfig.GPIO_PinPuPdCtrl << (2*pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle -> pGPIOx -> PUPDR &= ~(0x3 <<pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);//clear the pinNumber
	pGPIOHandle -> pGPIOx -> PUPDR |= temp;

	/*4. Configure the output type
	 * Here no need to multiply by 1 since each pin belong to only 1 bit
	 */
	temp = 0;//again putting the value temp to zero
	temp = pGPIOHandle -> 	GPIO_PinConfig.GPIO_PinOPType << (1*pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle -> pGPIOx -> OTYPER &= ~(0x1 <<pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);//clear the pinNumber
	pGPIOHandle -> pGPIOx -> OTYPER |= temp;

	//5. Configure the alternate functionality only when the mode selected in the PinMode is alternate function
	if(pGPIOHandle -> 	GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFUN)
	{
		//configure the alternate function registers
		uint8_t temp1,temp2;
		temp1 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber/8;//dividing by 8 to select whether the alternate low or high register
		temp2 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber%8;//to get the actual bit position
		pGPIOHandle -> pGPIOx -> AFR[temp1] &= ~(0xF <<(4*temp2));//since 4 bits
		pGPIOHandle -> pGPIOx -> AFR[temp1] |= (pGPIOHandle -> GPIO_PinConfig.GPIO_PinAltFunMode<<(4*temp2));
	}
}


/* *********************************Documentation Section **********************************************
 * Function Name                 : GPIO_DeInit
 * Brief Description             : This function Deinitialise the GPIO Port
 * Input Parameter 1 description : Base address of the gpio peripheral
 * Return Type                   : None
 * Special Note                  : Here first we need to set the bit in RCC_AHB1RSTR and then reset the bit
 */
void GPIO_DEInit(GPIO_RegDef_t *pGPIOx)//deinitialize the GPIO Port i.e reset state
{

		if(pGPIOx == GPIOA)
			GPIOA_REG_RESET();
		else if(pGPIOx == GPIOB)
			GPIOB_REG_RESET();
		else if(pGPIOx == GPIOC)
			GPIOC_REG_RESET();
		else if(pGPIOx == GPIOD)
			GPIOD_REG_RESET();
		else if(pGPIOx == GPIOE)
			GPIOE_REG_RESET();
		else if(pGPIOx == GPIOF)
			GPIOF_REG_RESET();
		else if(pGPIOx == GPIOG)
			GPIOG_REG_RESET();
		else if(pGPIOx == GPIOH)
			GPIOH_REG_RESET();
		else if(pGPIOx == GPIOI)
			GPIOI_REG_RESET();


}

/* *********************************Documentation Section **********************************************
 * Function Name                 : GPIO_PeriClock_Ctrl
 * Brief Description             : This function enable or disable the peripheral clock for given GPIO port
 * Input Parameter 1 description : Base address of the gpio peripheral
 * Input Parameter 2 description : ENABLE or DISABLE Macro
 * Return Type                   : None
 * Special Note                  : None
 */
void GPIO_PeriClock_Ctrl(GPIO_RegDef_t *pGPIOx,uint8_t ENOrDI)//enable and disable the peripheral clock
{
	//enable the clock
	if(ENOrDI == ENABLE)
	{
		if(pGPIOx == GPIOA)
			GPIOA_PERI_CLK_EN();
		else if(pGPIOx == GPIOB)
			GPIOB_PERI_CLK_EN();
		else if(pGPIOx == GPIOC)
			GPIOC_PERI_CLK_EN();
		else if(pGPIOx == GPIOD)
			GPIOD_PERI_CLK_EN();
		else if(pGPIOx == GPIOE)
			GPIOE_PERI_CLK_EN();
		else if(pGPIOx == GPIOF)
			GPIOF_PERI_CLK_EN();
		else if(pGPIOx == GPIOG)
			GPIOG_PERI_CLK_EN();
		else if(pGPIOx == GPIOH)
			GPIOH_PERI_CLK_EN();
		else if(pGPIOx == GPIOI)
			GPIOI_PERI_CLK_EN();

	}
	//disable the peripheral clock
	else
	{
		if(pGPIOx == GPIOA)
			GPIOA_PERI_CLK_DI();
		else if(pGPIOx == GPIOB)
			GPIOB_PERI_CLK_DI();
		else if(pGPIOx == GPIOC)
			GPIOC_PERI_CLK_DI();
		else if(pGPIOx == GPIOD)
			GPIOD_PERI_CLK_DI();
		else if(pGPIOx == GPIOE)
			GPIOE_PERI_CLK_DI();
		else if(pGPIOx == GPIOF)
			GPIOF_PERI_CLK_DI();
		else if(pGPIOx == GPIOG)
			GPIOG_PERI_CLK_DI();
		else if(pGPIOx == GPIOH)
			GPIOH_PERI_CLK_DI();
		else if(pGPIOx == GPIOI)
			GPIOI_PERI_CLK_DI();

	}
}

/* *********************************Documentation Section **********************************************
 * Function Name                 : GPIO_ReadFromInputPin
 * Brief Description             : This function read the pin
 * Input Parameter 1 description : Base address of the gpio peripheral
 * Input Parameter 2 description : PinNumber
 * Return Type                   : 1 or 0 i.e set or reset
 * Special Note                  : None
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)//to read from Input Pin
{
	uint8_t value;//temporary variable to store the value read
	/*right shift the corresponding bit to LSB and then mask it with 1 to extract  LSB*/
	/* Here typecast to uint8_t since the value is 8 bit */
	value = (uint8_t)(pGPIOx->IDR >> PinNumber) & 0x00000001;
	return value;

}

/* *********************************Documentation Section **********************************************
 * Function Name                 : GPIO_ReadFromInputPort
 * Brief Description             : This function read the port
 * Input Parameter 1 description : Base address of the gpio peripheral
 * Return Type                   : 16 bit value of GPIOx port
 * Special Note                  : None
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)//to read from Input Port
{
	uint16_t value;//temporary variable to store the value read
	value = (uint16_t)(pGPIOx->IDR );//read the entire GPIOx Port
	return value;

}
/* *********************************Documentation Section **********************************************
 * Function Name                 : GPIO_WriteToOutputPin
 * Brief Description             : This function write to the port
 * Input Parameter 1 description : Base address of the gpio peripheral
 * Input Parameter 1 description : Pin NUmber of gpio
 * Input Parameter 1 description : 8 bit value
 * Return Type                   : None
 * Special Note                  : None
 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value)//to write to output pin
{
	//if the value is SET i.e 1
	if(Value == GPIO_PIN_SET)
	{
		//write the 1 to the bit field at the corresponding pin number
		//pGPIOx->ODR |= (1<<PinNumber);
		pGPIOx -> BSRR |= (1 << PinNumber);//writing into BSRR cause to write in ODR
	}
	else
	{
		//write the 0 to the bit field at the corresponding pin number
		//pGPIOx->ODR &= ~(1<<PinNumber);
		pGPIOx -> BSRR |= ((1 << PinNumber) << 16);//writing into BSRR cause to write in ODR
	}
}

/* *********************************Documentation Section **********************************************
 * Function Name                 : GPIO_WriteToOutputPort
 * Brief Description             : This function write to the GPIO Port
 * Input Parameter 1 description : Base address of the gpio peripheral
 * Input Parameter 2 description : Value to be written
 * Return Type                   : None
 * Special Note                  : None
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value)//to write to output port
{
	pGPIOx ->ODR = Value;
}

/* *********************************Documentation Section **********************************************
 * Function Name                 : GPIO_ToggleOutputPin
 * Brief Description             : This function toggle the Pin corresponding to PinNUmber
 * Input Parameter 1 description : Base address of the gpio peripheral
 * Input Parameter 2 description : Pin Number
 * Return Type                   : None
 * Special Note                  : None
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)//to toggle the pin
{
	pGPIOx -> ODR ^= (1 << PinNumber);
}

/* *********************************Documentation Section **********************************************
 * Function Name                 : GPIO_IRQInterruptConfig
 * Brief Description             : This function configure the interrupt
 * Input Parameter 1 description : Base address of the gpio peripheral
 * Input Parameter 2 description : 1 or 0
 * Return Type                   : None
 * Special Note                  : Here the IRQNumber is upto 96 only and that's why we use upto ISRx i.e x = 0 to 2 register
 *                                 Done on the Processor side and it's processor specific
 *                                 Below function to enable or disable the interrupt
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENOrDI)//to configure the Interrupt i.e IRQ no of GPIO Pin
{

	/* Configurations for the Interrupt Set Enable Register(ISER)*/
	if(ENOrDI == ENABLE)//if the Interrupt is enable
	{
		//select the IRQ No range
		if(IRQNumber <= 31)//since each register is of 32 bit size
		{
			//code for ISER0 Register
			//putting the IRQNumber in NVIC_ISER0 Register
			(*NVIC_ISER0_BASE_ADDR) |= (1<<IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <64)//for getting the next IRQNumber add 32.
		{
			//code for ISER1Register
			/*putting the IRQNumber in NVIC_ISER0 Register.
			 * Since to select the particular register we modulus(%) by 32 since each register is of 32 bit size*/
			(*NVIC_ISER1_BASE_ADDR) |= (1<<(IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber <96)//for getting the next IRQNumber add 32
		{
			//code for ISER2 Register
			//putting the IRQNumber in NVIC_ISER0 Register
			(*NVIC_ISER2_BASE_ADDR) |= (1<<(IRQNumber  % 64));
		}
	}

	else //configurations for the ICER(Interrupt Clear Enable Register) Register to disable the interrupt
		{
				//select the IRQ No range
				if(IRQNumber <= 31)//since each register is of 32 bit size
				{
					//code for ICER0 Register
					//putting the IRQNumber in NVIC_ISER0 Register
					*NVIC_ICER0_BASE_ADDR |= (1<<(IRQNumber ));
				}
				else if(IRQNumber > 31 && IRQNumber <64)//for getting the next IRQNumber add 32.
				{
					//code for ICER1Register
					//putting the IRQNumber in NVIC_ISER0 Register
					(*NVIC_ICER1_BASE_ADDR) |= (1<<(IRQNumber % 32) );
				}
				else if(IRQNumber >= 64 && IRQNumber <96)//for getting the next IRQNumber add 32
				{
					//code for ICER2 Register
					//putting the IRQNumber in NVIC_ISER0 Register
					(*NVIC_ICER2_BASE_ADDR) |= (1<<(IRQNumber % 64));
				}

		}

}
/* *********************************Documentation Section **********************************************
 * Function Name                 : GPIO_IRQPriorityConfig
 * Brief Description             : This function configure the interrupt Priority
 * Input Parameter 1 description : IRQ Number
 * Input Parameter 2 description : Priority
 * Return Type                   : None
 * Special Note                  : Make the IRQPriority as 32 bit to since we're shifting by value>15
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//Step 1. Select the IPRx Register
	uint8_t iprx = IRQNumber / 4;
	//select the section from a particular iprx register
	uint8_t iprx_section = IRQNumber % 4;
	/*Here *4 since to shift the iprx register base address by 4
	 * Here *8 to access that particular bit in iprx register
	 * NO_OF_PRIORITY_BITS_IMPLEMENTED is depend on processor and 8- for select the high 4 bits in priority
	 * since the lower bits aren't implemented in each priority field	*/
	uint8_t shift_amt = (8 * iprx_section) + (8 - NO_OF_PRIORITY_BITS_IMPLEMENTED);
	//NOTE: In NVIC Register lower 8 bits isn't implemented so max value is 15
	*(NVIC_PR_BASE_ADDR + (iprx  )) |= (IRQPriority <<  shift_amt);


}
/* *********************************Documentation Section **********************************************
 * Function Name                 : GPIO_IRQHandling
 * Brief Description             : This function handle the interrupt
 * Input Parameter 1 description : Pin Number
 * Return Type                   : None
 * Special Note                  : Check page 244 of datasheet.
 */

void GPIO_IRQHandling(uint8_t PinNumber)//the actual function that gets called on Interrupt or Interrupt handler
{
	//clear the EXTI_PR Register corresponding to PinNumber
	if( (EXTI -> EXTI_PR) &(1<< PinNumber))//if the PR bit is set
	{
		//clear that pending register bit by writing 1 to it
		EXTI -> EXTI_PR |= (1<< PinNumber);

	}
}
