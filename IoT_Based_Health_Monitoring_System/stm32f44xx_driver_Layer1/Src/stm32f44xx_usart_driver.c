/*
 * stm32f44xx_uart_driver.c
 *
 *  Created on: 30-Jan-2021
 *      Author: vermas
 */


#include"stm32f44xx_usart_driver.h"




/*******************************************************************************************
 ****************************API's definitions supported by this driver*********************************/


/* *********************************Documentation Section **********************************************
 * @fn                           : USART_PeriClockCtrl
 * @brief                        : Enable or Disable the peripheral clock for USART/UART.
 *  @param[in]                   : Pointer to Register structure
 * @param[in]                    : Enable or Disable Macro.
 * @return                       : None.
 * Special Note                  : USART1 and USART6 connected to APB2 bus.
 *                                 USART2, USART3, UART4, UART5 connected to APB1 bus.
 */
void USART_PeriClockCtrl(USART_RegDef_t *pUSARTx, uint8_t ENOrDI)
{
	//enable the clock for USART1, USART2, USART3, UART4, UART5
		if(ENOrDI == ENABLE)
		{
			if(pUSARTx == USART1)
				USART1_PERI_CLK_EN();
			else if(pUSARTx == USART2)
				USART2_PERI_CLK_EN();
			else if(pUSARTx == USART3)
				USART3_PERI_CLK_EN();
			else if (pUSARTx == UART4)
				UART4_PERI_CLK_EN();
			else if (pUSARTx == UART5)
				UART5_PERI_CLK_EN();
			else if (pUSARTx == USART6)
				USART6_PERI_CLK_EN();


		}
		//disable the clock USART1, USART2, USART3, UART4, UART5
		else
		{
			if(pUSARTx == USART1)
				USART1_PERI_CLK_DI();
			else if(pUSARTx == USART2)
				USART2_PERI_CLK_DI();
			else if(pUSARTx == USART3)
				USART3_PERI_CLK_DI();
			else if (pUSARTx == UART4)
				UART4_PERI_CLK_DI();
			else if (pUSARTx == UART5)
				UART5_PERI_CLK_DI();
			else if (pUSARTx == USART6)
				USART6_PERI_CLK_DI();


		}

}

/* *********************************Documentation Section **********************************************
 * @fn                           : USART_PeripheralControl
 * @brief                        : Enable the USART/UART Peripheral.
 * @param[in]                   : Pointer to handle structure
 * @param[in]                    : Enable or Disable Macro.
 * @return                       : None.
 * Special Note                  :
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx,uint8_t ENOrDI)
{
	if(ENOrDI == ENABLE)
		pUSARTx -> USART_CR1 |= (1 << USART_CR1_UE);
	else
		pUSARTx -> USART_CR1 &= ~(1 << USART_CR1_UE);
}



/* *********************************Documentation Section **********************************************
 * @fn                           : USART_ClearFlag
 * @brief                        : Clear the Flag of Status Register.
 * @param[in]                    : Pointer to USART Register Definition Structure.
 *  @param[in]                   : Name of Status Flag.
 * @return                       : None
 * Special Note                  :
 *
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx,uint32_t FlagName)
{
	if((pUSARTx -> USART_SR) & FlagName)
		pUSARTx -> USART_SR &= ~(FlagName)	;

}

/* *********************************Documentation Section **********************************************
 * @fn                           : USART_GetFlagStatus
 * @brief                        : Return the Current status of Flag.
 *  @param[in]                   : Pointer to USART register structure
 * @param[in]                    : Flag name from Status Register(USART_SR).
 * @return                       : Current Status of Flag.
 * Special Note                  :
 */

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx,uint32_t FlagName)
{
	/*test the FlagName  in USART_SR Register.
		If the value is set return 1(FLAG_SET) */
		if((pUSARTx -> USART_SR) & FlagName)
			return USART_FLAG_SET;

		//else return 0(FLAG_RESET)
		return USART_FLAG_RESET;
}

/* *********************************Documentation Section **********************************************
 * @fn                           : USART_IRQInterruptConfig
 * @brief                        : Enable the Interrupt in NVIC in processor side corresponding
 *                                 to IRQ Number.
 *  @param[in]                   : IRQ Number corresponding to Interrupt.
 * @param[in]                    : Enable or Disable Macro.
 * @return                       : None.
 * Special Note                  :
 */

//to configure the Interrupt i.e IRQ no of USART
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENOrDI)
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
 * @fn                           : USART_IRQPriorityConfig
 * @brief                        : Configure the priority of Interrupt.
 *  @param[in]                   : Interrupt Request Number.
 * @param[in]                    : Priority of Interrupt.
 * @return                       : None.
 * Special Note                  :
 */

//to configure the priority of USART
void USART_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
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
 * @fn                           : USART_DeInit
 * @brief                        : Deinit the peripheral
 *  @param[in]                   : Pointer to USART register structure.
 * @return                       : None.
 * Special Note                  :
 */
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if(pUSARTx  == USART1)
		USART1_REG_RESET();
	else if(pUSARTx  == USART2)
		USART2_REG_RESET();
	else if(pUSARTx  == USART3)
		USART3_REG_RESET();
	else if(pUSARTx  == UART4)
		UART4_REG_RESET();
	else if(pUSARTx  == UART5)
		UART5_REG_RESET();
	else if(pUSARTx  == USART6)
		USART6_REG_RESET();

}






/* *********************************Documentation Section **********************************************
 * @fn                           : USARTx_Init
 * @brief                        : Init the usart peripheral.
 * @param[in]                    : Pointer to handle structure
 * @return                       : None.
 * Special Note                  : Refer to page 801 of reference manual
 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{



	//Step1 : Enable the Peripheral clock
	USART_PeriClockCtrl(pUSARTHandle-> pUSARTx, ENABLE);




	//Step1 : Enable the peripheral
	USART_PeripheralControl(pUSARTHandle -> pUSARTx, ENABLE);

	//Step2 : configure the word length as given by user
	pUSARTHandle -> pUSARTx -> USART_CR1 |= ( (pUSARTHandle -> USART_Config.USART_WordLength) << USART_CR1_M);

	//Step3 : Configure the number of stop bits as given by user
	pUSARTHandle -> pUSARTx -> USART_CR2 |= ( (pUSARTHandle ->  USART_Config.USART_NoOfStopBits) << USART_CR2_STOP);

	//Step4 : Configure the baud rate in USART_BRR Register
	uint32_t user_baudrate = pUSARTHandle -> USART_Config.USART_BAUD_RATE;

	/* temporary variables to store the data */
	uint16_t USART_DIV_Mantissa = 0;
	uint8_t USART_DIV_Fraction = 0;
	uint32_t Pclk2 = 0, Pclk1 = 0;
	float USART_DIV_Value = 0;

	//check the usart number connected to APB2 bus
	if ( (pUSARTHandle -> pUSARTx == USART1)  ||
		 (pUSARTHandle -> pUSARTx == USART6) )
	{
		Pclk2 = RCC_GetPCLK2Value();
		USART_DIV_Value = Pclk2 / (8 * 2 * user_baudrate);

		//mantissa is the integer part of float value . e.g 18.25 -> 18
		//fraction is the decimal part of float value . e.g 18.25 -> 0.25
		USART_DIV_Mantissa = (uint16_t)USART_DIV_Value;
		USART_DIV_Fraction = (uint8_t) (USART_DIV_Value * 16);


	}
	//check the usart number connected to APB1 bus
	else if ( (pUSARTHandle -> pUSARTx == USART3)  ||
		 (pUSARTHandle -> pUSARTx == USART2) ||
		 (pUSARTHandle -> pUSARTx == UART4)||
		 (pUSARTHandle -> pUSARTx == UART5) )
	{
		Pclk1 = RCC_GetPCLK1Value();
		USART_DIV_Value = Pclk1 / (8 * 2 * user_baudrate);

		//mantissa is the integer part of float value . e.g 18.25 -> 18
		//fraction is the decimal part of float value . e.g 18.25 -> 0.25
		USART_DIV_Mantissa = (uint16_t)USART_DIV_Value;
		USART_DIV_Fraction = (uint8_t) (USART_DIV_Value * 16);


	}


	//Write the values to USART_BRR Register
	pUSARTHandle -> pUSARTx -> USART_BRR =( (USART_DIV_Mantissa << USART_BRR_DIV_MANTISSA) |
			                                 (USART_DIV_Fraction << USART_BRR_DIV_FRACTION) );


	uint8_t USART_Mode = pUSARTHandle ->USART_Config.USART_Mode;

		//Enable the receiver according to user selection
		if(USART_Mode == USART_MODE_ONLY_RX )
		{
			//Step1 : Enable the receiver
			pUSARTHandle -> pUSARTx -> USART_CR1 |= (1 << USART_CR1_RE);
		}
		//if the mode is selected both Rx and Tx then enable the transmitter also
		else if (USART_Mode == USART_MODE_TXRX)
		{
			pUSARTHandle -> pUSARTx -> USART_CR1 |= (1 << USART_CR1_TE);
			pUSARTHandle -> pUSARTx -> USART_CR1 |= (1 << USART_CR1_RE);
		}



}




/* *********************************Documentation Section **********************************************
 * @fn                           : USARTx_Transmit_Byte_Data
 * @brief                        : Transmit the data byte by byte.
 * @param[in1]                   : Pointer to USART register structure.
 * @param[in2]                   : Pointer to data to be send
 * @param[in3]                   : Count of number of bytes
 * @return                       : None.
 * Special Note                  : checkout Page 801 of Reference Manual.
 */
void USARTx_Transmit_Byte_Data(USART_Handle_t *pUSARTHandle, uint8_t *pData,uint32_t Byte_Count)
{

	/*
	//Step5 : Enable the transmitter which Send the Idle frame as first frame.
	//Enable the transmitter mode only when the mode is selected as transmitter or
	//both
	uint8_t USART_Mode = pUSARTHandle -> USART_Config.USART_Mode;

	//if the transmit only mode is selected then enable TE bit
	if(USART_Mode == USART_MODE_ONLY_TX)
		pUSARTHandle -> pUSARTx -> USART_CR1 |= (1 << USART_CR1_TE);
	//if both mode are selected then enable the RE bit also
	else if (USART_Mode == USART_MODE_TXRX)
	{
		pUSARTHandle -> pUSARTx -> USART_CR1 |= (1 << USART_CR1_TE);
		pUSARTHandle -> pUSARTx -> USART_CR1 |= (1 << USART_CR1_RE);
	}
	*/
	uint8_t USART_Mode = pUSARTHandle -> USART_Config.USART_Mode;
	//if any of above mode is selected then only send the data
	if( (USART_Mode == USART_MODE_ONLY_TX) || (USART_Mode == USART_MODE_TXRX) )
	{
		while(Byte_Count > 0)
		{
			//Step1 : wait until the TXE bit becomes high to indicate DR register is empty
			//When the TXE bit set then Data move from DR to shift register
			while( !(pUSARTHandle -> pUSARTx ->USART_SR & (1 << USART_SR_TXE)) );

			//Step2 : Write the data in DR register. This clear the TXE bit
			pUSARTHandle -> pUSARTx -> USART_DR = *pData++;

			Byte_Count--;

		}

		//for last byte. The data is still in DR register
		if(Byte_Count == 0)
		{
			//wait until the TC = 1 i.e last frame transmitted successfully
			while(!(pUSARTHandle-> pUSARTx -> USART_SR & (1 << USART_SR_TC))) ;


			//clear the TC bit. Used only for multibuffer communication.
			//USART_ClearFlag(pUSARTHandle -> pUSARTx, USART_SR_TC);
		}
	}
}


/* *********************************Documentation Section **********************************************
 * @fn                           : USARTx_Receive_Byte_Data
 * @brief                        : Receive the data byte by byte.
 * @param[in1]                   : Pointer to USART register structure.
 * @param[in2]                   : Pointer to buffer where data gets stored
 * @param[in3]                   : Count of number of bytes to be received.
 * @return                       : None.
 * Special Note                  : checkout Page 804 of Reference Manual.
 */

//How to find the data bytes received?
uint32_t USARTx_Receive_Byte_Data(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t RxBufferlen)
{
	//first reset the buffer
	memset(pRxBuffer, 0, RxBufferlen);

	uint8_t USART_Mode = pUSARTHandle -> USART_Config.USART_Mode;
	uint32_t Byte_Count= 0;
	if( (USART_Mode == USART_MODE_ONLY_RX) || (USART_Mode == USART_MODE_TXRX) )
	{

		//continue the loop until the data is received
		//do
		{
			//Wait until the RXNE is set to indicate data pushed from shift register
			//to DR
			while(! (pUSARTHandle -> pUSARTx -> USART_SR & (1 << USART_SR_RXNE)) );

			//Clear the RXNE bit by reading the DR register
			*pRxBuffer++ = pUSARTHandle -> pUSARTx ->USART_DR;
			Byte_Count++;


		}//while(*pRxBuffer != '\n');


	}
	return Byte_Count;


}

/* *********************************Documentation Section **********************************************
 * @fn                           :
 * @brief                        :
 * @param[in1]                   :
 * @param[in2]                   :
 * @return                       :
 * @Note                         :
 */
void USART_Interrupt_Config(USART_Handle_t *pUSARTHandle, uint8_t TxOrRxInterrupt)
{
	if(TxOrRxInterrupt == USART_INTERRUPT_TX_EN)
		pUSARTHandle -> pUSARTx -> USART_CR1 |= (1 << USART_CR1_TXEIE);

	else if (TxOrRxInterrupt == USART_INTERRUPT_TX_DI)
		pUSARTHandle -> pUSARTx -> USART_CR1 &= ~(1 << USART_CR1_TXEIE);

	else if (TxOrRxInterrupt == USART_INTERRUPT_RX_EN)
			pUSARTHandle -> pUSARTx -> USART_CR1 |= (1 << USART_CR1_RXNEIE);

	else if (TxOrRxInterrupt == USART_INTERRUPT_RX_DI)
			pUSARTHandle -> pUSARTx -> USART_CR1 &= ~(1 << USART_CR1_RXNEIE);


}


/* *********************************Documentation Section **********************************************
 * @fn                           :
 * @brief                        :
 * @param[in1]                   :
 * @param[in2]                   :
 * @return                       :
 * @Note                         :
 */
void USART_TX_OR_RX_Byte_IRQConfig(USART_Handle_t *pUSARTHandle)
{

	uint8_t USART_Mode = pUSARTHandle -> USART_Config.USART_Mode;

	//if any of above mode is selected then only send the data
	if( (USART_Mode == USART_MODE_ONLY_TX)  )
	{

		//enable the interrupt when transmit first byte of data
		USART_Interrupt_Config(pUSARTHandle , USART_INTERRUPT_TX_EN);


	}
	else if(USART_Mode == USART_MODE_ONLY_RX)
	{

		//enable the interrupt when transmit first byte of data
		USART_Interrupt_Config(pUSARTHandle , USART_INTERRUPT_RX_EN);


	}
	else if (USART_Mode == USART_MODE_TXRX)
	{
		//enable the interrupt when transmit first byte of data
		USART_Interrupt_Config(pUSARTHandle , USART_INTERRUPT_TX_EN);

		//enable the interrupt when receive first byte of data
		USART_Interrupt_Config(pUSARTHandle , USART_INTERRUPT_RX_EN);

	}

	//Step2 : Enable the interrupt on processor side
	USART_IRQInterruptConfig(IRQ_NO_USART2, ENABLE);
	USART_IRQPriorityConfig(IRQ_NO_USART2, NVIC_IRQ_PRIO0);


}




/* *********************************Documentation Section **********************************************
 * @fn                           : USART_Transmit_Data_Callback
 * @brief                        : Callback function when interrupt is triggered
 * @param[in1]                   :
 * @param[in2]                   :
 * @return                       :
 * @Note                         : This function put the data to DR register each time interrupt is triggered.
 */

void USART_Transmit_Byte_Data_Callback(USART_Handle_t *pUSARTHandle,USARTx_Param_Config *pUSARTParam)
{

	//Step1 : Enable the interrupt only if is not enabled
	if(USART_GetFlagStatus(pUSARTHandle -> pUSARTx,USART_CR1_TXEIE ) == USART_FLAG_RESET)
		USART_Interrupt_Config(pUSARTHandle , USART_INTERRUPT_TX_EN);


	pUSARTHandle -> pUSARTx -> USART_DR = *(pUSARTParam -> pTxData);
	(pUSARTParam -> pTxData)++;
	--(pUSARTParam -> TxLen);






}

void USART_Receive_Byte_Data_Callback(USART_Handle_t *pUSARTHandle,USARTx_Param_Config *pUSARTParam)
{
	//Step1 : Enable the Receive interrupt only if is not enabled
	if(USART_GetFlagStatus(pUSARTHandle -> pUSARTx,USART_CR1_RXNEIE ) == USART_FLAG_RESET)
		USART_Interrupt_Config(pUSARTHandle , USART_INTERRUPT_RX_EN);

	//put the data from DR to buffer
	*(pUSARTParam -> pRxData) = pUSARTHandle -> pUSARTx -> USART_DR ;
	(pUSARTParam -> pRxData)++;
	(pUSARTParam -> RxLen)++;

}




