/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: 18-Sep-2020
 *      Author: vermas
 */



#include"stm32f44xx_i2c_driver.h"



/*******************************************************************************************
 ****************************API's definitions supported by this driver*********************************/

/* *********************************Documentation Section **********************************************
 * @fn                           : I2C_PeriClock_Ctrl
 * @brief                        : Function to Control the peripheral clock of I2C.
 * @param[in]                    : Base address of the I2C Register definition structure .
 * @param[in]                    : Bit to enable or disable.
 * @return                       : None
 * Special Note                  : None
 */

void I2C_PeriClock_Ctrl(I2C_RegDef_t *pI2Cx,uint8_t ENOrDI)//enable and disable the peripheral clock
{
	//enable the clock for I2C1, I2C2, I2C3
	if(ENOrDI == ENABLE)
	{
		if(pI2Cx == I2C1)
			I2C1_PERI_CLK_EN();
		else if(pI2Cx == I2C2)
			I2C2_PERI_CLK_EN();
		else if(pI2Cx == I2C3)
			I2C3_PERI_CLK_EN();

	}
	//disable the clock I2C1, I2C2, I2C3
	else
	{
		if(pI2Cx == I2C1)
			I2C1_PERI_CLK_DI();
		else if(pI2Cx == I2C2)
			I2C2_PERI_CLK_DI();
		else if(pI2Cx == I2C3)
			I2C3_PERI_CLK_DI();

	}
}






/* *********************************Documentation Section **********************************************
 * @fn                           : I2C_Init
 * @brief                        : Function to initialize the parameters given by user.
 * @param[in]                    : Base address of the I2C Handle structure .
 * @return                       : None
 * Special Note                  : Refer to Page 765 in RM0390 reference manual.
 *                                 Here the I2C Init must be called first and then
 *                                 Peripheral must be enable
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;//temporary register to store the data

	//enable the I2C Peripheral Clock
	I2C_PeriClock_Ctrl(pI2CHandle -> pI2Cx,ENABLE);

	/******************* step 1: enable the ACK***************************************/
	 /* done since input by user*/
		tempreg = 0;
		tempreg |= ( (pI2CHandle -> I2C_Config.I2C_ACKCtrl) << I2C_CR1_ACK);
	    (pI2CHandle ->pI2Cx->I2C_CR[0]) |= tempreg;//put the value in I2C_CR1 Ack field.
         //(*(pI2CHandle ->pI2Cx->I2C_CR[0])) |= tempreg;

	/*****************Step 2: configure the FREQ fields of I2C_CR2 Register  ********/
	/* This is done to set the APB1 frequency value for I2C peripheral*/
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value()/1000000U;//to get only 16 instead of 16000000
	//to check only 5 LSB Bits and store value in I2C_CR2 Register.
	pI2CHandle -> pI2Cx -> I2C_CR[1] |= (tempreg & 0x3F);






	/******************* Step3 : Select the slave address if device is slave ***********************************/
	tempreg = 0;
	//getting the slave address as entered by user.
	tempreg |= pI2CHandle -> I2C_Config.I2C_DeviceAddress << 1;//for excluding the ADD0 bit
	tempreg |= (1<<14);//As specified by reference Manual

	//copy the value in ADD field of I2C_OAR1 Register
	//putting the 1 in 14th bit of I2C_OAR1 Register i.e reserved bit.
	pI2CHandle ->pI2Cx->I2C_OAR[0] |= (tempreg );

	/******************* Step4 : Select the CCR Value according to mode ***********************************/
    /* This is used to produce the clock of I2C peripheral */
	//CCR Calculations
	uint16_t ccr_value = 0;//I2C_CCR Register is of 16 bit
	tempreg = 0;

	//Step a : configure CCR value for the standard mode i.e 100 KHz
	if(pI2CHandle -> I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_ST_MODE)
	{
		//get the ccr value using the pclk1 and i2cscl clock.
		ccr_value = RCC_GetPCLK1Value() / (2 * pI2CHandle -> I2C_Config.I2C_SCLSpeed);
		tempreg |= (ccr_value & 0xFFF);//check only LSB 12 bits and store in tempreg.
	}

	//Step b: configure CCR value for the fast mode i.e > 100KHz
	else
	{
		//step 1. select the mode as fast .
		tempreg |= (1 << I2C_CCR_F_S);

		//step2 select duty cycle as entered by user.
		tempreg |= ( (pI2CHandle -> I2C_Config.I2C_FMDUTYCYCL) << I2C_CCR_DUTY);

		//check the duty cycle value = 0 i.e 2
		if(pI2CHandle -> I2C_Config.I2C_FMDUTYCYCL == I2C_FM_DUTY2)
			ccr_value = RCC_GetPCLK1Value()/ (3 * pI2CHandle -> I2C_Config.I2C_SCLSpeed);
		else
			ccr_value = RCC_GetPCLK1Value()/ (25 * pI2CHandle -> I2C_Config.I2C_SCLSpeed);

		//put the value from ccr_value in tempreg where only 12 bits are taken.
		tempreg |= ccr_value & 0xFFF;

	}
	//put the value in I2C_CCR Register from tempreg. Must be done when the I2C Peripheral is disabled
	pI2CHandle -> pI2Cx -> I2C_CCR |= tempreg;


	/************ Step5: Configure the Rise time i.e I2C_TRISE Register******************/
	/* Must be configured when the peripheral is disable*/
	//to store the rise time for both standard and fast mode

	uint8_t Trise = 0;

	//if mode is standard mode that means the Trise max = 1000ns
	//taken from i2c bus specification manual.
	if(pI2CHandle -> I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_ST_MODE)
	{

		//(1000 * 10 ^ (-9)) / (TPCLK1) -> FPCLK1 * 10^(-6) -> FPCLK1 / (10^(6))
		//Here 1000 * 10 ^(-9) -> Trise time
		Trise = (RCC_GetPCLK1Value() / 1000000U) + 1;

	}
	//if mode is fast mode then rise time = 300ns
	else
	{
		//(300 * 10 ^ (-9)) / (TPCLK1) -> FPCLK1 * 300 * 10^(-9) -> FPCLK1 / ( 300 * (10^(9)) )
		//Here 300 * 10 ^(-9) -> Trise time
		Trise = ( (RCC_GetPCLK1Value() * 300) / 1000000000U ) + 1;
	}
    //put the value in I2C_TRISE Register.
	pI2CHandle -> pI2Cx ->I2C_TRISE |= (Trise & 0x3F);//masking i.e filtering  the 5 lowest bits only.

	//After that enable the peripheral
	//After that set the start bit to enter into master mode else enter into slave mode
}




/* *********************************Documentation Section **********************************************
 * @fn                           : I2C_DeInit
 * @brief                        : Function to deinitialize /reset peripheral of I2C.
 * @param[in]                    : Base address of the I2C Handle structure .
 * @return                       : None
 * Special Note                  : None
 */
void I2C_DeInit(I2C_Handle_t *pI2CHandle) //Deinitialize the I2C
{
	if(pI2CHandle -> pI2Cx == I2C1)
		I2C1_REG_RESET();
	else if(pI2CHandle -> pI2Cx == I2C2)
		I2C2_REG_RESET();
	else if(pI2CHandle -> pI2Cx == I2C3)
		I2C3_REG_RESET();

}

/* *********************************Documentation Section **********************************************
 * @fn                           : I2C_GenerateStartCondition
 * @brief                        : Function to generate repeated start condition I2C.
 * @param[in]                    : Base address of the I2C Register structure .
 * @return                       : None
 * Special Note                  : This is private to this file.
 */

void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx -> I2C_CR[0] |= ( 1 << I2C_CR1_START);//set the START Bit of I2C_CR1 to activate repeated start.
}

/* *********************************Documentation Section **********************************************
 * @fn                           : I2C_ExecuteAddressPhase
 * @brief                        : Function to send 7 bit slave address + read/write bit.
 * @param[in1]                   : Base address of the I2C Register structure .
 * @param[in2]                   : Read or Write bit
 * @return                       : None
 * Special Note                  : Here the slave address is first left shfited to add the R/W bit
 */
static void I2C_ExecuteAddressPhase(I2C_RegDef_t  *pI2Cx,uint8_t SlaveAddr,uint8_t R_W_Bit)
{
	//SlaveAddr = 0x27;
	SlaveAddr <<= 1;//make space for r/w bit in LSB position.

	//if the Master wants to write/Transmit Data then put the LSB = 0
	if(R_W_Bit == 0)
		//clear the 0th bit
		SlaveAddr &= ~(1);
	//else set the last bit to read/receive data by master
	else
		SlaveAddr |= (1);//slave address is slaveaddr+r/w(1)


	//put the value in DR Register
	pI2Cx ->I2C_DR = SlaveAddr;
}

/* *********************************Documentation Section **********************************************
 * @fn                           : I2C_ClearADDRFlagInterrupt
 * @brief                        : Function to clear ADDR flag after address phase is done
 * @param[in]                    : Base address of the I2C Handle structure .
 * @return                       : None
 * Special Note                  : Done in interrupt mode.
 */
static void I2C_ClearADDRFlagInterrupt(I2C_Handle_t *pI2CHandle)
{
	//check the device is in master mode
	if(pI2CHandle -> pI2Cx -> I2C_SR[1] & (1 << I2C_SR2_MSL))
	{
		//checking the reception busy state for received data
		if(pI2CHandle -> TxRxState == I2C_BUSY_IN_RX)
		{
			//if length of received data is 1
			if(pI2CHandle -> RxLen == 1)
			{
				//1. first disable the ACK
				(pI2CHandle -> pI2Cx ->I2C_CR[0]) &= ~(1 << I2C_CR1_ACK);

				//2. Clear ADDR flag
				//read the dummy data from I2C_SR1
				uint16_t dummyRead = pI2CHandle -> pI2Cx -> I2C_SR[0];//read SR1 register
				dummyRead = pI2CHandle -> pI2Cx -> I2C_SR[1];//read SR2 Register
				(void )dummyRead;//avoid warning of unused variable
			}

			//check the transmitted data
			else
			{
				//Clear ADDR flag
				//read the dummy data from I2C_SR1
				uint16_t dummyRead = pI2CHandle -> pI2Cx -> I2C_SR[0];//read SR1 register
				dummyRead = pI2CHandle -> pI2Cx -> I2C_SR[1];//read SR2 Register
				(void )dummyRead;//avoid warning of unused variable
			}
		}


	}

	//check the device is in slave mode
	else
	{
		//Clear ADDR flag
		//read the dummy data from I2C_SR1
		uint16_t dummyRead = pI2CHandle -> pI2Cx -> I2C_SR[0];//read SR1 register
		dummyRead = pI2CHandle -> pI2Cx -> I2C_SR[1];//read SR2 Register
		(void )dummyRead;//avoid warning of unused variable
	}


}

/* *********************************Documentation Section **********************************************
 * @fn                           : I2C_ClearADDRFlag
 * @brief                        : Function to clear ADDR flag after address phase is done
 * @param[in]                    : Base address of the I2C register structure .
 * @return                       : None
 * Special Note                  : None
 */
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2C)
{
	//Clear ADDR flag
	//read the dummy data from I2C_SR1
	uint16_t dummyRead = pI2C -> I2C_SR[0];//read SR1 register
	dummyRead = pI2C -> I2C_SR[1];//read SR2 Register
	(void )dummyRead;//avoid warning of unused variable
}


/* *********************************Documentation Section **********************************************
 * @fn                           : I2C_GenerateStopCondition
 * @brief                        : Generate the stop condition by setting the STOP bit.
 * @param[in]                    : Base address of the I2C register structure .
 * @return                       : None
 * Special Note                  : None
 */
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx -> I2C_CR[0] |= ( 1 << I2C_CR1_STOP);//set the STOP Bit of I2C_SR1 Register.
}


/* *********************************Documentation Section **********************************************
 * @fn                           : I2C_GetFlagStatus
 * @brief                        : Function to Read the bit of I2C status Register i.e either I2C_SR1 or I2C_SR2
 * @param[in]                    : Base address of the I2C Handle structure .
 *  @param[in]                   : Flag of status register.
 * @return                       : Status of bit , 0 -> reset, 1 -> set
 * Special Note                  : None
 */

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint32_t FlagName)
{
	/*test the FlagName  in I2C_SR1 Register.
	If the value is set return 1(FLAG_SET) */
	if((pI2Cx -> I2C_SR[0]) & FlagName)
		return I2C_FLAG_SET;
	   //else return 0(FLAG_RESET)
	return I2C_FLAG_RESET;
}



/* *********************************Documentation Section **********************************************
 * @fn                           : I2C_MasterSendData
 * @brief                        : Function to send the data to slave byte by byte.
 * @param[in1]                   : Base address of the I2C Handle structure .
 * @param[in2]                   : Pointer to user buffer.
 * @param[in3]                   : length of user buffer.
 * @param[in4]                   : 7 bit slave address.
 * @param[in5]                   : Control the stop condition generation.
 * @return                       : None
 * Special Note                  : check Fig276 in RM0390 on Page 768
 */
Error_Status I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,uint32_t len,uint8_t SlaveAddr ,uint8_t RepeatStartConditionCtrl)
{
	uint32_t timeout = I2C_MASTER_TIMOUT;

	//Create Variable of enum type
	Error_Status I2CErrorStatus =0;

	//step1. Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle -> pI2Cx);

	/******** After Step 2 and step 3 the SB bit is cleared *******/
	//step 2. Confirm that start condition is generated or not by checking the SB bit of I2C_SR1 Register.
	//SCL will be low until SB = 1 condition isn't met .
	//SB bit is 1 when start condition is generated successfully.
	while(!I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_SB))
	{
		//keep on waiting if timeout occurs i.e reach to 0 then ON the timeout LED
		if(!(--timeout) )
		{
			Error_Handler(STATUS_TIMEOUT);
			I2CErrorStatus = STATUS_TIMEOUT;
			return I2CErrorStatus;
		}
	}

	//Here to clear SB bit Read SR1 register then write into DR register

	//step 3: send address of slave with read/write bit = 0 (to write slave address)
	I2C_ExecuteAddressPhase(pI2CHandle -> pI2Cx,SlaveAddr,I2C_MASTER_TRANSMITTER);

    /********************************** SB Bit in I2C_SR1 Register is cleared ************************/

	timeout = I2C_MASTER_TIMOUT;

	//step 4: check the ADDR bit in I2C_SR1 Register to confirm address phase completed or not.
	//i.e address has been successfully transmitted.
	while(!I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_ADDR)  )//wait until it return 1
	{
		//keep on waiting if timeout occurs i.e reach to 0  then ON the error LED i.e peripheral isn't connected
		if(!(--timeout) )
		{
			Error_Handler(STATUS_ERROR);
			I2CErrorStatus = STATUS_ERROR;
			return I2CErrorStatus;

		}

	}

	//to clear the ADDR flag read SR1 register followed by read to SR2 Register

	//step 5: clear the ADDR bit in I2C_SR1 Register
	//until this is cleared SCL will be low.
	I2C_ClearADDRFlag(pI2CHandle -> pI2Cx );
   /************************** After this the ADDR bit is cleared i.e
    * address has been transferred successfully*********************/

	/*Now master can decide whether to enter into transmitter or receiver
	 mode (i.e to transmit or receive data from slave)
	 and depend on LSB bit in slave address sent */

    //step 6: Send the data 1 byte at a time to DR register.
     while(len > 0)
     {
    	 timeout = I2C_MASTER_TIMOUT;

    	 //TXE = 1 indicate the shift and DR register is empty so we can write data.
    	 while( !I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_TXE))//wait until TXE becomes zero
    	 {
    		//keep on waiting if timeout occurs i.e reach to 0 then ON the timeout LED
			if(!(--timeout) )
			{
				Error_Handler(STATUS_TIMEOUT);
				I2CErrorStatus = STATUS_TIMEOUT;
				return I2CErrorStatus;

			}
    	 }
    	 //putting the data from user pointer in I2C_DR Register with 1 byte at a time.
    	 //for second, third .... byte DR becomes empty but shift register isn't empty and at that time
    	 //writing of data in DR can be done.
    	 pI2CHandle -> pI2Cx ->I2C_DR =  *pTxBuffer;
    	 pTxBuffer++;//to point to next byte data.
    	 len--;
     }

     //step 7: Wait until BTF and TXE is set in I2C_SR1 Register.
	 //when both BTF and TXE is set i.e both SR and DR is empty so next transmission should begin.
	 //when BTF = 1 SCL clock is stretched.
     /*
     If the TXE = 1 and data isn't written before the last data byte data
     transmission then BTF is set and it's cleared by Writing Data to DR register
     and until then SCL clock is stretch to low.
	`*/
     timeout = I2C_MASTER_TIMOUT;
	 while( !I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_TXE))//wait until TXE is set
	 {
	    //keep on waiting if timeout occurs i.e reach to 0 then ON the timeout LED
		if(!(--timeout) )
		{
			Error_Handler(STATUS_TIMEOUT);
			I2CErrorStatus = STATUS_TIMEOUT;
			return I2CErrorStatus;

		}
	 }

     timeout = I2C_MASTER_TIMOUT;
     while(!I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_BTF))//wait until BTF is set.
     {
    	 //keep on waiting if timeout occurs i.e reach to 0 then ON the timeout LED
    	 if(!(--timeout) )
    	 {
    		 Error_Handler(STATUS_TIMEOUT);
    		 I2CErrorStatus = STATUS_TIMEOUT;
    		 return I2CErrorStatus;

    	 }
     }


	 //step 8: Generate stop condition.
	 //clear the BTF flag automatically
     if(RepeatStartConditionCtrl != REPEATE_START_CONDITION_DISABLE)
     {
    	 I2C_GenerateStopCondition(pI2CHandle -> pI2Cx);
     }

     I2CErrorStatus = STATUS_OK;

	 return I2CErrorStatus;
}

/* *********************************Documentation Section **********************************************
 * @fn                           : I2C_MasterReceiveData
 * @brief                        : Function to receive the data via the I2C.
 * @param[in]                    : Base address of the I2C Handle structure .
 *@param[in]                     : Pointer to RxBuffer to store the data received
 *@param[in]                     : No of bytes of user buffer.
 *@param[in]                     : SlaveAddress.
 * @return                       : None
 * Special Note                  : Received Data stored in pRxBuffer
 */
Error_Status I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,uint32_t maxlen,uint8_t SlaveAddr)
{
	//timeout value when status bit checking isn't done
	uint32_t timeout = I2C_MASTER_TIMOUT;

	//Create Variable of enum type
	Error_Status I2CErrorStatus =0;

	//step1. Generate the start condition
	//api private to driver.
	I2C_GenerateStartCondition(pI2CHandle -> pI2Cx);

	/******** After Step 2 and step 3 the SB bit is cleared *******/
		//step 2. Confirm that start condition is generated or not by checking the SB bit of I2C_SR1 Register.
		//SCL will be low until SB = 1 condition isn't met .
		//SB bit is 1 when start condition is generated successfully.
		while(!I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_SB))
		{
			//keep on waiting if timeout occurs i.e reach to 0 then ON the timeout LED
			 if(!(--timeout) )
			 {
				 Error_Handler(STATUS_TIMEOUT);
				 I2CErrorStatus = STATUS_TIMEOUT;
				 return I2CErrorStatus;

			 }
		}

		//step 3: send address of slave with read/write bit = 0 (to write)
		I2C_ExecuteAddressPhase(pI2CHandle -> pI2Cx,SlaveAddr,I2C_MASTER_RECEIVER);

	/********************************** SB Bit in I2C_SR1 Register is cleared ************************/

		timeout = I2C_MASTER_TIMOUT;//Init again
	//step 4: wait until the ADDR bit in I2C_SR1 Register to confirm address phase completed or not.
	while(!I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_ADDR)  )//wait until it return 1
	{
		//keep on waiting if timeout occurs i.e reach to 0 then ON the timeout LED
		 if(!(--timeout) )
		 {
			 Error_Handler(STATUS_TIMEOUT);
			 I2CErrorStatus = STATUS_TIMEOUT;
			 return I2CErrorStatus;

		 }
	}
	//step 5 : Read the data

			//for receiving the ACK must be disable first before clearing ADDR flag.
	    //and after that upon receiving data stop condition is generated
		//step 5a: Reading 1 byte of data
		if(maxlen == 1)
		{
			//Step 5a1 : disable the ACK
	    	(pI2CHandle -> pI2Cx ->I2C_CR[0]) &= ~(1 << I2C_CR1_ACK);

	    	//Step 5a2 : Clear the ADDR flag(as seen in timing diagram)
	    	I2C_ClearADDRFlag(pI2CHandle -> pI2Cx );

	    	//Step 5a3 : Generate stop condition
	    	I2C_GenerateStopCondition(pI2CHandle -> pI2Cx);



	    	timeout = I2C_MASTER_TIMOUT;

	    	//Step 5a4 : Wait until RxNE becomes 1 so as to confirm data has been received or not.
	    	while( !I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_RxNE))
	    	{
	    		//keep on waiting if timeout occurs i.e reach to 0 then ON the timeout LED
				 if(!(--timeout) )
				 {
					 Error_Handler(STATUS_TIMEOUT);
					 I2CErrorStatus = STATUS_TIMEOUT;
					 return I2CErrorStatus;

				 }
	    	}



	    	//clearing the RXNE flag by reading DR register.
	    	//Step 5a5 : Read the 1 byte data
			 //putting the data from user pointer in I2C_DR Register
	    	 *pRxBuffer = pI2CHandle -> pI2Cx ->I2C_DR ;

		}

		/*
		//Step 5b: Read 2 byte data
		else if(maxlen == 2)
		{
			//set ACK low
			(pI2CHandle -> pI2Cx ->I2C_CR[0]) &= ~(1 << I2C_CR1_ACK);

			//clear ADDR flag
			I2C_ClearADDRFlag(pI2CHandle -> pI2Cx );

			//wait until the BTF  = 1
			//Here DATA1 is in DR and data2 is in shift register
			while(!I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_BTF));

			//generate the stop condition
			I2C_GenerateStopCondition(pI2CHandle -> pI2Cx);

			//Read Data1 and DAta2
			*pRxBuffer++ = pI2CHandle-> pI2Cx-> I2C_DR;
			*pRxBuffer = pI2CHandle-> pI2Cx-> I2C_DR;

		}

		else
		{

		}

		*/
		//Step 5b: Read >1 byte of data
		 /*Note : To generate the NACK pulse after the last received data byte, the ACK bit
		              must be cleared just after reading the second last data byte
		*/


		if(maxlen > 1)
		{
			//Step 5b1 : clear the ADDR flag for EV6 event
			I2C_ClearADDRFlag(pI2CHandle -> pI2Cx );

			//Step 5b1 : disable the ACK
			//(pI2CHandle -> pI2Cx ->I2C_CR[0]) &= ~(1 << I2C_CR1_ACK);



			//Step 5b2 : Read the data until the len becomes 0
			for(uint32_t i = maxlen;i>0;i--)
			{
				timeout = I2C_MASTER_TIMOUT;

				//Step 5b3 : Wait until the RxNE becomes 1
				while( !I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_RxNE))
				{
					//keep on waiting if timeout occurs i.e reach to 0 then ON the timeout LED
					 if(!(--timeout) )
					 {
						 Error_Handler(STATUS_TIMEOUT);
						 I2CErrorStatus = STATUS_TIMEOUT;
						 return I2CErrorStatus;

					 }
				}

				//Step 5b4 : if last 2 bytes are remaining
				if(i == 2)
				{
					//clear the ACK bit
					(pI2CHandle -> pI2Cx ->I2C_CR[0]) &= ~(1 << I2C_CR1_ACK);

					//generate STOP condition
					I2C_GenerateStopCondition(pI2CHandle -> pI2Cx);


				}

				//Step 5b5 : Read the data from data register into user buffer
				//           To clear the RXNE flag i.e for EV7 event
				*pRxBuffer = pI2CHandle -> pI2Cx ->I2C_DR ;

				//Step 5b6 : Increment the user buffer address
				pRxBuffer++;
			}
		}

		//re-enable the ACK if user specified it for first time
		if(pI2CHandle -> I2C_Config.I2C_ACKCtrl == I2C_ACK_EN)
			(pI2CHandle -> pI2Cx ->I2C_CR[0]) |= (1 << I2C_CR1_ACK);


		return STATUS_OK;


}


/****************************** INterrupt Related API's *******************/
/* *********************************Documentation Section **********************************************
 * @fn                           : I2C_IRQInterruptConfig
 * @brief                        :
 * @param[in]                    :
 *@param[in]                     :
 * @return                       : None
 * Special Note                  : None
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENOrDI)//to configure the Interrupt i.e IRQ no of I2C
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
 * @fn                           : I2C_IRQPriorityConfig
 * @brief                        :
 * @param[in]                    :
 *@param[in]                     :
 * @return                       : None
 * Special Note                  : None
 */

void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)//to configure the priority of I2C
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
 * @fn                           : I2C_PeripheralControl
 * @brief                        : Function to enable the I2C Peripheral.
 * @param[in]                    : Base address of the I2C Register structure .
 *@param[in]                     : Enable or Disable .
 * @return                       : None
 * Special Note                  : None
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)//enable the I2C Peripheral
	{
		pI2Cx -> I2C_CR[0] |= (1<<I2C_CR1_PE);//enable the PE bit of I2C_CR1 Register

	}
		else//disable the SPI Peripheral.
			pI2Cx -> I2C_CR[0] &= ~(1<<I2C_CR1_PE);//disables the I2C bit of I2C_CR1 Register
}



/* *********************************Documentation Section **********************************************
 * @fn                           : I2C_MasterSendData_Interrupt
 * @brief                        : Send the data to slave via the interrupt
 * @param[in]                    : Address of handle structure.
 * @param[in]                    :
 * @param[in]                    :
 * @param[in]                    :
 * @return                       : None
 * Special Note                  : None
 */
uint8_t I2C_MasterSendData_Interrupt(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,uint32_t len,uint8_t SlaveAddr,uint8_t SrValue)
{
	//Getting the state whether it is transmitting or receiving.
	uint8_t busystate = pI2CHandle->TxRxState;

		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHandle->pTxBuffer = pTxBuffer;
			pI2CHandle->TxLen = len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_TX ;
			pI2CHandle->DevAddr = SlaveAddr;
			pI2CHandle->SrValue = SrValue;

			/*************** First generate the start condition and then enable
			 *  the bit to enable the interrupt for event and error.
			 */
			//Implement code to Generate START Condition
			I2C_GenerateStartCondition(pI2CHandle -> pI2Cx);

			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->I2C_CR[1] |= ( 1 << I2C_CR2_ITBUFEN);

			//Implement the code to enable ITEVTEN Control Bit
			pI2CHandle->pI2Cx->I2C_CR[1] |= ( 1 << I2C_CR2_ITEVTEN);

			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->I2C_CR[1] |= ( 1 << I2C_CR2_ITERREN);

		}

		return busystate;


}


/* *********************************Documentation Section **********************************************
 * @fn                           : I2C_MasterReceiveData_Interrupt
 * @brief                        :
 * @param[in]                    :
 *@param[in]
 *@param[in]
 *@param[in]                     :
 * @return                       : None
 * Special Note                  : None
 */

uint8_t I2C_MasterReceiveData_Interrupt(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,uint32_t len,uint8_t SlaveAddr,uint8_t SrValue)
{
	//Getting the state whether it is transmitting or receiving.
	uint8_t busystate = pI2CHandle->TxRxState;

		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHandle->pTxBuffer = pRxBuffer;
			pI2CHandle->TxLen = len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_RX ;
			pI2CHandle->DevAddr = SlaveAddr;
			pI2CHandle->SrValue = SrValue;

			//Implement code to Generate START Condition
			I2C_GenerateStartCondition(pI2CHandle -> pI2Cx);

			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->I2C_CR[1] |= ( 1 << I2C_CR2_ITBUFEN);

			//Implement the code to enable ITEVTEN Control Bit
			pI2CHandle->pI2Cx->I2C_CR[1] |= ( 1 << I2C_CR2_ITEVTEN);

			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->I2C_CR[1] |= ( 1 << I2C_CR2_ITERREN);

		}

		return busystate;

}

/* *********************************Documentation Section **********************************************
 * @fn                           :
 * @brief                        :
 * @param[in]                    :
 * @return                       : None
 * Special Note                  :
 */
static void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//disable the Tx and Rx interrupts events
	pI2CHandle -> pI2Cx -> I2C_CR[1] &= ~( 1 << I2C_CR2_ITBUFEN);

	//disable the event interrupt
	pI2CHandle -> pI2Cx -> I2C_CR[1] &=  ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle -> TxLen = 0;
	pI2CHandle -> pTxBuffer = NULL;
	pI2CHandle -> TxRxState = I2C_READY;

}

/* *********************************Documentation Section **********************************************
 * @fn                           :
 * @brief                        :
 * @param[in]                    :
 * @return                       : None
 * Special Note                  :
 */

static void I2C_CloseReceivedData( I2C_Handle_t *pI2CHandle)
{
	//disable the Tx and Rx interrupts events
	pI2CHandle -> pI2Cx -> I2C_CR[1] &= ~( 1 << I2C_CR2_ITBUFEN);

	//disable the event interrupt
	pI2CHandle -> pI2Cx -> I2C_CR[1] &=  ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle -> RxLen = 0;
	pI2CHandle ->TxRxState = I2C_READY;//change the state to ready state
	pI2CHandle -> pRxBuffer = NULL;//buffer is NULL
	pI2CHandle -> RxSize = 0;
	//enable the ACK if ACK is enabled
	if(pI2CHandle ->pI2Cx -> I2C_CR[0] & (1 << I2C_CR1_ACK))
		pI2CHandle ->pI2Cx -> I2C_CR[0] |= (1 << I2C_CR1_ACK);
}


__attribute__((weak))void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{

}

/* *********************************Documentation Section **********************************************
 * @fn                           :
 * @brief                        :
 * @param[in]                    :
 * @return                       : None
 * Special Note                  :
 */

void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	//if 1 byte of data is there to be received
		if(pI2CHandle -> RxSize == 1)
		{
			//read the DR and store in RxBuffer
			*(pI2CHandle -> pRxBuffer) = pI2CHandle -> pI2Cx ->I2C_DR;
			//decrement length
			pI2CHandle -> RxLen--;

		}

		//if Received data length > 1
		else if(pI2CHandle -> RxSize > 1)
		{
			if(pI2CHandle ->RxLen == 2)
			{
				//clear ACK bit
				pI2CHandle -> pI2Cx -> I2C_CR[0] &= (1 << I2C_CR1_ACK);
			}

			//read DR
			*(pI2CHandle -> pRxBuffer) = pI2CHandle -> pI2Cx ->I2C_DR;
			pI2CHandle -> pRxBuffer++;//increment the pointer.
			//decrement length
			pI2CHandle -> RxLen--;
		}

		//if all data has been received
		else if(pI2CHandle -> RxSize   == 0)
		{
			//close the Rx reception and notfiy application

			//1. Generate STOP condition
			I2C_GenerateStopCondition(pI2CHandle -> pI2Cx);

			//2. Close the I2C Rx
			I2C_CloseReceivedData(pI2CHandle);

			//3. Notify the application
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_RX_COMPLETE);
		}


}

/* *********************************Documentation Section **********************************************
 * @fn                           : I2C_MasterHandleTXEInterrupt
 * @brief                        : Helper function to handle the TX interrupt
 * @param[in]                    :
 * @return                       : None
 * Special Note                  :
 */
void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	//TxE bit is set i.e DR register is empty then transmit the data
	if(pI2CHandle -> TxRxState == I2C_BUSY_IN_TX)
	{
		//if data is there to be sent
		if(pI2CHandle -> TxLen > 0)
		{
			//1. Load the data in DR from TxBuffer
			pI2CHandle ->pI2Cx ->I2C_DR  = *(pI2CHandle -> pTxBuffer);

			//2. Decrement Tx length
			(pI2CHandle -> TxLen)--;

			//3. Increment buffer address to point to next byte.
			(pI2CHandle -> pTxBuffer)++;
		}
	}

}


/* *********************************Documentation Section **********************************************
 * @fn                           : I2C_EVENT_IRQHandling
 * @brief                        : Interrupt handling for both the master and slave mode of device whenever event occurs.
 * @param[in]                    : Address of handle structure.
 * @return                       : None
 * Special Note                  : This is common for both the master and slave mode interrupt handling.
 */
void I2C_EVENT_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handle for various events
	//getting the status of ITEVTEN bit in temp1 to find out whether event interrupt is enabled or not.
	uint32_t temp1 = pI2CHandle -> pI2Cx -> I2C_CR[1] & (1 << I2C_CR2_ITEVTEN);
	/*getting the status of ITBUFEN bit to find out whether interrupt event is generated when tx
	/buffer is empty or Rx buffer is filled with data.*/
	uint32_t temp2 = pI2CHandle -> pI2Cx -> I2C_CR[1] & (1 << I2C_CR2_ITBUFEN);


	//1. Handling interrupt event generated by SB event and done in Master mode only.
	//getting the status of SB bit in temp3
	uint32_t temp3 = pI2CHandle -> pI2Cx -> I2C_SR[0] & (1 << I2C_SR1_SB);
	//if interrupt is enabled and SB value is +ve then SB is set and execute the address phase.
	if(temp1 && temp3 )
	{
		//Interrupt is generated by SB event
		//Generated only in Master mode. For slave mode always 0
		/*Start condition generated and then we handle the address phase
		 i.e Writing the slave address */
		if(pI2CHandle -> TxRxState == I2C_BUSY_IN_TX)
			I2C_ExecuteAddressPhase(pI2CHandle -> pI2Cx, pI2CHandle ->DevAddr,0 );//0 means write data

		//1 means make the master to read the data by first writing the slave address
		else if (pI2CHandle -> TxRxState == I2C_BUSY_IN_RX)
			I2C_ExecuteAddressPhase(pI2CHandle -> pI2Cx, pI2CHandle ->DevAddr,1 );

	}

	//2. Handling interrupt event generated by ADDR event
	//for master mode -> Address is sent.
	//for slave mode -> Address is matched with Own address in I2C_OARx register
	temp3 = pI2CHandle -> pI2Cx -> I2C_SR[0] & (1 << I2C_SR1_ADDR);
	//if interrupt is enabled and ADDR value is +ve then ADDR bit is set
	if(temp1 && temp3 )
	{
		//ADDR bit is set then clear it since clock may be stretch to low.
		I2C_ClearADDRFlagInterrupt(pI2CHandle );



	}


	//3. Handling interrupt event generated by BTF(byte transfer finished) event
	temp3 = pI2CHandle -> pI2Cx -> I2C_SR[0] & (1 << I2C_SR1_BTF);
	//if interrupt is enabled and BTF value is +ve then BTF bit is set
	if(temp1 && temp3 )
	{
		//BTF bit is set
		//check for event i.e Rx or Tx
		if(pI2CHandle -> TxRxState == I2C_BUSY_IN_TX)
		{
			//check the TXE flag i.e Tx buffer is empty
			if(pI2CHandle -> pI2Cx -> I2C_SR[0] & (1 << I2C_SR1_TXE))
			{
				//BTF = 1, TXE = 1 i.e close the data transmission
				//close the communication only when length is 0 i.e all data has been sent.
				if(pI2CHandle -> TxLen == 0)
				{
					//1. Generate the stop condition only if repeated start is disabled
					if(pI2CHandle -> SrValue == I2C_DISABLE_SR)//get macro from @I2C Repeated Start
						I2C_GenerateStopCondition(pI2CHandle -> pI2Cx);

					//2. Reset all member elements of handle structure
					I2C_CloseSendData(pI2CHandle);

					//3. Notify the application for transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_TX_COMPLETE);
				}
			}
		}
		//if state is receive
		else if(pI2CHandle -> TxRxState == I2C_BUSY_IN_RX)
		{
			;
		}
	}


	//4. Handling interrupt event generated by STOPF event
	//applicable only when device is in slave mode.
	temp3 = pI2CHandle -> pI2Cx -> I2C_SR[0] & (1 << I2C_SR1_STOPF);
	//if interrupt is enabled and STOPF value is +ve then STOPF bit is set
	if(temp1 && temp3 )
	{
		//if STOPF bit is set then clear it
		pI2CHandle -> pI2Cx -> I2C_CR[1] |= (0x0000);
		//Handle the STOPF event.
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_STOPF);
	}

	//5. Handling interrupt event generated by TxE event
	//generated when ITEVFEN and ITBUFEN bit are set.
	temp3 = pI2CHandle -> pI2Cx -> I2C_SR[0] & (1 << I2C_SR1_TXE);
	//if interrupt is enabled and TxE value is +ve then TxE bit is set
	if(temp1 && temp2 && temp3 )
	{
		//Handle the TxE event only if device is in master mode
		if(pI2CHandle ->pI2Cx ->I2C_SR[1] & (1 << I2C_SR2_MSL))
		{
			I2C_MasterHandleTXEInterrupt(pI2CHandle);
		}
	}


	//6. Handling interrupt event generated by RxNE event
	//generated when ITEVFEN and ITBUFEN bit are set.
	temp3 = pI2CHandle -> pI2Cx -> I2C_SR[0] & (1 << I2C_SR1_RXNE);
	//if interrupt is enabled and RxNE value is +ve then RxNE bit is set
	if(temp1 && temp2 && temp3 )
	{
		//RxNE bit is set
		//RxNE bit is set i.e DR register is filled with received data.
		/********works only if device is in master mode  */
		if(pI2CHandle -> pI2Cx -> I2C_SR[1] & (1 << I2C_SR2_MSL))
		{
			if(pI2CHandle -> TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
	}


}






/* *********************************Documentation Section **********************************************
 * @fn                           : I2C_ERR_IRQHandling
 * @brief                        :
 * @param[in]                    :
 *@param[in]
 *@param[in]
 *@param[in]                     :
 * @return                       : None
 * Special Note                  : None
 */
void I2C_ERR_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1,temp2;

	    //Know the status of  ITERREN control bit in the CR2
		temp2 = (pI2CHandle->pI2Cx->I2C_CR[1]) & ( 1 << I2C_CR2_ITERREN);


	/***********************Check for Bus error************************************/
		temp1 = (pI2CHandle->pI2Cx->I2C_SR[0]) & ( 1<< I2C_SR1_BERR);
		if(temp1  && temp2 )
		{
			//This is Bus error

			//Implement the code to clear the buss error flag
			pI2CHandle->pI2Cx->I2C_SR[0] &= ~( 1 << I2C_SR1_BERR);

			//Implement the code to notify the application about the error
		   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
		}

	/***********************Check for arbitration lost error************************************/
		temp1 = (pI2CHandle->pI2Cx->I2C_SR[0]) & ( 1 << I2C_SR1_ARLO );
		if(temp1  && temp2)
		{
			//This is arbitration lost error

			//Implement the code to clear the arbitration lost error flag
			pI2CHandle->pI2Cx->I2C_SR[0] &= ~( 1 << I2C_SR1_ARLO);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
		}

	/***********************Check for ACK failure  error************************************/

		temp1 = (pI2CHandle->pI2Cx->I2C_SR[0]) & ( 1 << I2C_SR1_AF);
		if(temp1  && temp2)
		{
			//This is ACK failure error

		    //Implement the code to clear the ACK failure error flag
			pI2CHandle->pI2Cx->I2C_SR[0] &= ~( 1 << I2C_SR1_AF);
			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
		}

	/***********************Check for Overrun/underrun error************************************/
		temp1 = (pI2CHandle->pI2Cx->I2C_SR[0]) & ( 1 << I2C_SR1_OVR);
		if(temp1  && temp2)
		{
			//This is Overrun/underrun


		    //Implement the code to clear the Overrun/underrun error flag
			pI2CHandle->pI2Cx->I2C_SR[0] &= ~( 1 << I2C_SR1_OVR);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
		}

	/***********************Check for Time out error************************************/
		temp1 = (pI2CHandle->pI2Cx->I2C_SR[0]) & ( 1 << I2C_SR1_TIMEOUT);
		if(temp1  && temp2)
		{
			//This is Time out error

		    //Implement the code to clear the Time out error flag
			pI2CHandle->pI2Cx->I2C_SR[0] &= ~( 1 << I2C_SR1_TIMEOUT);
			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
		}
}





