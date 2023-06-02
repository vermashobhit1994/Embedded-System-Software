/*
 * MCP9808_Temp_Sensor.c
 *
 *  Created on: 16-Jul-2021
 *      Author: vermas
 */


/* Description :
 *
 */

#include"MCP9808_Temp_Sensor.h"

/* *********************************Documentation Section **********************************************
 * @fn                           :
 * @brief                        :
 * @param[in1]                   :
 * @param[in2]                   :
 * @return                       :
 * @Note                         :
 */








/* *********************************Documentation Section **********************************************
 * @fn                           : MCP9808_Write_Reg
 * @brief                        : Write 1 byte to Register specified
 * @param[in1]                   :
 * @param[in1]                   : Bit corresponding to register address to be select.
 * @param[in2]                   : Data to be written.
 * @param[in3]                   :
 * @return                       :
 * @Note                         : Here higher MSB byte must be send first then lower LSB byte
 */
Error_Status MCP9808_Sensor_Write_Register(I2C_Handle_t *pI2CHandle,  MCP9808_REG_ADDR MCP9808_RegAddr,uint16_t data, uint8_t MCP9808_SlaveAddr)
{
	//to store the data to be sent
	uint8_t temp[3] = {0};

	//since we're writing so slave address lsb = 0
	//uint8_t SlaveAddr_Write = (uint8_t)(MCP9808_SlaveAddr << 1) & ~(1 << 0);


	//switch to select the address for register on which to be written.
	//this is done by register pointer
	//Here the MSB byte send first then LSB byte
	switch (MCP9808_RegAddr)
	{
		case MCP9808_REG_ADDR_CONFIG:
			temp[0] = MCP9808_REG_ADDR_CONFIG; //Send the register address first
			temp[1] = (data >> 8) & 0x7;//extract bits [10:8] and put in [0:2]
			temp[2] = (data ) & 0xFF;//extract bit[7:0]

			//Here stop condition generation when sending the data
			//since we're sending the data continuously without stop condition.
			if ( I2C_MasterSendData(pI2CHandle, temp,3 , MCP9808_SlaveAddr, REPEATE_START_CONDITION_DISABLE) != STATUS_OK)
					return STATUS_ERROR;
			break;

			//Here we write data of temperature in *C (degree centigrate)
		case MCP9808_REG_ADDR_ALERT_TEMP_UPPER:
			temp[0] = MCP9808_REG_ADDR_ALERT_TEMP_UPPER;
			temp[1] = (data >> 8) & 0x1F;
			temp[2] = (data ) & 0xFC;
			if ( I2C_MasterSendData(pI2CHandle, temp,3 , MCP9808_SlaveAddr,REPEATE_START_CONDITION_DISABLE) != STATUS_OK)
				return STATUS_ERROR;
		    break;

		case MCP9808_REG_ADDR_ALERT_TEMP_LOWER:
			temp[0] = MCP9808_REG_ADDR_ALERT_TEMP_LOWER;

			temp[1] = (data >> 8) & 0x1F;
			temp[2] = (data ) & 0xFC;
			if ( I2C_MasterSendData(pI2CHandle, temp,3 ,MCP9808_SlaveAddr , REPEATE_START_CONDITION_DISABLE) != STATUS_OK)
				return STATUS_ERROR;
			break;

		case MCP9808_REG_ADDR_CRITICAL_TEMP_TRIP:
			temp[0] = MCP9808_REG_ADDR_CRITICAL_TEMP_TRIP;
			temp[1] = (data >> 8) & 0x1F;
			temp[2] = (data) & 0xFC;
			if ( I2C_MasterSendData(pI2CHandle, temp,3 , MCP9808_SlaveAddr, REPEATE_START_CONDITION_DISABLE) != STATUS_OK)
				return STATUS_ERROR;

			break;

		/* Here isn't implemented since read only
		case MCP9808_REG_AMBIENT_TEMP:
			temp[0] = MCP9808_REG_AMBIENT_TEMP;
			break;

		case MCP9808_REG_MANUFACTURE_ID:
			temp[0] = MCP9808_REG_MANUFACTURE_ID;
			break;
		case MCP9808_REG_DEVICE_ID:
			temp[0] = MCP9808_REG_DEVICE_ID;
			break;
		*/
		case MCP9808_REG_ADDR_RESOLUTION:
			temp[0] = MCP9808_REG_ADDR_RESOLUTION;//write the register address
			temp[1] = data & 0x3;//extract bits [1:0]
			if ( I2C_MasterSendData(pI2CHandle, temp,2 , MCP9808_SlaveAddr, REPEATE_START_CONDITION_DISABLE) != STATUS_OK)
				return STATUS_ERROR;
			break;

		//if wrong register is selected i.e writing to read only register
			//then turn ON error led
		default:
			Error_Handler(STATUS_ERROR);
			return STATUS_ERROR;

	}

	//Here the data has been already written in register structure
	//check the size of register

	return STATUS_OK;

}

/* *********************************Documentation Section **********************************************
 * @fn                           :MCP9808_Read_Reg
 * @brief                        : Read the register value and store in buffer
 * @param[in1]                   : Pointer to data to be write
 * @param[in2]                   : Register address bit for selected register.
 * @param[in3]                   : Pointer to register to store data
 * @return                       : Error status
 * @Note                         : Here the MSB byte read first then the LSB byte
 *                                 Here pRxBuffer[0] -> store the MSB byte
 *                                 Here pRxBuffer[1] -> store the LSB byte
 *
 */
Error_Status MCP9808_Read_Reg(I2C_Handle_t *pI2CHandle, MCP9808_REG_ADDR RegAddr, uint8_t *pRxBuffer, uint8_t MCP9808_SlaveAddr)
{
	//since we're writing so slave address lsb = 0

	//to store the register address
	uint8_t MCP9808_RegAddr = 0;




	switch(RegAddr)
	{
		case  MCP9808_REG_ADDR_CONFIG:
			MCP9808_RegAddr = MCP9808_REG_ADDR_CONFIG;
			break;
		case MCP9808_REG_ADDR_ALERT_TEMP_UPPER:
			MCP9808_RegAddr = MCP9808_REG_ADDR_ALERT_TEMP_UPPER;
			break;
		case MCP9808_REG_ADDR_ALERT_TEMP_LOWER:
			MCP9808_RegAddr = MCP9808_REG_ADDR_ALERT_TEMP_LOWER;
			break;
		case MCP9808_REG_ADDR_CRITICAL_TEMP_TRIP:
			MCP9808_RegAddr = MCP9808_REG_ADDR_CRITICAL_TEMP_TRIP;
			break;
		case MCP9808_REG_ADDR_AMBIENT_TEMP:
			MCP9808_RegAddr = MCP9808_REG_ADDR_AMBIENT_TEMP;
			break;
		case MCP9808_REG_ADDR_MANUFACTURE_ID:
			MCP9808_RegAddr = MCP9808_REG_ADDR_MANUFACTURE_ID;
			break;
		case MCP9808_REG_ADDR_DEVICE_ID:
			MCP9808_RegAddr = MCP9808_REG_ADDR_DEVICE_ID;
			break;
		case MCP9808_REG_ADDR_RESOLUTION:
			MCP9808_RegAddr = MCP9808_REG_ADDR_RESOLUTION;
			break;
		default:
			Error_Handler(STATUS_ERROR);
			return STATUS_ERROR;

	}

	//sending the slave address along with address for register to be select
	//Here no stop condition generation
	if ( I2C_MasterSendData(pI2CHandle, &MCP9808_RegAddr, 1, MCP9808_SlaveAddr, REPEATE_START_CONDITION_ENABLE) != STATUS_OK)
		return STATUS_ERROR;


	//get the data using the repeated start condition
	//first need to activate repeated start condition while receiving data
	//Here we're setting the LSB bit to 1
	//if ( I2C_MasterSendData(pI2CHandle, &temp[0],1 , SlaveAddr_Read) != STATUS_OK)
	//	return STATUS_ERROR;


	//
	//Here first the slave address is written and then the 2 bytes is received

	//receive 2nd byte only if register is of 16 bit i.e the register is
	//other than resolution register
	if (RegAddr != MCP9808_REG_ADDR_RESOLUTION)
	{
		if ( I2C_MasterReceiveData(pI2CHandle, pRxBuffer, 2, MCP9808_SlaveAddr) != STATUS_OK)
				return STATUS_ERROR;

	}
	else
	{
		if ( I2C_MasterReceiveData(pI2CHandle, pRxBuffer, 1, MCP9808_SlaveAddr) != STATUS_OK)
						return STATUS_ERROR;
	}






	//generate the stop condition
	I2C_GenerateStopCondition(pI2CHandle -> pI2Cx);

	return STATUS_OK;

}

/* *********************************Documentation Section **********************************************
 * @fn                           : MCP9808_Check_Temp_Range
 * @brief                        : Function to check if the current temperature is in lower
 *                                 and higher limits
 * @param[in1]                   :
 * @param[in2]                   :
 * @return                       :
 * @Note                         : Here we're checking if the temperature goes out of range or is in range
 *                                 If the temperature goes out of range then turn ON buzzer
 */
void MCP9808_Check_Temp_Range(uint16_t Temperature_Data_Buffer)
{





}



/* *********************************Documentation Section **********************************************
 * @fn                           :
 * @brief                        :
 * @param[in1]                   :
 * @param[in2]                   :
 * @return                       :
 * @Note                         : Return Float negative value out of range
 *                                 Private function
 */

static float MCP9808_Calculate_Temperature_DecimalVal(int16_t TemperatureValue, uint8_t TempResolution)
{
	//to store the final decimal value of temperature
	float FinalTempCelsius = 0.0;

	uint8_t TempDecimalVal = (TemperatureValue) & 0x000F;//extract the last 4 bits

	//variable to hold the temperature resolution
	//uint8_t TempResolution = 0;

	//array to hold the temperature resolution values.
	//Here written according to bit position
	float TempResolutionValArray [4] = {0.5, 0.25, 0.125, 0.0625};




	switch(TempResolution)
	{
		//temp resolution = 0.5 *C
		case 0:

			//if bit isn't zero then only put the value
			if( (TempDecimalVal & 0x8U))
				FinalTempCelsius += TempResolutionValArray[0];


			break;

		//temp resolution = 0.25 *C
		case 1 :

			for(uint8_t i = 0;i<2;i++)
			{
				if( TempDecimalVal & (0x8 >> i))
				{
					FinalTempCelsius +=  TempResolutionValArray[i];
				}
			}
			break;
		//temp resolution = 0.125
		case 2 :


			for(uint8_t i = 0;i<3;i++)
			{
				if( TempDecimalVal & (0x8 >> i))
				{
					FinalTempCelsius +=  TempResolutionValArray[i];
				}
			}
			break;

		//temp resolution = 0.0625
		case 3 :

			for(uint8_t i = 0;i<4;i++)
			{
				if( TempDecimalVal & (0x8 >> i))
				{
					FinalTempCelsius +=  TempResolutionValArray[i];
				}
			}
			break;

		default :
			Error_Handler(STATUS_ERROR);
			return FLOAT_MIN;


	}

	//return the final decimal temperature value
	return  FinalTempCelsius;

}


/* *********************************Documentation Section **********************************************
 * @fn                           : MCP9808_Read_TempValue_In_Celsius
 * @brief                        : Function to read the temperature value in *C
 * @param[in1]                   :
 * @param[in2]                   :
 * @return                       : Failure -> 0.0, Success -> Current temperature value
 * @Note                         :
 */
float MCP9808_Read_TempValue_In_Celsius(MCP9808_Param_Config_t *ParamConfig)
{


	//can be negative also
	uint16_t Temperature_Data_Buffer = 0;

	//variable to hold the temperature resolution
	uint8_t TempResolution = 0;

	//Step1 : Read the temperature resolution first
	//checking the resolution of decimal value

	if (MCP9808_Read_Reg(ParamConfig -> pI2CHandle,MCP9808_REG_ADDR_RESOLUTION, (uint8_t*)&TempResolution,ParamConfig -> MCP9808_SlaveAddr ) != STATUS_OK)
	{
			Error_Handler(STATUS_ERROR);
			return FLOAT_MIN;
	}

	switch(TempResolution)
	{
		//when the resolution = 0.5 *C
		case 0:
			delay_ms(30);
			break;
		//when the resolution = 0.25 *C
		case 1:
			delay_ms(65);
			break;

		//when the resolution = 0.125 *C
		case 2:
			delay_ms(130);
			break;

		//when the resolution = 0.0625 *C
		case 3 :
			delay_ms(250);
			break;

		default:
			Error_Handler(STATUS_ERROR);
			return FLOAT_MIN;

	}

	//Step2 : Then read the ambient (surrounding) temperature

	/* here endianess problem arises if use the 16 bit data int type as buffer */
	/* After checking found that it is little endian*/
	//so, the LSB byte contains the temperature MSB byte data of sensor.
	if (MCP9808_Read_Reg(ParamConfig -> pI2CHandle,MCP9808_REG_ADDR_AMBIENT_TEMP, (uint8_t*)&Temperature_Data_Buffer,ParamConfig -> MCP9808_SlaveAddr ) != STATUS_OK)
	{
		//Error_Handler(STATUS_ERROR);
		return FLOAT_MIN;
	}

	//shift the LSB byte and MSB byte data to MSB and lSB position
	Temperature_Data_Buffer = ((Temperature_Data_Buffer & 0xFF) << 8 ) |( (Temperature_Data_Buffer & 0xFF00)>>8);


	//temporary value to store the temperature data (contains both integer and decimal value)
	//uint16_t TemporaryVal = Temperature_Data_Buffer;


	float FinalTempCelsius = 0.0;

	/*********** Calculate the Integer temperature value *****************************/
	uint8_t TemperatureIntMSBByte = 0;
	uint8_t TemperatureIntLSBByte = 0;

	//taken as int since can be negative
	int8_t TemperatureIntVal = 0;



	// check whether the current temperature value is in range specified by user
	//Here we need to check 15:13 bits of T_A Register
	//Here we're turning ON buzzer along with display of temperature on LCD
	MCP9808_Check_Temp_Range(Temperature_Data_Buffer ) ;


	//clear the sign bit if temperature value is in range
	Temperature_Data_Buffer &= ~(1 << MCP9808_REG_BIT_POSITION_SIGN);


	//clear the 15:13 bits if temperature value is in range
	Temperature_Data_Buffer &= ~(0xE000);


	//extract the temperature LSB and MSB byte
	TemperatureIntMSBByte = ((Temperature_Data_Buffer >> 8) & 0xF) << 4 ;
	TemperatureIntLSBByte = (Temperature_Data_Buffer >> 4) & 0xF;

	//if temperature value is negative
	if(Temperature_Data_Buffer & (1 << MCP9808_REG_BIT_POSITION_SIGN  ) )
	{

		//can be positive or negative
		TemperatureIntVal = 256 - TemperatureIntMSBByte  + TemperatureIntLSBByte;

		//temp int value
		FinalTempCelsius += TemperatureIntVal;
	}

	//if temperature value is positive
	else
	{

		//can be positive or negative
		TemperatureIntVal = TemperatureIntMSBByte + TemperatureIntLSBByte;

		//temp int value
		FinalTempCelsius += TemperatureIntVal;
	}

	/*********************************************************************************/


	/**** ############### calculate the decimal value for temperature based on resolution***********/

	FinalTempCelsius += MCP9808_Calculate_Temperature_DecimalVal(Temperature_Data_Buffer,TempResolution);

	//MCP9808_Calculate_Temperature_DecimalVal(Temperature_Data_Buffer,TempResolution);

	/*********************************************************************************/

	return FinalTempCelsius;

}


/* *********************************Documentation Section **********************************************
 * @fn                           :
 * @brief                        :
 * @param[in1]                   :
 * @param[in2]                   :
 * @return                       :
 * @Note                         : Used only when the average of temperature is to be calculated for better accuracy
 */
float MCP9808_Calculate_Avg_Temperature_Value(MCP9808_Param_Config_t *pParamReg, uint32_t No_Of_Samples)
{
	float sum = 0.0;

	//calculate sum of all the temperature values
	for(uint32_t i = 0;i<No_Of_Samples ;i++)
		sum += MCP9808_Read_TempValue_In_Celsius(pParamReg);

	return sum / No_Of_Samples;
}


/* *********************************Documentation Section **********************************************
 * @fn                           :
 * @brief                        : Function to convert the decimal value to
 *                                 data to be written to 16 bit register
 * @param[in1]                   :
 * @param[in2]                   :
 * @return                       :
 * @Note                         :
 */
void MCP9808_Convert_Temperature_To_Register_Data(float Data, uint16_t *pRegisterData)
{

	uint8_t DataDecimalValue = 0;

	//extract the decimal and integer value from float data
	int8_t DataIntValue = (int8_t) Data;


	//remove the negative sign first for extracting the decimal value
	if(Data < 0)
	{

		//extract the decimal value
		DataDecimalValue =  ( (uint8_t)( (Data*-1) * N_DECIMAL_POINTS_PRECISION ) % N_DECIMAL_POINTS_PRECISION);

		//set the sign bit
		*pRegisterData |= ( MCP9808_REG_BIT_POSITION_SIGN);

	}

	//if the value is positive
	else
	{
		//extract the decimal value
		DataDecimalValue =  ( (uint8_t)( (Data) * N_DECIMAL_POINTS_PRECISION ) % N_DECIMAL_POINTS_PRECISION);

		//clear the sign bit
		*pRegisterData &= ~( MCP9808_REG_BIT_POSITION_SIGN);
	}


	//write the decimal data to variable
	*pRegisterData |= (DataDecimalValue & 0x3) <<2;

	//write the integer temperature data
	*pRegisterData |= ( (DataIntValue & 0x0F) << 4) | ( ( (DataIntValue & 0xF0)>>4) << 8);



}


/* *********************************Documentation Section **********************************************
 * @fn                           :
 * @brief                        : Init the parameter as specified by user.
 * @param[in1]                   :
 * @param[in2]                   :
 * @return                       :
 * @Note                         :
 */
float MCP9808_Convert_Celsius_To_Farenheit(float TempCelsius)
{
	float val = (9.00000/5) * TempCelsius + 32.0000;
	return val;
}


/* *********************************Documentation Section **********************************************
 * @fn                           :
 * @brief                        : Init the parameter as specified by user.
 * @param[in1]                   :
 * @param[in2]                   :
 * @return                       :
 * @Note                         :
 */
void MCP9808_Temp_Sensor_Init(MCP9808_Param_Config_t *pParamConfig)
{
	//data to be written to sensor Register
	uint16_t MCP9808_Register_Write_Data = 0;

	//uint8_t SlaveAddr = pParamConfig -> MCP9808_SlaveAddr;

	/*

	//Step1 : Write the lower limit temperature data
	MCP9808_Convert_Temperature_To_Register_Data(pParamConfig -> Lower_Alert_Temp_In_Celsius, &MCP9808_Register_Write_Data);


	if (MCP9808_Sensor_Write_Register(pParamConfig -> pI2CHandle,MCP9808_REG_ADDR_ALERT_TEMP_LOWER,MCP9808_Register_Write_Data, pParamConfig-> MCP9808_SlaveAddr) != STATUS_OK)
	{
		//Error_Handler(STATUS_ERROR);
		return;
	}

	//make the temperature data buffer to zero to prevent overwrite problem.
	MCP9808_Register_Write_Data = 0;

	//Step2 : Write the upper limit temperature data
 	//copy the data from param register to register from where the data is being extracted to be send via I2C 
	MCP9808_Convert_Temperature_To_Register_Data(pParamConfig ->Higher_Alert_Temp_In_Celsius , &MCP9808_Register_Write_Data);

	if (MCP9808_Sensor_Write_Register(pParamConfig -> pI2CHandle,MCP9808_REG_ADDR_ALERT_TEMP_UPPER,MCP9808_Register_Write_Data, pParamConfig-> MCP9808_SlaveAddr) != STATUS_OK)
	{
		//Error_Handler(STATUS_ERROR);
		return;
	}


	//make the temperature data buffer to zero to prevent overwrite problem.
	MCP9808_Register_Write_Data = 0;


	//Step3 : Write the critical temperature data
	MCP9808_Convert_Temperature_To_Register_Data(pParamConfig ->Critical_Temp_In_Celsius , &MCP9808_Register_Write_Data);

	if (MCP9808_Sensor_Write_Register(pParamConfig -> pI2CHandle,MCP9808_REG_ADDR_CRITICAL_TEMP_TRIP,MCP9808_Register_Write_Data, pParamConfig-> MCP9808_SlaveAddr) != STATUS_OK)
	{
		//Error_Handler(STATUS_ERROR);
		return;
	}




	//make the temperature data buffer to zero to prevent overwrite problem.
	MCP9808_Register_Write_Data = 0;



	//Step4 : write the config register data

	//Step4a : Enable /disable the alert status bit
	if(pParamConfig -> AlertOutputCtrl == MCP9808_ALERT_OUTPUT_EN)
		MCP9808_Register_Write_Data |= MCP9808_REG_CONFIG_BIT_POSITION_ALERT_CTRL;
	else
		MCP9808_Register_Write_Data &= ~MCP9808_REG_CONFIG_BIT_POSITION_ALERT_CTRL;

	//Step4b: Set the Alert output polarity
	if(pParamConfig -> AlertOutputPolarity == MCP9808_ALERT_OUTPUT_POLARITY_HIGH)
		MCP9808_Register_Write_Data |= MCP9808_REG_CONFIG_BIT_POSITION_ALERT_POL;
	else
		MCP9808_Register_Write_Data &= ~MCP9808_REG_CONFIG_BIT_POSITION_ALERT_POL;


	//Step4c : Set the Alert output mode
	if(pParamConfig -> AlertOuputMode == MCP9808_ALERT_OUTPUT_MODE_INTERRUPT)
		MCP9808_Register_Write_Data |= MCP9808_REG_CONFIG_BIT_POSITION_ALERT_POL;
	else//comparator mode
		MCP9808_Register_Write_Data &= ~MCP9808_REG_CONFIG_BIT_POSITION_ALERT_POL;


	//Step4d : Set the Alert output select
	if(pParamConfig -> AlertOutputSelect == MCP9808_ALERT_OUTPUT_TUP_TLOW_TCRITICAL)
		MCP9808_Register_Write_Data |= MCP9808_REG_CONFIG_BIT_POSITION_ALERT_SEL;
	else
		MCP9808_Register_Write_Data &= ~MCP9808_REG_CONFIG_BIT_POSITION_ALERT_SEL;


	//Write the value to register
	if (MCP9808_Sensor_Write_Register(pParamConfig ->pI2CHandle, MCP9808_REG_ADDR_CONFIG,MCP9808_Register_Write_Data,pParamConfig -> MCP9808_SlaveAddr   ) != STATUS_OK)
	{
		//Error_Handler(STATUS_ERROR);
		return;
	}



	// changing from decimal to binary

	//Put the temperature value into the register
	//float TempfloatValue = pParamConfig->Lower_Alert_Temp_In_Celsius;
	//(void)TempfloatValue;
	//MCP9808_Extract_Byte();

	//put the higher and lower byte. Done since MSB byte transfer first then LSB
	//pTempReg -> MCP9808_REG_= (TempRegHighByte << 8)|TempRegLowerByte;

	//MCP9808_Write_Reg(preg,RegAddrBits );

	//make the temperature data buffer to zero to prevent overwrite problem.
	MCP9808_Register_Write_Data = 0;

	*/

	//Step5 : Write the resolution as specified by user
	MCP9808_Register_Write_Data = pParamConfig -> TempResolutionVal;
	if (MCP9808_Sensor_Write_Register(pParamConfig ->pI2CHandle, MCP9808_REG_ADDR_RESOLUTION,MCP9808_Register_Write_Data,pParamConfig -> MCP9808_SlaveAddr   ) != STATUS_OK)
	{
		//Error_Handler(STATUS_ERROR);
		return;
	}



}

/************************* Temperature sensor Init **********************************************************/


/* *********************************Documentation Section **********************************************
 * @fn                           :
 * @brief                        :
 * @param[in1]                   :
 * @param[in2]                   :
 * @return                       :
 * @Note                         : PB10 -> I2C2_SCL
 *                                 PB11 -> I2C2_SDA
 */

static void MCP9808_I2C_GPIOPinsConfig(void)
{
	GPIO_Handle_t MCP9808Pins;

	//pins for SCL
	MCP9808Pins.pGPIOx = MCP9808_GPIO_PORT_SCL;
	MCP9808Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;
	MCP9808Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_OPEN_DRAIN;
	MCP9808Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERY_HIGH;
	MCP9808Pins.GPIO_PinConfig.GPIO_PinNumber = MCP9808_GPIO_PIN_SCL;
	MCP9808Pins.GPIO_PinConfig.GPIO_PinAltFunMode = MCP9808_ALT_FUNC_MODE ;

	GPIO_Init(&MCP9808Pins);


	//pins for SDA
	MCP9808Pins.pGPIOx = MCP9808_GPIO_PORT_SDA;
	MCP9808Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;
	MCP9808Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_OPEN_DRAIN;
	MCP9808Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERY_HIGH;
	MCP9808Pins.GPIO_PinConfig.GPIO_PinNumber = MCP9808_GPIO_PIN_SDA;
	MCP9808Pins.GPIO_PinConfig.GPIO_PinAltFunMode = MCP9808_ALT_FUNC_MODE;


	GPIO_Init(&MCP9808Pins);


}

/* *********************************Documentation Section **********************************************
 * @fn                           :
 * @brief                        :
 * @param[in1]                   :
 * @param[in2]                   :
 * @return                       :
 * @Note                         :
 */
I2C_Handle_t* MCP9808_I2C_Config(I2C_Handle_t *pI2CHandle )
{

	MCP9808_I2C_GPIOPinsConfig();

	//Here first Initialize the I2Cx
	pI2CHandle -> pI2Cx = MCP9808_I2Cx  ;
	pI2CHandle -> I2C_Config.I2C_DeviceAddress = MCP9808_SLAVE_ADDR;
	pI2CHandle -> I2C_Config.I2C_FMDUTYCYCL = I2C_FM_DUTY2 ;
	pI2CHandle -> I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_FAST_MODE_400K;
	pI2CHandle -> I2C_Config.I2C_ACKCtrl= I2C_ACK_EN  ;

	I2C_Init(pI2CHandle);

	//Then enable the peripheral
	I2C_PeripheralControl(MCP9808_I2Cx , ENABLE);

	return pI2CHandle;

}



/* *********************************Documentation Section **********************************************
 * @fn                           : MCP9808_Temp_Sensor_Init_Param
 * @brief                        : Initialize the parameters for temperature sensor
 * @param[in1]                   :
 * @param[in2]                   :
 * @return                       :
 * @Note                         :
 */
void MCP9808_Param_Init(void)
{

	//First config the GPIO pins and I2Cx Peripheral
	MCP9808_Param_Config_t *pParamConfig = &Temperature_SensorParam;


	pParamConfig -> pI2CHandle = MCP9808_I2C_Config(&Temperature_Sensor_I2CHandle) ;

	pParamConfig -> Higher_Alert_Temp_In_Celsius = TEMP_HIGH_LIMIT;
	pParamConfig -> Lower_Alert_Temp_In_Celsius = TEMP_LOW_LIMIT;
	pParamConfig -> Critical_Temp_In_Celsius = TEMP_CRITICAL;
	pParamConfig -> MCP9808_SlaveAddr = pParamConfig -> pI2CHandle ->I2C_Config.I2C_DeviceAddress;
	pParamConfig -> TempResolutionVal = TEMP_RESOLUTION ;
	pParamConfig -> AlertOutputCtrl = TEMP_ALERT_O_P_CTRL;
	pParamConfig -> AlertOutputPolarity = TEMP_ALERT_O_P_POLARITY;

	MCP9808_Temp_Sensor_Init(pParamConfig );
}


/*************************************************************************************************************/




/* *********************************Documentation Section **********************************************
 * @fn                           :
 * @brief                        :
 * @param[in1]                   :
 * @param[in2]                   :
 * @return                       :
 * @Note                         :
 */
/*
uint8_t* MCP9808_Send_TempValue(float TempfloatValue)
{
	//Here TempRegByte[0] -> Higherbyte
		//TempRegByte[1] -> Lower Byte
		uint8_t TempRegByte[2] = 0;
		uint8_t TempRegHighByte,TempRegLowerByte;

	uint8_t LowerTempAlertIntValue = (int) TempfloatValue;

		uint8_t LowerTempAlertDecimalValue =  ( (int)(TempfloatValue * N_DECIMAL_POINTS_PRECISION ) % N_DECIMAL_POINTS_PRECISION);

		if(pParamConfig->Lower_Alert_Temp_In_Celsius < 0)
		{


			TempRegHighByte |= (MCP9808_REG_BIT_SIGN >>8 );//put the sign bit
			TempRegHighByte |=  (LowerTempAlertIntValue & 0xF0)>>5;

			TempRegLowerByte |= (LowerTempAlertIntValue << 4) & 0xF0;//put the actual value
			TempRegLowerByte |= (LowerTempAlertDecimalValue <<2 ) & 0x0C;



		}
		else
		{
			TempRegHighByte &= ~(MCP9808_REG_BIT_SIGN >>8 );//put the sign bit
			TempRegHighByte |=  (LowerTempAlertIntValue & 0xF0)>>5;

			TempRegLowerByte |= (LowerTempAlertIntValue << 4) & 0xF0;//put the actual value
			TempRegLowerByte |= (LowerTempAlertDecimalValue <<2 ) & 0x0C;
		}

		TempRegByte[0] = TempRegHighByte;
		TempRegByte[1] = TempRegLowerByte;

		return TempRegByte;


}
*/






