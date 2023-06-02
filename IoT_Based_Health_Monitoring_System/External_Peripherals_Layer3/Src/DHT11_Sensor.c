/*
 * DHT11_Sensor.c
 *
 *  Created on: 04-Jul-2021
 *      Author: vermas
 */

#include"DHT11_Sensor.h"


/* global declarations */


/* *********************************Documentation Section **********************************************
 * @fn                            : DHT11_Sensor_SetPinOutput
 * @brief                         : Set the DHT11 Pin as output
 * @return                        : None
 * @Note                          : By Default Pin is in Input Floating mode.
 */
 void DHT11_Sensor_SetPinOutput(void)
{
	//configure the GPIO Pins and Port
	GPIO_Handle_t dht11pinoutput = {0};//initialize the structure members as 0
	dht11pinoutput.pGPIOx = DHT11_GPIO_PORT;
	dht11pinoutput.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;

	/* Here according to datasheet the pins must be configured in Open Drain in
	 * low or High Z state
	 */
	dht11pinoutput.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_OPEN_DRAIN;

	dht11pinoutput.GPIO_PinConfig.GPIO_PinNumber = DHT11_GPIO_PIN_NO;
	dht11pinoutput.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_PULL_UP;
	dht11pinoutput.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERY_HIGH;
	GPIO_Init(&dht11pinoutput);


}

/* *********************************Documentation Section **********************************************
 * @fn                            : DHT11_Sensor_SetPinInput
 * @brief                         : Set the DHT11 Pin as output
 * @return                        : None
 * @Note                          : By Default Pin is in Input Floating mode.
 *
 */
void DHT11_Sensor_SetPinInput(void)
{
	//configure the GPIO Pins and Port
	GPIO_Handle_t dht11pinInput = {0}; //initialize all memebers as 0
	dht11pinInput.pGPIOx = DHT11_GPIO_PORT;
	dht11pinInput.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;

	//making the Pin as Pull up. Not done since external pull up by 3.3K
	//dht11pinInput.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_PULL_UP;

	dht11pinInput.GPIO_PinConfig.GPIO_PinNumber = DHT11_GPIO_PIN_NO;
	dht11pinInput.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERY_HIGH;
	GPIO_Init(&dht11pinInput);

}


static void DHT11_SendAndGetResponseByMCU(void)
{
	GPIO_WriteToOutputPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN_NO, 0);


	//Step 2 : Send start signal by MCU

	//configure the DHT11 Pin as output mode first. Not done here since done in init.
	//DHT11_Sensor_SetPinOutput();

	//step 2a : MCU dht11 pin as low for 18ms. Default value = 0 (in init)
	delay_ms(20);

	//(*funcptr_delay_us)(18);

	//step 2b : MCU put dht11 pin as high for max time 40us and wait for sensor response

	//GPIO_WriteToOutputPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN_NO, 1);


	//(*funcptr_delay_us)(30);
	delay_us(20);

	//put the pin as input mode to read status of bit
	DHT11_Sensor_SetPinInput();


	//MCU wait until the pin becomes low
	//while(GPIO_ReadFromInputPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN_NO));

	//after that the pin goes low by sensor








}

static int8_t DHT11_Sensor_Check_Response(void)
{
	//flag to check whether the sensor response correct or not
	int8_t Sensor_Response_Flag = 0;


	//wait for 40us to make the pin goes low
	//delay_us(40);


	/*Step 3 : DHT11 Sends out a response and high notification peripheral
	           signal to indicate receive of data */

	//step 3a : sensor send out low response signal and keep it for 80us
	// This is done for response signal by sensor to MCU
	//checking the low signal for 80 us. only done if the signal is low
	if (!GPIO_ReadFromInputPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN_NO))
	{
		delay_us(80);

		//if the signal goes high after 80us i.e response is correct
		//sensor pull up the voltage high upto 80 us
		if (GPIO_ReadFromInputPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN_NO))
			Sensor_Response_Flag = 1;
		else
			Sensor_Response_Flag = 2;
	}

	//wait until the signal goes to low again to indicate the actual data
	while(GPIO_ReadFromInputPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN_NO));
	//delay_us(79);

	return Sensor_Response_Flag;

}

static int8_t DHT11_Sensor_Read_Data(void)
{
	//to store the actual data
	int8_t data = 0;



	//step 4 : Reading actual data
	//Here the total data transmitted bit = 40bit
	//for each transmission 8 bit is used
	for(uint8_t j = 0;j<8;j++)
	{

		//wait until the pin goes high to indicate that 50us has been done.
		//since at end of data transfer the sensor puts the signal low for
		//50us.
		while(!GPIO_ReadFromInputPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN_NO));


		//if signal duration 26-28us then it is low bit
		//if signal duration 70us then it is high bit
		//So, here we're taking the signal as 40us
		delay_us(40);
		//(*funcptr_delay_us)(40);

		//If after the 40us the signal is low i.e the bit is low
		if(!GPIO_ReadFromInputPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN_NO))
			data &= ~(1 << (7-j) );//write 0 in data starting from MSB

		//If after the 40us the signal is high i.e the bit is high
		else
			data |= (1 << (7-j));//write 1 in data starting from MSB bit

		//wait for signal to low to indicate next bit data
		while(GPIO_ReadFromInputPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN_NO));
			//delay_us(70-40);
	}
	return data;
}

/* *********************************Documentation Section **********************************************
 * @fn                           : LCD_I2C_HD44780_Init
 * @brief                        : Turn display on and initialize it parameters.
 * @return                       : 1 -> LCD successfully Init, 0 -> error occured.
 * @Note                         :
 */
void DHT11_Sensor_Init(void)
{
	//Step1 : Wait for 1 second to stabilize the unstable status

	//(*funcptr_delay_ms)(1000);

	//Step2 : Default configuration - Open drain Output mode with 0 state
	DHT11_Sensor_SetPinOutput();





}

/* *********************************Documentation Section **********************************************
 * @fn                           : LCD_I2C_HD44780_Init
 * @brief                        : Turn display on and initialize it parameters.
 * @return                       : 1 -> LCD successfully Init, 0 -> error occured.
 * @Note                         :
 */
uint8_t DHT11_Sensor_GetData(DHT11_Sensor_Data *pDHTData)
{

	DHT11_Sensor_Init();
	DHT11_SendAndGetResponseByMCU();


	int8_t response = DHT11_Sensor_Check_Response();
	(void)response;
	/*if ( response == 2)
	{
		//Error_Handler(STATUS_ERROR);
		return -1;
	}*/


	//40bit data
	//Relative humidity (RH) data
	uint8_t RHIntData       = DHT11_Sensor_Read_Data();
	uint8_t RHDecimalData   = DHT11_Sensor_Read_Data();

	//Temperature data
	uint8_t TempIntData     = DHT11_Sensor_Read_Data();
	uint8_t TempDecimalData = DHT11_Sensor_Read_Data();

	uint8_t CheckSum        = DHT11_Sensor_Read_Data();

	(void) RHIntData;
	(void) RHDecimalData;
	(void ) TempIntData;
	(void) TempDecimalData;
	(void)CheckSum;

	/*

	//check for valid data
	if (CheckSum == (RHIntData + RHDecimalData + TempIntData + TempDecimalData) )
	{
		//here the last 4 bits are for decimals and multiply by 0.1 for 1 decimal point
		//pDHTData->Temperature = TempIntData + (TempDecimalData & 0x0F)*0.01;
		pDHTData->Temperature = TempIntData;

		//here the last 4 bits are for decimals and multiply by 0.1 for 1 decimal point
		//pDHTData->Humidity = RHIntData + (RHDecimalData)*0.01;
		pDHTData->Humidity = RHIntData;

	}

	*/
	//DHT11_Sensor_Init();//for 1 sec wait
	DHT11_Sensor_SetPinOutput();
	return 1;

}



