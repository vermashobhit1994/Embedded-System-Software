




#include"stm32f44xx_rcc_driver.h"


/* *********************************Documentation Section **********************************************
 * @fn                           :  RCC_GetPLLOutputClk
 * @brief                        : Function to get the PLL clock source value.
 * @return                       : 32 bit clock source value.
 * Special Note                  : Not implemented.
 */

static uint32_t RCC_GetPLL_P_OutputClk(void)
{
	return 180000000;//configured by delay function
}

/* *********************************Documentation Section **********************************************
 * @fn                           :  RCC_GetPLLROutputClk
 * @brief                        : Function to get the PLL_R clock source value.
 * @return                       : 32 bit clock source value.
 * Special Note                  : Not implemented.
 */
static uint32_t RCC_GetPLL_R_OutputClk(void)
{
	return 1;
}



/*array to hold the prescaler division factor for APB1 Prescaler*/
static uint8_t APB1_Prescaler_array [4] = {2,4,8,16};

/* array to hold the prescaler division factor for AHB Prescaler.*/
static uint16_t AHB_Prescaler_array [] = {2,4,8,16,64,128,256,512};



/* *********************************Documentation Section **********************************************
 * @fn                           : RCC_GetPCLK1Value
 * @brief                        : Function to get the clock value for APB1 bus.
 * @return                       : 32 bit clock value.
 * Special Note                  : Taken from clock tree.
 */
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t Pclk1,SystemClk;//variable to store the APB1 clock value.
    uint8_t temp,clksrc,APB1_Prescaler_Value,AHB_Prescaler_Value;

	/* ************** STEP 1 : to find the current clock source value from  HSE,HSI,PLLCLK,PLLR************
	 * Refer to page 133 on RM0390
	 * Done by two bits of SWS in RCC_CFGR Register.
	 * Here our clock source is HSI i.e 16MHz.*/
	clksrc = (  (RCC -> RCC_CFGR >> 2) & 0x3);//varible to hold current clock source by reading value.

	/*select HSI i.e High Speed Internal clock as clock source.*/
	if(clksrc == 0)
	{
		SystemClk = 16000000;//i.e 16 MHz Internal Clock
	}
	/* Select the HSE as system clock. In Nucleo f446re there is no external oscillator*/
	else if (clksrc == 1)
		SystemClk = 8000000;//external clock source of 8 MHz
	/* PLL_P is the system Clock */
	else if (clksrc == 2)
		SystemClk = RCC_GetPLL_P_OutputClk();//not implemented Here
	else if(clksrc == 3)
		SystemClk = RCC_GetPLL_R_OutputClk();


	/* ***************** step2 : find out the AHB prescaler value**************************************
	 * Done by reading the HPRE Bit of RCC_CFGR Register.*/
	temp = ( (RCC -> RCC_CFGR >> 4) & 0xF);
	if(temp < 8)//no prescaler
		AHB_Prescaler_Value = 1;
	//select the prescaler value from array
	else
		//here -8 since to refer the value corresponding to index since value starting from 8
		AHB_Prescaler_Value = AHB_Prescaler_array[temp - 8];


	/*** Step 3 : find the APB1 Prescaler value since I2C is connected to APB1 bus.*/
	temp = ( (RCC -> RCC_CFGR >> 10) & 0x7);
	if(temp < 4)//no prescaler
		APB1_Prescaler_Value = 1;
	else
		APB1_Prescaler_Value = APB1_Prescaler_array[temp - 4];//extract value from array.

	/* ************* Step 4: Put value in Pclk1 calculated ****************************************************/
	/*Here SYSCLK is connected to AHB prescaler and PCLK1(APB1 Peripheral
	clock) is connected via APB1 Prescaler.*/
	//first find systemclk then divide by AHB1 prescaler value then by APB1 prescaler value.
	Pclk1 = (SystemClk / AHB_Prescaler_Value)/APB1_Prescaler_Value;

	return Pclk1;

}



static uint8_t APB2_Prescaler_array[4] = {2,4,8,16};

/* *********************************Documentation Section **********************************************
 * @fn                           : RCC_GetPCLK2Value
 * @brief                        : Function to get the clock value for APB2 bus.
 * @return                       : 32 bit clock value.
 * Special Note                  : Taken from clock tree.
 */
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t Pclk2,SystemClk;//variable to store the APB2 clock value.
    uint8_t temp,clksrc,APB2_Prescaler_Value,AHB_Prescaler_Value;

	/* ************** STEP 1 : to find the current clock source value from  HSE,HSI,PLLCLK,PLLR************
	 * Refer to page 133 on RM0390
	 * Done by two bits of SWS in RCC_CFGR Register.
	 * Here our clock source is HSI i.e 16MHz.*/
	clksrc = (  (RCC -> RCC_CFGR >> 2) & 0x3);//varible to hold current clock source by reading value.

	/*select HSI i.e High Speed Internal clock as clock source.*/
	if(clksrc == 0)
	{
		SystemClk = 16000000;//i.e 16 MHz Internal Clock
	}
	/* Select the HSE as system clock. In Nucleo f446re there is no external oscillator*/
	else if (clksrc == 1)
		SystemClk = 8000000;//external clock source of 8 MHz
	/* PLL_P is the system Clock */
	else if (clksrc == 2)
		SystemClk = 180000000;//not implemented Here
	else if(clksrc == 3)
		SystemClk = 0;
		//SystemClk = RCC_GetPLL_R_OutputClk();



	/* ***************** step2 : find out the AHB prescaler value**************************************
	 * Done by reading the HPRE Bit of RCC_CFGR Register.*/
	temp = ( (RCC -> RCC_CFGR >> 4) & 0xF);
	if(temp < 8)//no prescaler
		AHB_Prescaler_Value = 1;
	//select the prescaler value from array
	else
		//here -8 since to refer the value corresponding to index since value starting from 8
		AHB_Prescaler_Value = AHB_Prescaler_array[temp - 8];


	/*** Step 3 : find the APB1 Prescaler value since I2C is connected to APB1 bus.*/
	temp = ( (RCC -> RCC_CFGR >> 10) & 0x7);
	if(temp < 4)//no prescaler
		APB2_Prescaler_Value = 1;
	else
		APB2_Prescaler_Value = APB2_Prescaler_array[temp - 4];//extract value from array.

	/* ************* Step 4: Put value in Pclk1 calculated ****************************************************/
	/*Here SYSCLK is connected to AHB prescaler and PCLK1(APB1 Peripheral
	clock) is connected via APB1 Prescaler.*/
	//first find systemclk then divide by AHB1 prescaler value then by APB1 prescaler value.
	Pclk2 = (SystemClk / AHB_Prescaler_Value)/APB2_Prescaler_Value;

	return Pclk2;

}



/* *********************************Documentation Section **********************************************
 * Function Name                 : SystemClockConfig
 * Brief Description             : Configure the SysClk as PLL_P
 * Return Type                   : None
 * Special Note                  : SysClk = 180MHz
 */
void SystemClockConfig(void)
{
	//Step1 : Enable the external crystal (HSE) and wait for it to become ready
	RCC -> RCC_CR |= (0x1U << RCC_CR_HSEON) ;

	//enable the bypass since no HSE in stm32f446re
	//avoid since SysClk gets the clock from HSE not from HSE bypass
	//RCC -> RCC_CR &= ~(0x1U << RCC_CR_HSEBYP);


	//wait until the HSE crystal is ready
	while (!(RCC -> RCC_CR & (0x1U << RCC_CR_HSERDY) ) );

	//Step2 : Set the power enable clock and volage regulator scaling output
	RCC -> RCC_APB1ENR |= (0x1U << RCC_APB1ENR_PWR_EN);
	PWR -> PWR_CR |= (PWR_CR_VOS_SCALE1 << PWR_CR_VOS);

	//Step3 : Configure the flash prefetch and latency related settings
	//instruction cache , data cache and prefetch is enabled.
	FLASH ->FLASH_ACR |= (1 << FLASH_ACR_ICEN) | ( 1 << FLASH_ACR_DCEN) | (1 << FLASH_ACR_PRFTEN);
	FLASH -> FLASH_ACR |= ((FLASH_ACR_LATENCY_5_WS << FLASH_ACR_LATENCY));

	//Step4 : Configure the prescaler for peripheral clocks i.e for HCLK, PCLK1, PCLK2
	RCC -> RCC_CFGR |= (RCC_CFGR_HPRE_HCLK_NODIV << RCC_CFGR_HPRE);//select the ahb presclar = 1
	RCC -> RCC_CFGR |= (RCC_CFGR_PPRE1_PCLK1_DIV8 << RCC_CFGR_PPRE1) ;//AHB clock divided by 4 i.e APB1 clock = 180/8=22.5MHz
	RCC -> RCC_CFGR |= (RCC_CFGR_PPRE2_PCLK2_DIV2 << RCC_CFGR_PPRE2);//AHB clock divided by 2 i.e APB2 clock = 90MHz

	//Now the APB1 Timer Clock = APB1 Clock * 2 = 22.5MHz * 2 = 45MHz
	//Done since we want the timer delay = 1ms

	//Step5 : Configure the main PLL clock.
	//clearing the old value first then write new value since Reset value isn't zero
	RCC -> RCC_PLLCFGR &= ~(0x3F << RCC_PLLCFGR_PLLM);
	RCC -> RCC_PLLCFGR |= ( (PLL_M & 0x3F ) << RCC_PLLCFGR_PLLM) ;

	RCC -> RCC_PLLCFGR &= ~(0x1FF << RCC_PLLCFGR_PLLN);
	RCC -> RCC_PLLCFGR |= ( (PLL_N & 0x1FF ) << RCC_PLLCFGR_PLLN)  ;

	RCC -> RCC_PLLCFGR &= ~( 0x3  << RCC_PLLCFGR_PLLP);
	RCC -> RCC_PLLCFGR |= ( (PLL_P ) << RCC_PLLCFGR_PLLP);

	//select the pll source i.e HSE
	RCC -> RCC_PLLCFGR |= (0x1U << RCC_PLLCFGR_PLLSRC);


	//Step6 : Enable the main PLL and wait it to become ready.
	RCC -> RCC_CR |= (0x1U << RCC_CR_PLLON);
	//wait until the PLL clock is ready
	while (!(RCC -> RCC_CR & (0x1U << RCC_CR_PLLRDY) ) );


	//Step7 : Select the PLLP as clock source and wait for it to be set.
	RCC -> RCC_CFGR |= ( RCC_CFGR_SW_SYSCLK_PLL_P << RCC_CFGR_SW);
	//wait until the clock source is ready i.e PLL_P clock source
	while ( ( ((RCC -> RCC_CFGR) >> 2) & 0x3 )  != RCC_CFGR_SWS_PLL );

}


/* *********************************Documentation Section **********************************************
 * Function Name                 : GetSystemCoreClock
 * Brief Description             : This function used to Get the current SysClock
 * Return Type                   : System clock used.
 * Special Note                  : Here the Peripheral clock = 180MHz
 */
uint32_t GetSystemCoreClock(void)
{
	uint8_t SWS_BITS = ((RCC_CFGR_SWS >> 2) & 0x3);
	uint32_t SystemCoreClock = 0;
	switch(SWS_BITS)
	{
	case 0:
		SystemCoreClock = 16000000;//when internal RC oscillator is used.
		break;
	case 1 :
		//in nucleo f446re external clock from debugger i.e 8MHz is consided.
		SystemCoreClock = 8000000;//when external crystal or external clock is used.
		break;
	case 2:
		//configured using SystemClockConfig()
		 SystemCoreClock = 180000000;//max frequency clock = 180MHz
		 break;
	case 3:
		 break;
	}
    return SystemCoreClock;
}




