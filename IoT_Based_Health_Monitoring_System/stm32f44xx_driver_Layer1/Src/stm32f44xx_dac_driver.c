/*
 * stm32f44xx_dac_driver.c
 *
 *  Created on: 18-Jul-2021
 *      Author: vermas
 */

#include"stm32f44xx_dac_driver.h"
/* *********************************Documentation Section **********************************************
 * @fn                           : DAC_PeriClock_Ctrl
 * @brief                        : Function to Control the peripheral clock of DAC.
 * @param[in]                    : Base address of the DAC Register definition structure .
 * @param[in]                    : Bit to enable or disable.
 * @return                       : None
 * Special Note                  : None
 */
void DAC_PeriClock_Ctrl(DAC_RegDef_t* pDAC, uint8_t EnOrDi)
{
	DAC_PERI_CLK_EN();
}


/* *********************************Documentation Section **********************************************
 * @fn                           :
 * @brief                        :
 * @param[in1]                   :                   :
 * @return                       :
 * Special Note                  :
 */
void DAC_Triangular_Wave_EN_DI(DAC_Handle_t *pDACHandle , uint8_t EnOrDi)
{
	if((EnOrDi == ENABLE) && (pDACHandle -> DACConfig.DAC_Channel) == DAC_CHANNEL_1)
		pDACHandl -> pDAC -> DAC_CR |= (0x2 << DAC_CR1_WAVE1);

	//Here we're disable the channel also
	else if((EnOrDi == DISABLE) && (pDACHandle -> DACConfig.DAC_Channel) == DAC_CHANNEL_1)
	{
		pDACHandl -> pDAC -> DAC_CR &= ~(0x3 << DAC_CR1_WAVE1);
		pDACHandle -> pDAC -> DAC_CR &= ~(0x1 << DAC_CR1_WAVE1);
	}

	else if((EnOrDi == ENABLE) && (pDACHandle -> DACConfig.DAC_Channel) == DAC_CHANNEL_2)
		pDACHandl -> pDAC -> DAC_CR |= (0x2 << DAC_CR1_WAVE2);

	//Here we're disable the channel also
	else if((EnOrDi == DISABLE) && (pDACHandle -> DACConfig.DAC_Channel) == DAC_CHANNEL_2)
	{
		pDACHandl -> pDAC -> DAC_CR &= ~(0x3 << DAC_CR1_WAVE2);
		pDACHandle -> pDAC -> DAC_CR &= ~(0x1 << DAC_CR1_WAVE2);
	}

}

void DAC_Init(DAC_RegDef_t *pDAC)
{
	DAC_PeriClock_Ctrl(pDAC, ENABLE);
}

static uint8_t TriangleAmplitude[12] = {1, 3, 7, 15, 31, 63, 127,
		                              255, 511, 1023, 2047, 4095};

/* *********************************Documentation Section **********************************************
 * @fn                           :
 * @brief                        :
 * @param[in1]                   :                   :
 * @return                       :
 * Special Note                  :
 */
void DAC_Triangle_Wave_Generate(DAC_Handle_t *pDACHandle)
{

	//Step2 : Enable the output buffer
	pDACHandle -> pDAC -> DAC_CR |= (1 << DAC_CR_BOFF1);

	//Step3 : Configure the amplitude

	//if channel 1 is selected
	if (pDACHandle -> DACConfig.DAC_Channel == DAC_CHANNEL_1)
	{
		pDACHandle -> pDAC -> DAC_CR |= (pDACHandle -> DACConfig.TriangularWaveAmplitude) << DAC_CR_MAMP1;
	}

	//if channel 1 is selected
	if (pDACHandle -> DACConfig.DAC_Channel == DAC_CHANNEL_2)
	{
		pDACHandle -> pDAC -> DAC_CR |= (pDACHandle -> DACConfig.TriangularWaveAmplitude) << DAC_CR_MAMP2;
	}

	//Step4 : Select the trigger event
	//Triangular wave counter is increment by 3 APB1 clk cycle after
	//each trigger event.It goes on until the DORx value is less than
	//programmed amplitude

	//Step1 : Enable the channel number selected in user
	if (pDACHandle -> DACConfig.DAC_Channel == DAC_CHANNEL_1)
		pDACHandle -> pDAC -> DAC_CR |= (1 << DAC_CR_EN1);
	else if (pDACHandle -> DACConfig.DAC_Channel == DAC_CHANNEL_2)
			pDACHandle -> pDAC -> DAC_CR |= (1 << DAC_CR_EN2);


	//Step5 : Start or Disable the triangular wave
	DAC_Triangular_Wave_EN_DI(pDACHandle );

	//wait for 3 APB1 clock cycles
}

/* *********************************Documentation Section **********************************************
 * @fn                           :
 * @brief                        :
 * @param[in1]                   :                   :
 * @return                       :
 * Special Note                  :
 */
uint16_t DAC_GetTriangularWaveAmplitude(DAC_Handle_t *pDACHandle)
{
	if(pDACHandle -> DACConfig.DAC_Channel == DAC_CHANNEL_1)
		return (pDACHandle -> pDAC -> DAC_DOR1 & 0xFFF);
	else
		return (pDACHandle -> pDAC -> DAC_DOR2 & 0xFFF);
}


