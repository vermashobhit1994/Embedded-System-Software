/*
 * stm32f44xx_dac_driver.h
 *
 *  Created on: 18-Jul-2021
 *      Author: vermas
 */

#ifndef INC_STM32F44XX_DAC_DRIVER_H_
#define INC_STM32F44XX_DAC_DRIVER_H_

#include"stm32f446xx.h"

typedef struct
{
	uint8_t TriangularWaveAmplitude;//Possible values from @ref DAC_AMP
	uint8_t DAC_Channel;

}DAC_Config_t;


/* Macro for Channel Number */
#define DAC_CHANNEL_1              1
#define DAC_CHANNEL_2              2

typedef struct
{
	DAC_RegDef_t *pDAC;
	DAC_Config_t DACConfig;
}DAC_Handle_t;

/* @ref DAC_AMP
 * Possible values of DAC Triangular Wave amplitude
 */
typedef enum
{
	DAC_TRIA_AMP_1 = 0x00U,
	DAC_TRIA_AMP_3,
	DAC_TRIA_AMP_7,
	DAC_TRIA_AMP_15,
	DAC_TRIA_AMP_31,
	DAC_TRIA_AMP_63,
	DAC_TRIA_AMP_127,
	DAC_TRIA_AMP_255,
	DAC_TRIA_AMP_511,
	DAC_TRIA_AMP_1023,
	DAC_TRIA_AMP_2047,
	DAC_TRIA_AMP_4095,

}DAC_Triangular_Amplitude;

/* Bit Macros for DAC_CR register */
#define DAC_CR_EN1                0
#define DAC_CR_BOFF1              1
#define DAC_CR_TEN1               2
#define DAC_CR_TSEL1              3
#define DAC_CR_WAVE1              6
#define DAC_CR_MAMP1              8
#define DAC_CR_DMAEN1             12
#define DAC_CR_DMAUDRIE1          13
#define DAC_CR_EN2                16
#define DAC_CR_BOFF2              17
#define DAC_CR_TEN2               18
#define DAC_CR_TSEL2              19
#define DAC_CR_WAVE2              22
#define DAC_CR_MAMP2              24
#define DAC_CR_DMAEN2             28
#define DAC_CR_DMAUDRIE2          29



























#endif /* INC_STM32F44XX_DAC_DRIVER_H_ */
