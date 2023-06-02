/*
 * stm32f44xx_rcc_driver.h
 *
 *  Created on: 17-Jul-2021
 *      Author: vermas
 */

#ifndef INC_STM32F44XX_RCC_DRIVER_H_
#define INC_STM32F44XX_RCC_DRIVER_H_

#include<stdint.h>
#include"stm32f446xx.h"

/* PLL configuration macros */
#define PLL_M         0x4U
#define PLL_N         180
#define PLL_P         0x0//i.e 2


/* Bit definition macros for PWR Register */

#define PWR_CR_VOS_SCALE1       0x3U
#define PWR_CR_VOS_SCALE2       0x2U
#define PWR_CR_VOS_SCALE3       0x1U
/*******************************************/

/* Bit definition macros for PWR Register */

/* macro for wait state for flash latency*/
#define FLASH_ACR_LATENCY_0_WS    0x0U
#define FLASH_ACR_LATENCY_1_WS    0x1U
#define FLASH_ACR_LATENCY_2_WS    0x2U
#define FLASH_ACR_LATENCY_3_WS    0x3U
#define FLASH_ACR_LATENCY_4_WS    0x4U
#define FLASH_ACR_LATENCY_5_WS    0x5U
#define FLASH_ACR_LATENCY_6_WS    0x6U
#define FLASH_ACR_LATENCY_7_WS    0x7U
#define FLASH_ACR_LATENCY_8_WS    0x8U
#define FLASH_ACR_LATENCY_9_WS    0x9U
#define FLASH_ACR_LATENCY_10_WS   0xAU
#define FLASH_ACR_LATENCY_11_WS   0xBU
#define FLASH_ACR_LATENCY_12_WS   0xCU
#define FLASH_ACR_LATENCY_13_WS   0xDU
#define FLASH_ACR_LATENCY_14_WS   0xEU
#define FLASH_ACR_LATENCY_15_WS   0xFU

/*******************************************/

/* Bit definition macros for RCC_CFGR Register */

/* for AHB prescaler */
#define RCC_CFGR_HPRE_HCLK_NODIV       0x0U
#define RCC_CFGR_HPRE_HCLK_DIV2       0x8U
#define RCC_CFGR_HPRE_HCLK_DIV4       0x9U
#define RCC_CFGR_HPRE_HCLK_DIV8       0xAU
#define RCC_CFGR_HPRE_HCLK_DIV16      0xBU
#define RCC_CFGR_HPRE_HCLK_DIV64      0xCU
#define RCC_CFGR_HPRE_HCLK_DIV128     0xDU
#define RCC_CFGR_HPRE_HCLK_DIV256     0xEU
#define RCC_CFGR_HPRE_HCLK_DIV512     0xFU

/* for APB1 prescaler */
#define RCC_CFGR_PPRE1_PCLK1_DIV0     0x0U
#define RCC_CFGR_PPRE1_PCLK1_DIV2     0x4U
#define RCC_CFGR_PPRE1_PCLK1_DIV4     0x5U
#define RCC_CFGR_PPRE1_PCLK1_DIV8     0x6U
#define RCC_CFGR_PPRE1_PCLK1_DIV16     0x7U


/* for APB2 prescaler */
#define RCC_CFGR_PPRE2_PCLK2_DIV0     0x0U
#define RCC_CFGR_PPRE2_PCLK2_DIV2     0x4U
#define RCC_CFGR_PPRE2_PCLK2_DIV4     0x5U
#define RCC_CFGR_PPRE2_PCLK2_DIV8     0x6U
#define RCC_CFGR_PPRE2_PCLK2_DIV16    0x7U

/* for System clock switch */
#define RCC_CFGR_SW_SYSCLK_HSI        0x0U
#define RCC_CFGR_SW_SYSCLK_HSE        0x1U
#define RCC_CFGR_SW_SYSCLK_PLL_P      0x2U
#define RCC_CFGR_SW_SYSCLK_PLL_R      0x3U









/*******************************************/



uint32_t RCC_GetPCLK1Value(void);

uint32_t RCC_GetPCLK2Value(void);

void SystemClockConfig(void);
uint32_t GetSystemCoreClock(void);


#endif /* INC_STM32F44XX_RCC_DRIVER_H_ */
