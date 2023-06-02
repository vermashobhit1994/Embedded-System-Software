/*
 * stm32f446xx.h
 *
 *  Created on: Jul 14, 2020
 *      Author: vermas
 */
/* Description : MCU Specific header file*/

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

//To use NULL in SPI_TXE_Interrupt_Handle function .
#include<stddef.h>
#include<stdint.h>


/*volatile short notation*/
#define __vo volatile

 /*******************************SPECIAL NOTE: PROVIDE () WHILE SPECIFY BASE ADDRESS OF PERIPHERAL*****************/


/***************************************************************************************************************/
/*************************************** Processor Specific Details START *********************************************
 * For the ARM Cortex M4 NVIC ISRx Register i.e to enable the Interrupt.
 * Here each register is of 32 bit (4bytes) size and there are 8 such registers*/

/* NVIC ISER(Interrupt Set Enable Register) Base Addresses*/
#define NVIC_ISER0_BASE_ADDR                  ( (__vo uint32_t*)0xE000E100)
#define NVIC_ISER1_BASE_ADDR                  ( (__vo uint32_t*)0xE000E104)
#define NVIC_ISER2_BASE_ADDR                  ( (__vo uint32_t*)0xE000E108)
#define NVIC_ISER3_BASE_ADDR                  ( (__vo uint32_t*)0xE000E10C)
#define NVIC_ISER4_BASE_ADDR                  ( (__vo uint32_t*)0xE000E110)
#define NVIC_ISER5_BASE_ADDR                  ( (__vo uint32_t*)0xE000E114)
#define NVIC_ISER6_BASE_ADDR                  ( (__vo uint32_t*)0xE000E118)
#define NVIC_ISER7_BASE_ADDR                  ( (__vo uint32_t*)0xE000E11C)

/* NVIC ICER(Interrupt Clear Enable Register) Base Addresses*/
#define NVIC_ICER0_BASE_ADDR                  ( (__vo uint32_t*)0xE000E180)
#define NVIC_ICER1_BASE_ADDR                  ( (__vo uint32_t*)0xE000E184)
#define NVIC_ICER2_BASE_ADDR                  ( (__vo uint32_t*)0xE000E188)
#define NVIC_ICER3_BASE_ADDR                  ( (__vo uint32_t*)0xE000E18C)
#define NVIC_ICER4_BASE_ADDR                  ( (__vo uint32_t*)0xE000E190)
#define NVIC_ICER5_BASE_ADDR                  ( (__vo uint32_t*)0xE000E194)
#define NVIC_ICER6_BASE_ADDR                  ( (__vo uint32_t*)0xE000E198)
#define NVIC_ICER7_BASE_ADDR                  ( (__vo uint32_t*)0xE000E19C)

/* NVIC ICER(Interrupt Priority Register) Base Addresses*/
#define NVIC_PR_BASE_ADDR                     ( (__vo uint32_t*)0xE000E400)

/*Macro for the no of priority bits implemented for the Interrupt Priority Register
 * For the ARM Cortex M4 processor it is 4*/
#define NO_OF_PRIORITY_BITS_IMPLEMENTED         4

/* Macros for all possible IRQ Priority */
#define NVIC_IRQ_PRIO0                 0
#define NVIC_IRQ_PRIO1                 1
#define NVIC_IRQ_PRIO2                 2
#define NVIC_IRQ_PRIO3                 3
#define NVIC_IRQ_PRIO4                 4
#define NVIC_IRQ_PRIO5                 5
#define NVIC_IRQ_PRIO6                 6
#define NVIC_IRQ_PRIO7                 7
#define NVIC_IRQ_PRIO8                 8
#define NVIC_IRQ_PRIO9                 9
#define NVIC_IRQ_PRIO10                10
#define NVIC_IRQ_PRIO11                11
#define NVIC_IRQ_PRIO12                12
#define NVIC_IRQ_PRIO13                13
#define NVIC_IRQ_PRIO14                14
#define NVIC_IRQ_PRIO15                15
#define NVIC_IRQ_PRIO16                16
#define NVIC_IRQ_PRIO17                17
#define NVIC_IRQ_PRIO18                18
#define NVIC_IRQ_PRIO19                19
#define NVIC_IRQ_PRIO20                20
#define NVIC_IRQ_PRIO21                21
#define NVIC_IRQ_PRIO22                22
#define NVIC_IRQ_PRIO23                23
#define NVIC_IRQ_PRIO24                24
#define NVIC_IRQ_PRIO25                25
#define NVIC_IRQ_PRIO26                26
#define NVIC_IRQ_PRIO27                27
#define NVIC_IRQ_PRIO28                28
#define NVIC_IRQ_PRIO29                29
#define NVIC_IRQ_PRIO30                30
#define NVIC_IRQ_PRIO31                31


/***************************************************************************************************************/
/*************************************** Processor Specific Details ENDS *********************************************/



#include<stdint.h>//for uint32_t,uint16_t,uint8_t

/* defining the base address of FLASH and SRAM memory
**************************************************************************************************************/

#define FLASH_SECTOR_0_BASE_ADDR                 ( (uint32_t)0x08000000)   /*typecast to unsigned int*/
#define SRAM1_BASE_ADDR                 0x20000000U       //change the address to unsigned int
#define SRAM                            SRAM1_BASE_ADDR //define the SRAM1 as the main SRAM
#define SRAM2_BASE_ADDR                 0x2001C000U //base address of SRAM1 + size of the SRAM1
#define ROM_BASE_ADDR                   0x1FFF0000U//here the system memory is the ROM memory
#define OTP_AREA_BASE_ADDR              0x1FFF7800U//one time programmable area.Get from Flash memory details.

/*defining the base address of buses used*/
#define PERIPH_BASE_ADDR                (0x40000000U)//base address of the APB1 bus
#define APB1_PERIPH_BASE_ADDR           PERIPH_BASE_ADDR
#define APB2_PERIPH_BASE_ADDR           0x40010000U //base address of APB2  bus
#define AHB1_PERIPH_BASE_ADDR           0x40020000U//base address of AHB1 bus
#define AHB2_PERIPH_BASE_ADDR           0x50000000U//base address of AHB2 bus

  /* ******************* AHB1 bus peripherals base addresses***********************************************************
 * Base_ADDR_AHB1+offset(for the peripheral)*/
#define GPIOA_BASE_ADDR   (AHB1_PERIPH_BASE_ADDR+0x0000)
#define GPIOB_BASE_ADDR   (AHB1_PERIPH_BASE_ADDR+0x0400)
#define GPIOC_BASE_ADDR   (AHB1_PERIPH_BASE_ADDR+0x0800)
#define GPIOD_BASE_ADDR   (AHB1_PERIPH_BASE_ADDR+0x0C00)
#define GPIOE_BASE_ADDR   (AHB1_PERIPH_BASE_ADDR+0x1000)
#define GPIOF_BASE_ADDR   (AHB1_PERIPH_BASE_ADDR+0x1400)
#define GPIOG_BASE_ADDR   (AHB1_PERIPH_BASE_ADDR+0x1800)
#define GPIOH_BASE_ADDR   (AHB1_PERIPH_BASE_ADDR+0x1C00)
#define GPIOI_BASE_ADDR   (AHB1_PERIPH_BASE_ADDR+0x2000)

/* Base address for FLASH */
#define FLASH_BASE_ADDR   (AHB1_PERIPH_BASE_ADDR+0x3C00U)


//Base Address for the RCC Register
#define RCC_BASE_ADDR     (	AHB1_PERIPH_BASE_ADDR+0x3800)

/* ******************* APB1 bus peripherals base addresses*********************/
/*Defining the peripheral base address on the APB1 bus*/
#define I2C1_BASE_ADDR        (APB1_PERIPH_BASE_ADDR+0x5400U)
#define I2C2_BASE_ADDR        (APB1_PERIPH_BASE_ADDR+0x5800U)
#define I2C3_BASE_ADDR        (APB1_PERIPH_BASE_ADDR+0x5C00U)

#define USART2_BASE_ADDR      (APB1_PERIPH_BASE_ADDR+0x4400U)
#define USART3_BASE_ADDR      (APB1_PERIPH_BASE_ADDR+0x4800U)
#define UART4_BASE_ADDR       (APB1_PERIPH_BASE_ADDR+0x4C00U)
#define UART5_BASE_ADDR       (APB1_PERIPH_BASE_ADDR+0x5000U)

#define SPI2_BASE_ADDR        (APB1_PERIPH_BASE_ADDR+0x3800)
#define SPI3_BASE_ADDR        (APB1_PERIPH_BASE_ADDR+0x3C00)

/* used while configuring the SysClock*/
#define PWR_BASE_ADDR         (APB1_PERIPH_BASE_ADDR+0x7000U)

/* Used for delay */
#define TIM6_BASE_ADDR        (APB1_PERIPH_BASE_ADDR+0x1000U)


/*used for delay with interrupt*/
#define TIM7_BASE_ADDR        (APB1_PERIPH_BASE_ADDR + 0x1400U)

#define DAC_BASE_ADDR         (APB1_PERIPH_BASE_ADDR + 0x7400U)

/*peripheral base address on the APB2 bus*/

#define EXTI_BASE_ADDR        (APB2_PERIPH_BASE_ADDR+0x3C00)

#define SPI1_BASE_ADDR        (APB2_PERIPH_BASE_ADDR+0x3000)
#define SPI4_BASE_ADDR        (APB2_PERIPH_BASE_ADDR + 0x3400)

#define SYSCFG_BASE_ADDR      (APB2_PERIPH_BASE_ADDR+0x3800)

#define USART1_BASE_ADDR      (APB2_PERIPH_BASE_ADDR+0x1000U)
#define USART6_BASE_ADDR      (APB2_PERIPH_BASE_ADDR+0x1400U)

/*************************************************************************************************************/

/*Defining the peripheral specific register structure*/
/*Generic Register Definition Structure for GPIO's
 * Here the structure members helps in the offset.*/
/*************************************************************************************************************/
typedef struct
{
	__vo uint32_t MODER; //GPIO port mode register
	__vo uint32_t OTYPER;//GPIO port output type register
	__vo uint32_t OSPEEDR;//GPIO port output speed register
	__vo uint32_t PUPDR;//GPIO port pull-up/pull-down register
	__vo uint32_t IDR;//GPIO port input data register
	__vo uint32_t ODR;//GPIO port output data register
	__vo uint32_t BSRR;//GPIO port bit set/reset register
	__vo uint32_t LCKR;//GPIO port configuration lock register
	__vo uint32_t AFR[2];//AFR[0] -> for the alternate function low register and AFR[1] is for AFRH. GPIO Port configuration Alternate functionality register

}GPIO_RegDef_t;

/*Generic Register Definition Structure for RCC Peripheral
 * Here the RESERVED is written just for the offset due to reserved one*/
typedef struct
{
	__vo uint32_t      RCC_CR;
	__vo uint32_t      RCC_PLLCFGR;
	__vo uint32_t      RCC_CFGR;
	__vo uint32_t      RCC_CIR;
	__vo uint32_t      RCC_AHB1RSTR;
	__vo uint32_t      RCC_AHB2RSTR;
	__vo uint32_t      RCC_AHB3RSTR;
	 uint32_t          RESERVED;
	__vo uint32_t      RCC_APB1RSTR;
	__vo uint32_t      RCC_APB2RSTR;
	uint32_t           RESERVED2[2];//for two reserved values
	__vo uint32_t      RCC_AHB1ENR;
	__vo uint32_t      RCC_AHB2ENR;
	__vo uint32_t      RCC_AHB3ENR;
	 uint32_t          RESERVED1;
	__vo uint32_t      RCC_APB1ENR;
	__vo uint32_t      RCC_APB2ENR;
	uint32_t           RESERVED3[2];
	__vo uint32_t      RCC_AHB1LPENR;
	__vo uint32_t      RCC_AHB2LPENR;
	__vo uint32_t      RCC_AHB3LPENR;
	uint32_t           RESERVED4;
	__vo uint32_t      RCC_APB1LPENR;
	__vo uint32_t      RCC_APB2LPENR;
	uint32_t           RESERVED5[2];
	__vo uint32_t      RCC_BDCR;
	__vo uint32_t      RCC_CSR;
	uint32_t           RESERVED62[2];
	__vo uint32_t      RCC_SSCGR;
	__vo uint32_t      RCC_PLLI2SCFGR;
	__vo uint32_t      RCC_PLLSAICFGR;
	__vo uint32_t      RCC_DCKCFGR;
	__vo uint32_t      RCC_CKGATENR;
	__vo uint32_t      RCC_DCKCFGR2;

}RCC_RegDef_t;

/*Generic Register Definition Structure for EXTI Peripheral
 */
typedef struct
{
	__vo uint32_t               EXTI_IMR;/* Interrupt mask register. Address Offset = 0x00*/
	__vo uint32_t               EXTI_EMR;/* Event mask register . Address Offset = 0x04*/
	__vo uint32_t               EXTI_RTSR;/* Rising trigger selection register . Address Offset = 0x08*/
	__vo uint32_t               EXTI_FTSR;/* Falling trigger selection register . Address Offset = 0x0C*/
	__vo uint32_t               EXTI_SWIER;/* Software interrupt event register . Address Offset = 0x10*/
	__vo uint32_t               EXTI_PR;/* Pending register . Address Offset = 0x14*/

}EXTI_RegDef_t;

/* Generic Register Definition Structure for the SYSCFG Peripheral*/
typedef struct
{
	__vo uint32_t                SYSCFG_MEMRMP;/*SYSCFG memory remap register. Address Offset =0x00*/
	__vo uint32_t                SYSCFG_PMC;/*SYSCFG peripheral mode configuration register.Address Offset =0x04*/
	__vo uint32_t                SYSCFG_EXTICR[4];/*SYSCFG external interrupt configuration register 1-4.Address Offset =0x08 to 0x14*/
	     uint32_t                RESERVED1[2];
	__vo uint32_t                SYSCFG_CMPCR;/*Compensation cell control register. Address Offset =0x20*/
	     uint32_t                RESERVED2[2];
	__vo uint32_t                SYSCFG_CFGR;/*SYSCFG configuration register. Address Offset =0x2C*/
}SYSCFG_RegDef_t;

/* Generic Register Definition Structure for the SPI Peripheral*/
typedef struct
{
	__vo uint32_t                SPI_CR[2];/*SPI[0] for SPI Control Register1 ,Address Offset = 0x00 SPI[1] for SPI Control Register 2, Address Offset = 0x04 */
	__vo uint32_t                SPI_SR;/*SPI status register . Address Offset = 0x08 */
	__vo uint32_t                SPI_DR;/*SPI data register . Address Offset = 0x0C */
	__vo uint32_t                SPI_CRCPR;/* SPI CRC polynomial register. Address Offset = 0x10*/
	__vo uint32_t                SPI_RXCRCR;/* SPI RX CRC register. Address Offset = 0x14*/
	__vo uint32_t                SPI_TXCRCR;/* SPI TX CRC register. Address Offset = 0x18*/
	__vo uint32_t                SPI_I2SCFGR;/* SPI_I2S configuration register . Address Offset = 0x1C*/
	__vo uint32_t                SPI_I2SPR;/* SPI_I2S prescaler register. Address Offset = 0x20*/
}SPI_RegDef_t;

/* Generic Register Defination structure for the i2c Peripheral*/
typedef struct
{
	__vo uint32_t                I2C_CR[2];/* I2C_CR[0] for I2C Control Register 1, Address Offset = 0x00 and I2C_CR[1] for I2C Control Register 2, Address offset = 0x04*/
	__vo uint32_t                I2C_OAR[2];/* I2C_OAR[0] for I2C Own Address Register1 , Address offset = 0x08 and I2C_OAR[1] for I2C Own Address Register 2, Address offset = 0x0C*/
	__vo uint32_t                I2C_DR ; /* I2C Data Register , Address Offset = 0x10*/
	__vo uint32_t                I2C_SR[2];/* I2C_SR[0] for I2C Status Register1 , Address offset = 0x14 and I2C_SR[1] for I2C Status Register 2, Address offset = 0x18*/
	__vo uint32_t                I2C_CCR;/* I2C Clock Control Register , Address Offset = 0x1C*/
	__vo uint32_t                I2C_TRISE;/* I2C TRISE Register , Address Offset = 0x20*/
	__vo uint32_t                I2C_FLTR;/* I2C FLTR (filter) Register , Address Offset = 0x24*/

}I2C_RegDef_t;


/*Generic Register Defination Structure for USART Peripheral */
typedef struct
{
	__vo uint32_t   USART_SR;
	__vo uint32_t   USART_DR;
	__vo uint32_t   USART_BRR;
	__vo uint32_t   USART_CR1;
	__vo uint32_t   USART_CR2;
	__vo uint32_t   USART_CR3;
	__vo uint32_t   USART_GTPR;

}USART_RegDef_t;

/* Register Defination structure for Timer6 and Timer7*/
//timer6 or timer7 generic register defination
typedef struct
{
	__vo uint32_t     TIMx_CR1;
	__vo uint32_t     TIMx_CR2;
	     uint32_t     RESERVED1;
	__vo uint32_t     TIMx_DIER;
	__vo uint32_t     TIMx_SR;
	__vo uint32_t     TIMx_EGR;
	     uint32_t     RESERVED[3];
	__vo uint32_t     TIMx_CNT;
	__vo uint32_t     TIMx_PSC;
	__vo uint32_t     TIMx_ARR;

}TIMx_RegDef_t;

/* Register definition structure for Flash*/
typedef struct
{
	__vo uint32_t FLASH_ACR;
	__vo uint32_t FLASH_KEYR;
	__vo uint32_t FLASH_OPTKEYR;
	__vo uint32_t FLASH_SR;
	__vo uint32_t FLASH_CR;
	__vo uint32_t FLASH_OPTCR;

}FLASH_RegDef_t;

/* Register definition structure for Power controller */
typedef struct
{
	__vo uint32_t PWR_CR;
	__vo uint32_t PWR_CSR;

}PWR_RegDef_t;

/*Register definitions for DAC*/
typedef struct
{
	volatile uint32_t DAC_CR;
	volatile uint32_t DAC_SWTRIGR;
	volatile uint32_t DAC_DHR12R1;
	volatile uint32_t DAC_DHR12L1;
	volatile uint32_t DAC_DHR8R1;
	volatile uint32_t DAC_DHR12R2;
	volatile uint32_t DAC_DHR12L2;
	volatile uint32_t DAC_DHR8R2;
	volatile uint32_t DAC_DHR12RD;
	volatile uint32_t DAC_DHR12LD;
	volatile uint32_t DAC_DHR8RD;
	volatile uint32_t DAC_DOR1;
	volatile uint32_t DAC_DOR2;
	volatile uint32_t DAC_SR;
}DAC_RegDef_t;




/************************ Peripheral Register Definition Structure Ends ************************************************/
/************************************************************************************************************************/


/*Peripheral definitions macros i.e the peripheral base address typecast to stuct type for particular peripheral*/
/**************************************************************************************************************/

/* Here the GPIOx would store the base address of the peripheral and  is treated as pointer*/
#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOF ((GPIO_RegDef_t*)GPIOF_BASE_ADDR)
#define GPIOG ((GPIO_RegDef_t*)GPIOG_BASE_ADDR)
#define GPIOH ((GPIO_RegDef_t*)GPIOH_BASE_ADDR)
#define GPIOI ((GPIO_RegDef_t*)GPIOI_BASE_ADDR)

/*Peripheral definitions for the RCC */
#define RCC  ((RCC_RegDef_t*)RCC_BASE_ADDR)

/*EXTI Peripheral definition */
#define EXTI  ((EXTI_RegDef_t*)EXTI_BASE_ADDR)

/*SYSCFG Peripheral Definition*/
#define SYSCFG   ((SYSCFG_RegDef_t*)SYSCFG_BASE_ADDR)

/*SPI Peripheral Definition Macros */
#define SPI1        ((SPI_RegDef_t*)SPI1_BASE_ADDR)
#define SPI2        ((SPI_RegDef_t*)SPI2_BASE_ADDR)
#define SPI3        ((SPI_RegDef_t*)SPI3_BASE_ADDR)
#define SPI4        ((SPI_RegDef_t*)SPI4_BASE_ADDR)

/* I2C Peripheral Definition Macros*/
#define I2C1       ((I2C_RegDef_t*)I2C1_BASE_ADDR)
#define I2C2       ((I2C_RegDef_t*)I2C2_BASE_ADDR)
#define I2C3       ((I2C_RegDef_t*)I2C3_BASE_ADDR)


/* USART Peripheral Defination Macros */
#define USART1      ((USART_RegDef_t*)USART1_BASE_ADDR)
#define USART2      ((USART_RegDef_t*)USART2_BASE_ADDR)
#define USART3      ((USART_RegDef_t*)USART3_BASE_ADDR)
#define UART4       ((USART_RegDef_t*)UART4_BASE_ADDR)
#define UART5       ((USART_RegDef_t*)UART5_BASE_ADDR)
#define USART6      ((USART_RegDef_t*)USART6_BASE_ADDR)

/* Timer6 peripheral base address pointer */
#define TIM6         ((TIMx_RegDef_t*)TIM6_BASE_ADDR)

/* Timer7 peripheral base address pointer */
#define TIM7         ((TIMx_RegDef_t*)TIM7_BASE_ADDR)

/* FLASH peripheral base address pointer*/
#define FLASH      ((FLASH_RegDef_t*)FLASH_BASE_ADDR)

/* PWR peripheral base address pointer*/
#define PWR        ((PWR_RegDef_t*)PWR_BASE_ADDR)

/* DAC peripheral Base address pointer*/
#define DAC         ( (DAC_RegDef_t*)DAC_BASE_ADDR)

/********************** Peripheral Definition Macros Ends **********************************************************/
/******************************************************************************************************************/

/*****************************************************************************************************************************/
/********************************************CLOCK ENABLING FOR PERIPHERALS***************************************************/


/*Clock Enable Macro for the GPIOx Peripheral*/
#define GPIOA_PERI_CLK_EN() (RCC -> RCC_AHB1ENR |= (1<<0))//High the 0th bit of the AHB1ENR of RCC to enable GPIOA clock.
#define GPIOB_PERI_CLK_EN() (RCC -> RCC_AHB1ENR |= (1<<1))//High the 1st bit of the AHB1ENR of RCC to enable GPIOB clock.
#define GPIOC_PERI_CLK_EN() (RCC -> RCC_AHB1ENR |= (1<<2))//High the 2nd bit of the AHB1ENR of RCC to enable GPIOC clock.
#define GPIOD_PERI_CLK_EN() (RCC -> RCC_AHB1ENR |= (1<<3))//High the 3rd bit of the AHB1ENR of RCC to enable GPIOD clock.
#define GPIOE_PERI_CLK_EN() (RCC -> RCC_AHB1ENR |= (1<<4))//High the 4th bit of the AHB1ENR of RCC to enable GPIOE clock.
#define GPIOF_PERI_CLK_EN() (RCC -> RCC_AHB1ENR |= (1<<5))//High the 5th bit of the AHB1ENR of RCC to enable GPIOF clock.
#define GPIOG_PERI_CLK_EN() (RCC -> RCC_AHB1ENR |= (1<<6))//High the 6th bit of the AHB1ENR of RCC to enable GPIOG clock.
#define GPIOH_PERI_CLK_EN() (RCC -> RCC_AHB1ENR |= (1<<7))//High the 7th bit of the AHB1ENR of RCC to enable GPIOH clock.
#define GPIOI_PERI_CLK_EN() (RCC -> RCC_AHB1ENR |= (1<<8))//High the 8th bit of the AHB1ENR of RCC to enable GPIOI clock.

//clock enable Macro for the I2Cx peripheral
//here I2Cx peripheral connected to APB1 bus
#define I2C1_PERI_CLK_EN()  (RCC -> RCC_APB1ENR |= (1<<21)) //high the 21st bit to enable I2C1
#define I2C2_PERI_CLK_EN()  (RCC -> RCC_APB1ENR |= (1<<22)) //high the 22nd bit to enable I2C2
#define I2C3_PERI_CLK_EN()  (RCC -> RCC_APB1ENR |= (1<<23)) //high the 23rd bit to enable I2C3

//clock enable Macro for the SPIx peripheral
#define SPI1_PERI_CLK_EN()  ( (RCC -> RCC_APB2ENR) |= (1<<12)) //high the 12th bit of RCC_APB2ENR to enable SPI1 Clock.
#define SPI2_PERI_CLK_EN()  (RCC -> RCC_APB1ENR |= (1<<14)) //high the 14th bit of RCC_APB1ENR to enable SPI2 Clock.
#define SPI3_PERI_CLK_EN()  (RCC -> RCC_APB1ENR |= (1<<15))//high the 15th bit of RCC_APB1ENR to enable SPI3 Clock.
#define SPI4_PERI_CLK_EN()  (RCC -> RCC_APB2ENR |= (1<<13))//high the 13th bit of RCC_APB2ENR to enable SPI4 Clock.

//clock enable Macro for the USARTx peripheral
#define USART1_PERI_CLK_EN()  (RCC -> RCC_APB2ENR |= (1<<4))//high the 4th bit of RCC_APB2ENR to enable USART1 Clock.
#define USART2_PERI_CLK_EN()  (RCC -> RCC_APB1ENR |= (1<<17))//high the 17th bit of RCC_APB1ENR to enable USART2 Clock.
#define USART3_PERI_CLK_EN()  (RCC -> RCC_APB1ENR |= (1<<18))//high the 18th bit of RCC_APB1ENR to enable USART3 Clock.
#define UART4_PERI_CLK_EN()  (RCC -> RCC_APB1ENR |= (1<<19))//high the 19th bit of RCC_APB1ENR to enable UART4 Clock.
#define UART5_PERI_CLK_EN()  (RCC -> RCC_APB1ENR |= (1<<20))//high the 20th bit of RCC_APB1ENR to enable UART5 Clock.
#define USART6_PERI_CLK_EN()  (RCC -> RCC_APB2ENR |= (1<<5))//high the 5th bit of RCC_APB2ENR to enable USART6 Clock.

//clock enable Macro for the SYSCFG peripheral
#define SYSCFG_PERI_CLK_EN() (RCC -> RCC_APB2ENR |= (1<<14))//high the 14th bit of RCC_APB2ENR to enable SYSCFG Clock.

//clock enable Macro for the TIMER6 Peripheral
#define TIM6_PERI_CLK_EN()    (RCC -> RCC_APB1ENR |= (1 << 4) )//high the 4th bit of RCC_APB1ENR to enable the TIM6 peripheral clock.

//clock enable Macro for the TIMER6 Peripheral
#define TIM7_PERI_CLK_EN()    (RCC -> RCC_APB1ENR |= (1 << 5) )//high the 5th bit of RCC_APB1ENR to enable the TIM7 peripheral clock.

//clock enable Macro for DAC
#define DAC_PERI_CLK_EN()     (RCC -> RCC_APB1ENR |= (1 << 29))

/*****************************************************************************************************************/

/*Clock Disable Macro for the GPIOx Peripheral*/
#define GPIOA_PERI_CLK_DI() (RCC -> RCC_AHB1ENR &= ~(1<<0))//Low the 0th bit of the AHB1ENR of RCC to disable GPIOA clock.
#define GPIOB_PERI_CLK_DI() (RCC -> RCC_AHB1ENR &= ~(1<<1))//Low the 1st bit of the AHB1ENR of RCC to disable GPIOB clock.
#define GPIOC_PERI_CLK_DI() (RCC -> RCC_AHB1ENR &= ~(1<<2))//Low the 2nd bit of the AHB1ENR of RCC to disable GPIOC clock.
#define GPIOD_PERI_CLK_DI() (RCC -> RCC_AHB1ENR &= ~(1<<3))//Low the 3rd bit of the AHB1ENR of RCC to disable GPIOD clock.
#define GPIOE_PERI_CLK_DI() (RCC -> RCC_AHB1ENR &= ~(1<<4))//Low the 4th bit of the AHB1ENR of RCC to disable GPIOE clock.
#define GPIOF_PERI_CLK_DI() (RCC -> RCC_AHB1ENR &= ~(1<<5))//Low the 5th bit of the AHB1ENR of RCC to disable GPIOF clock.
#define GPIOG_PERI_CLK_DI() (RCC -> RCC_AHB1ENR &= ~(1<<6))//Low the 6th bit of the AHB1ENR of RCC to disable GPIOG clock.
#define GPIOH_PERI_CLK_DI() (RCC -> RCC_AHB1ENR &= ~(1<<7))//Low the 7th bit of the AHB1ENR of RCC to disable GPIOH clock.
#define GPIOI_PERI_CLK_DI() (RCC -> RCC_AHB1ENR &= ~(1<<8))//Low the 8th bit of the AHB1ENR of RCC to disable GPIOI clock.

//clock Disable Macro for the I2Cx peripheral
//here I2Cx peripheral connected to APB1 bus
#define I2C1_PERI_CLK_DI()  (RCC -> RCC_APB1ENR &= ~(1<<21)) //Low the 21st bit to disable I2C1
#define I2C2_PERI_CLK_DI()  (RCC -> RCC_APB1ENR &= ~(1<<22)) //Low the 22nd bit to disable I2C2
#define I2C3_PERI_CLK_DI()  (RCC -> RCC_APB1ENR &= ~(1<<23)) //Low the 23rd bit to disable I2C3

//clock Disable Macro for the SPIx peripheral
#define SPI1_PERI_CLK_DI()  (RCC -> RCC_APB2ENR &= ~(1<<12)) //Low the 12th bit of RCC_APB2ENR to disable 12th bit
#define SPI2_PERI_CLK_DI()  (RCC -> RCC_APB1ENR &= ~(1<<14)) //Low the 14th bit of RCC_APB1ENR to disable 14th bit
#define SPI3_PERI_CLK_DI()  (RCC -> RCC_APB1ENR &= ~(1<<15))//Low the 15th bit of RCC_APB1ENR to disable 15th bit
#define SPI4_PERI_CLK_DI()  (RCC -> RCC_APB2ENR &= ~(1<<13))//Low the 13th bit of RCC_APB2ENR to disable 13th bit

//clock Disable Macro for the USARTx peripheral
#define USART1_PERI_CLK_DI()  (RCC -> RCC_APB2ENR &= ~(1<<4))
#define USART2_PERI_CLK_DI()  (RCC -> RCC_APB1ENR &= ~(1<<17))
#define USART3_PERI_CLK_DI()  (RCC -> RCC_APB1ENR &= ~(1<<18))
#define UART4_PERI_CLK_DI()  (RCC -> RCC_APB1ENR &= ~(1<<19))
#define UART5_PERI_CLK_DI()  (RCC -> RCC_APB1ENR &= ~(1<<20))
#define USART6_PERI_CLK_DI()  (RCC -> RCC_APB2ENR &= ~(1<<5))

//clock Disable Macro for the SYSCFG peripheral
#define SYSCFG_PERI_CLK_DI() (RCC -> RCC_APB2ENR &= ~(1<<14))

//Clock Disable Macro for DAC peripheral
#define DAC_PERI_CLK_DI()       (RCC ->RCC_APB1ENR &= ~( 1 << 29) )


/*macros to reset the GPIOx peripheral to their reset values
Here first we need to set the bit for corresponding peripheral then reset the bit
Here it is done by do while loop to include more than two statements in a single macro.
This do while loop is called as do while condition zero loop */
#define GPIOA_REG_RESET()    do { (RCC -> RCC_AHB1RSTR |= (1<<0)) ;  (RCC -> RCC_AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()    do { (RCC -> RCC_AHB1RSTR |= (1<<1)) ;  (RCC -> RCC_AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()    do { (RCC -> RCC_AHB1RSTR |= (1<<2)) ;  (RCC -> RCC_AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()    do { (RCC -> RCC_AHB1RSTR |= (1<<3)) ;  (RCC -> RCC_AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()    do { (RCC -> RCC_AHB1RSTR |= (1<<4)) ;  (RCC -> RCC_AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOF_REG_RESET()    do { (RCC -> RCC_AHB1RSTR |= (1<<5)) ;  (RCC -> RCC_AHB1RSTR &= ~(1<<5));}while(0)
#define GPIOG_REG_RESET()    do { (RCC -> RCC_AHB1RSTR |= (1<<6)) ;  (RCC -> RCC_AHB1RSTR &= ~(1<<6));}while(0)
#define GPIOH_REG_RESET()    do { (RCC -> RCC_AHB1RSTR |= (1<<7)) ;  (RCC -> RCC_AHB1RSTR &= ~(1<<7));}while(0)
#define GPIOI_REG_RESET()    do { (RCC -> RCC_AHB1RSTR |= (1<<8)) ;  (RCC -> RCC_AHB1RSTR &= ~(1<<8));}while(0)


/* Macros to reset the SPI Peripheral to their Reset Values*
 *
 */
#define SPI1_REG_RESET()    do { (RCC -> RCC_APB2RSTR |= (1<<12)) ;  (RCC -> RCC_APB2RSTR &= ~(1<<12));}while(0)//set and clear the 12th bit of APB2RSTR
#define SPI2_REG_RESET()    do { (RCC -> RCC_APB1RSTR |= (1<<14)) ;  (RCC -> RCC_APB1RSTR &= ~(1<<14));}while(0)//set and clear the 14th bit of APB1RSTR
#define SPI3_REG_RESET()    do { (RCC -> RCC_APB1RSTR |= (1<<15)) ;  (RCC -> RCC_APB1RSTR &= ~(1<<15));}while(0)//set and clear the 15th bit of APB1RSTR
#define SPI4_REG_RESET()    do { (RCC -> RCC_APB2RSTR |= (1<<13)) ;  (RCC -> RCC_APB2RSTR &= ~(1<<13));}while(0)//set and clear the 13th bit of APB2RSTR

/* Macros to reset  the I2C Peripheral to their Reset Values
 */
#define I2C1_REG_RESET()    do { (RCC -> RCC_APB1RSTR |= (1<<21)); (RCC -> RCC_APB1RSTR &= ~(1<<21));}while(0)//set and clear the 21st bit of APB1RSTR
#define I2C2_REG_RESET()    do { (RCC -> RCC_APB1RSTR |= (1<<22)); (RCC -> RCC_APB1RSTR &= ~(1<<22));}while(0)//set and clear the 22th bit of APB1RSTR
#define I2C3_REG_RESET()    do { (RCC -> RCC_APB1RSTR |= (1<<23)); (RCC -> RCC_APB1RSTR &= ~(1<<23));}while(0)//set and clear the 22th bit of APB1RSTR

/* Macros to reset  the USART and UART Peripheral to their Reset Values
 */
#define USART1_REG_RESET()  do { (RCC -> RCC_APB2RSTR |= (1<<4)); (RCC -> RCC_APB2RSTR &= ~(1<<4));}while(0)//set and clear the 4th bit of APB1RSTR
#define USART2_REG_RESET()  do { (RCC -> RCC_APB1RSTR |= (1<<17)); (RCC -> RCC_APB1RSTR &= ~(1<<17));}while(0)//set and clear the 4th bit of APB1RSTR
#define USART3_REG_RESET()  do { (RCC -> RCC_APB1RSTR |= (1<<18)); (RCC -> RCC_APB1RSTR &= ~(1<<18));}while(0)//set and clear the 4th bit of APB1RSTR
#define UART4_REG_RESET()   do { (RCC -> RCC_APB1RSTR |= (1<<19)); (RCC -> RCC_APB1RSTR &= ~(1<<19));}while(0)//set and clear the 4th bit of APB1RSTR
#define UART5_REG_RESET()   do { (RCC -> RCC_APB1RSTR |= (1<<20)); (RCC -> RCC_APB1RSTR &= ~(1<<20));}while(0)//set and clear the 4th bit of APB1RSTR
#define USART6_REG_RESET()  do { (RCC -> RCC_APB2RSTR |= (1<<5)); (RCC -> RCC_APB2RSTR &= ~(1<<5));}while(0)//set and clear the 4th bit of APB1RSTR




/*Macro to select the Code to select the GPIO Port*/
#define GPIO_BASE_ADDR_TO_CODE(x)   ( (x == GPIOA) ? 0:\
		                              (x == GPIOB) ? 1:\
		                              (x == GPIOC) ? 2:\
		                              (x == GPIOD) ? 3:\
		                              (x == GPIOE) ? 4:\
		                              (x == GPIOF) ? 5:\
		                              (x == GPIOG) ? 6:0)


/* Macro for the IRQ(Interrupt Request ) numbers from the Vector Table*/
/* these are related to GPIO Ports */
#define IRQ_NO_EXTI0                   6
#define IRQ_NO_EXTI1                   7
#define IRQ_NO_EXTI2                   8
#define IRQ_NO_EXTI3                   9
#define IRQ_NO_EXTI4                   10
#define IRQ_NO_EXTI_9_5                23
#define IRQ_NO_EXTI_15_10              40
#define IRQ_NO_TIM7                    55
#define IRQ_NO_USART1                  37
#define IRQ_NO_USART2                  38
#define IRQ_NO_USART3                  39
#define IRQ_NO_UART4                   52
#define IRQ_NO_UART5                   53

/* Macro definations corresponding to SPI IRQ Numbers*/
/* This is taken from vector table */
#define IRQ_NO_SPI1                   35
#define IRQ_NO_SPI2                   36
#define IRQ_NO_SPI3                   51
#define IRQ_NO_SPI4                   84

/* Macro definations corresponding to I2C IRQ Numbers */
/* Taken from Vector Table*/
#define IRQ_NO_I2C1_EV                31
#define IRQ_NO_I2C1_ER                32
#define IRQ_NO_I2C2_EV                33
#define IRQ_NO_I2C2_ER                34
#define IRQ_NO_I2C3_EV                72
#define IRQ_NO_I2C3_ER                73






/*************************************************************************************************************
 * **********************************Some Generic Macros Used by driver header file***************************/
#define ENABLE                    1
#define DISABLE                   0
#define SET                       ENABLE
#define RESET                     DISABLE
#define GPIO_PIN_SET              SET
#define GPIO_PIN_RESET            RESET
#define SPI_FLAG_RESET            RESET        //macro used by SPI driver file to check the status whether data is there or not
#define SPI_FLAG_SET              SET          //macro used by SPI driver file to check the status whether data is there or not

/*************************************************************************************************************/

/*************************************************************************************************************/
/************************************* BIT Position Defination Macros For SPI Peripheral***********************/
/*************************************************************************************************************/

/* Macros for SPI_CR1 Register*/
#define SPI_CR1_CPHA                     0
#define SPI_CR1_CPOL                     1
#define SPI_CR1_MSTR                     2
#define SPI_CR1_BR                       3
#define SPI_CR1_SPE                      6
#define SPI_CR1_LSBFIRST                 7
#define SPI_CR1_SSI                      8
#define SPI_CR1_SSM                      9
#define SPI_CR1_RXONLY                   10
#define SPI_CR1_DFF                      11
#define SPI_CR1_CRCNEXT                  12
#define SPI_CR1_CRCEN                    13
#define SPI_CR1_BIDIOE                   14
#define SPI_CR1_BIDIMODE                 15

/* Macros for SPI_CR2 Register*/
#define SPI_CR2_RXDMAEN                  0
#define SPI_CR2_TXDMAEN                  1
#define SPI_CR2_SSOE                     2
#define SPI_CR2_FRF                      4
#define SPI_CR2_ERRIE                    5
#define SPI_CR2_RXNEIE                   6
#define SPI_CR2_TXEIE                    7

/* Macro for SPI_SR Register*/
#define SPI_SR_RXNE                      0
#define SPI_SR_TXE                       1
#define SPI_SR_CHSIDE                    2
#define SPI_SR_UDR                       3
#define SPI_SR_CRCERR                    4
#define SPI_SR_MODF                      5
#define SPI_SR_OVR                       6
#define SPI_SR_BSY                       7
#define SPI_SR_FRE                       8



/************************************* BIT Position Defination Macros for SPI ENDS ***********************/
/*************************************************************************************************************/

/*************************************************************************************************************/
/**********BIT Position Defination Macros for I2C START ***********************/

//For I2C Control Register 1
#define I2C_CR1_PE                   0
#define I2C_CR1_SMBUS                1
#define I2C_CR1_SMBTYPE              3
#define I2C_CR1_ENARP                4
#define I2C_CR1_ENPEC                5
#define I2C_CR1_ENGC                 6
#define I2C_CR1_NOSTRETCH            7
#define I2C_CR1_START                8
#define I2C_CR1_STOP                 9
#define I2C_CR1_ACK                  10
#define I2C_CR1_POS                  11
#define I2C_CR1_PEC                  12
#define I2C_CR1_ALERT                13
#define I2C_CR1_SWRST                15

//For I2C Control Register 2
#define I2C_CR2_FREQ                 0
#define I2C_CR2_ITERREN              8
#define I2C_CR2_ITEVTEN              9
#define I2C_CR2_ITBUFEN              10
#define I2C_CR2_DMAEN                11
#define I2C_CR2_LAST                 12

//For I2C status Register 1
#define I2C_SR1_SB                    0
#define I2C_SR1_ADDR                  1
#define I2C_SR1_BTF                   2
#define I2C_SR1_ADD10                 3
#define I2C_SR1_STOPF                 4
#define I2C_SR1_RXNE                  6
#define I2C_SR1_TXE                   7
#define I2C_SR1_BERR                  8
#define I2C_SR1_ARLO                  9
#define I2C_SR1_AF                    10
#define I2C_SR1_OVR                   11
#define I2C_SR1_PECERR                12
#define I2C_SR1_TIMEOUT               14
#define I2C_SR1_SMBALERT              15

//For I2C status Register 2
#define I2C_SR2_MSL                   0
#define I2C_SR2_BUSY                  1
#define I2C_SR2_TRA                   2
#define I2C_SR2_GENCALL               4
#define I2C_SR2_SMBDEFAULT            5
#define I2C_SR2_SMBHOST               6
#define I2C_SR2_DUALF                 7
#define I2C_SR2_PEC                   8

//For I2C Clock Control Register
#define I2C_CCR_CCR                   0
#define I2C_CCR_DUTY                  14
#define I2C_CCR_F_S                   15


/************************************* BIT Position Defination Macros for I2C ENDS ***********************/
/*************************************************************************************************************/


/*************************************************************************************************************/
/**********BIT Position Defination Macros for USART/UART START***********************/

//for SR Register
#define USART_SR_PE            0
#define USART_SR_FE            1
#define USART_SR_NF            2
#define USART_SR_ORE           3
#define USART_SR_IDLE          4
#define USART_SR_RXNE          5
#define USART_SR_TC            6
#define USART_SR_TXE           7
#define USART_SR_LBD           8
#define USART_SR_CTS           9


//Bit Position Definition Macros For USART_CR1 Register
#define USART_CR1_SBK          0
#define USART_CR1_RWU          1
#define USART_CR1_RE           2
#define USART_CR1_TE           3
#define USART_CR1_IDLEIE       4
#define USART_CR1_RXNEIE       5
#define USART_CR1_TCIE         6
#define USART_CR1_TXEIE        7
#define USART_CR1_PEIE         8
#define USART_CR1_PS           9
#define USART_CR1_PCE          10
#define USART_CR1_WAKE         11
#define USART_CR1_M            12
#define USART_CR1_UE           13
#define USART_CR1_OVR8         15


//for CR2 Register
#define USART_CR2_LBDL         5
#define USART_CR2_LBDIE        6
#define USART_CR2_LBCL         8
#define USART_CR2_CPHA         9
#define USART_CR2_CPOL         10
#define USART_CR2_CLKEN        11
#define USART_CR2_STOP         12
#define USART_CR2_LINEN        14

//for CR3 Register
#define USART_CR3_EIE           0
#define USART_CR3_IREN          1
#define USART_CR3_IRLP          2
#define USART_CR3_HDSEL         3
#define USART_CR3_NACK          4
#define USART_CR3_SCEN          5
#define USART_CR3_DMAR          6
#define USART_CR3_DMAT          7
#define USART_CR3_RTSE          8
#define USART_CR3_CTSE          9
#define USART_CR3_CTSIE         10
#define USART_CR3_ONEBIT        11


/* Peripheral Register bit macros for USART_BRR Register*/
#define USART_BRR_DIV_FRACTION          0
#define USART_BRR_DIV_MANTISSA          4

/* RCC BDCR Macros corresponding to bits */
#define RCC_BDCR_LSEON          0
#define RCC_BDCR_LSERDY         1
#define RCC_BDCR_LSEBYP         2
#define RCC_BDCR_LSEMOD         3
#define RCC_BDCR_RTCSEL         8
#define RCC_BDCR_RTCEN          15
#define RCC_BDCR_BDRST          16

/* Bit definitions macros */
#define RCC_AHB1ENR_GPIOA_EN         0
#define RCC_AHB1ENR_GPIOB_EN         1
#define RCC_AHB1ENR_GPIOC_EN         2
#define RCC_AHB1ENR_GPIOD_EN         3
#define RCC_AHB1ENR_GPIOE_EN         4
#define RCC_AHB1ENR_GPIOF_EN         5
#define RCC_AHB1ENR_GPIOG_EN         6
#define RCC_AHB1ENR_GPIOH_EN         7
#define RCC_AHB1ENR_CRC_EN           12
#define RCC_AHB1ENR_BKPSRAM_EN       18
#define RCC_AHB1ENR_DMA1_EN          21
#define RCC_AHB1ENR_DMA2_EN          22
#define RCC_AHB1ENR_OTGHS_EN         29
#define RCC_AHB1ENR_OTGHSULPI_EN     30

/* Register Bit definition macros corresponding to RCC_APB1ENR Register */
#define RCC_APB1ENR_TIM6_EN          4
#define RCC_APB1ENR_TIM7_EN          5
#define RCC_APB1ENR_PWR_EN           28

/* Register Bit definition macros corresponding to RCC_PLLCFGR Register */
#define RCC_PLLCFGR_PLLM             0
#define RCC_PLLCFGR_PLLN             6
#define RCC_PLLCFGR_PLLP             16
#define RCC_PLLCFGR_PLLSRC           22
#define RCC_PLLCFGR_PLLQ             24
#define RCC_PLLCFGR_PLLR             28






/* Register Bit definition macros corresponding to RCC_CFGR Register */
#define RCC_CFGR_SW                  0
#define RCC_CFGR_SWS                 2
#define RCC_CFGR_HPRE                4
#define RCC_CFGR_PPRE1               10
#define RCC_CFGR_PPRE2               13


/* macro corresponding to apb1 prescaler clock division*/
#define RCC_AHB1_CLK_DIV1()          ( RCC -> RCC_CFGR &= ~(0x7U << RCC_CFGR_PPRE1) )
#define RCC_AHB1_CLK_DIV2()          ( RCC -> RCC_CFGR |= (0x4U << RCC_CFGR_PPRE1) )
#define RCC_AHB1_CLK_DIV4()          ( RCC -> RCC_CFGR |= (0x5U << RCC_CFGR_PPRE1) )
#define RCC_AHB1_CLK_DIV8()          ( RCC -> RCC_CFGR |= (0x6U << RCC_CFGR_PPRE1) )
#define RCC_AHB1_CLK_DIV16()         ( RCC -> RCC_CFGR |= (0x7U << RCC_CFGR_PPRE1) )


/* Flag bits for RCC CFGR SWS bits */
#define RCC_CFGR_SWS_HSI               0
#define RCC_CFGR_SWS_HSE               1
#define RCC_CFGR_SWS_PLL               2
#define RCC_CFGR_SWS_PLLR              3


/* Register Bit definition macros corresponding to RCC_CR Register */
#define RCC_CR_HSION                0
#define RCC_CR_HSIRDY               1
#define RCC_CR_HSEON                16
#define RCC_CR_HSERDY               17
#define RCC_CR_HSEBYP               18
#define RCC_CR_PLLON                24
#define RCC_CR_PLLRDY               25


#define TIMx_SR_UIF                  0

#define TIMx_CR1_CEN                 0
#define TIMx_CR1_URS                 2

#define TIMx_DIER_UIE                0

#define PWR_CR_VOS                   14

/* Register Bit definition macros corresponding to RCC_CR Register */
#define FLASH_ACR_LATENCY            0
#define FLASH_ACR_PRFTEN             8
#define FLASH_ACR_ICEN               9
#define FLASH_ACR_DCEN               10
#define FLASH_ACR_ICRST              11
#define FLASH_ACR_DCRST              12

/*************************************************************************************************************/

//#include"stm32f44xx_gpio_driver.h"//gpio device specific header file
//#include"stm32f44xx_i2c_driver.h"//i2c device specific header file
//#include"stm32f44xx_spi_driver.h"//spi device specific header file
//#include"stm32f44xx_usart_driver.h"//usart and uart both device specific header file
//#include"stm32f44xx_rtc_driver.h"




#endif /* INC_STM32F446XX_H_ */
