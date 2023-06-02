/*
 * stm32f44xx_usart_driver.h
 *
 *  Created on: 30-Jan-2021
 *      Author: vermas
 */

#ifndef INC_STM32F44XX_USART_DRIVER_H_
#define INC_STM32F44XX_USART_DRIVER_H_

#include"stm32f446xx.h"
#include"stm32f44xx_rcc_driver.h"
#include<string.h>



//User configuration structure
typedef struct
{
	uint8_t USART_Mode;//possible options from @USART_Modes
	uint32_t USART_BAUD_RATE;//Possible options from @USART_Baud
	uint8_t USART_NoOfStopBits;//Possible options from @USART_NoOfStopBits
	uint8_t USART_WordLength;//Possible options from @USART_WordLength
	uint8_t USART_ParityCtrl;//Possible options from @USART_ParityControl
	uint8_t USART_HWFlowCtrl;//Possible options from @USART_HWFlowControl


}USART_Config_t;

/* structure to hold the parameters related to data to be sent or received*/
typedef struct
{
	//since 1 byte of data transfer/receive each time
	uint8_t *pTxData;
	uint32_t TxLen;
	uint8_t  *pRxData;
	uint32_t RxLen;
}USARTx_Param_Config;

//Handle Structure
typedef struct
{
	USART_RegDef_t *pUSARTx;
	USART_Config_t USART_Config;

}USART_Handle_t;

/* Some other macros for checking if flag in status register is set or reset*/
#define USART_FLAG_SET         1
#define USART_FLAG_RESET       0


/* @USART_Modes*/
#define USART_MODE_ONLY_TX      0
#define USART_MODE_ONLY_RX      1
#define USART_MODE_TXRX         2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					2400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define USUART_STD_BAUD_3M 					3000000


/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE   0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1


/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3


/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/* Macros for enable /disable Tx and Rx interrupt*/
#define USART_INTERRUPT_TX_EN       1
#define USART_INTERRUPT_TX_DI       2
#define USART_INTERRUPT_RX_EN       3
#define USART_INTERRUPT_RX_DI       4



/********************************************************************************************************************
 *             API's Supported by this driver
 *
 */

//Enable or Disable peripheral clock
void USART_PeriClockCtrl(USART_RegDef_t *pUSARTx, uint8_t ENOrDI);

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

/*
 * Data Send and Receive
 */
void USARTx_Transmit_Byte_Data(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint32_t USARTx_Receive_Byte_Data(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer,uint32_t RxBufferLen);
//uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
//uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENOrDI);
void USART_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
//void USART_IRQHandling(USART_Handle_t *pHandle);
void USART_TX_OR_RX_Byte_IRQConfig(USART_Handle_t *pUSARTHandle);
void USART_Interrupt_Config(USART_Handle_t *pUSARTHandle, uint8_t EnOrDi);



/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx,uint8_t ENOrDI);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx,uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx,uint32_t FlagName);


/*
 * Application callback
 */
//void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);
void USART_Transmit_Byte_Data_Callback(USART_Handle_t *pUSARTHandle,USARTx_Param_Config *pUSARTParam);
void USART_Receive_Byte_Data_Callback(USART_Handle_t *pUSARTHandle,USARTx_Param_Config *pUSARTParam);





/************************************************************************/





#endif /* INC_STM32F44XX_USART_DRIVER_H_ */
