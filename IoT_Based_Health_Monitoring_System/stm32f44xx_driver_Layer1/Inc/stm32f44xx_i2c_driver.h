/*
 * stm32f446xx_i2c_driver.h
 *
 *  Created on: 18-Sep-2020
 *      Author: vermas
 */


#ifndef INC_STM32F44XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

//The below file must be included in every specific driver*/
#include"stm32f446xx.h"//include the device specific header file


#define COMMON_HEADER_FILES
#ifdef COMMON_HEADER_FILES
/* user configuration structure*/
typedef struct
{
	uint32_t I2C_SCLSpeed;/* Possible values from @I2C_SCL_Speed*/
	uint8_t I2C_DeviceAddress;/* slave Device Address of 7 bit . Given by user*/
	uint8_t I2C_ACKCtrl;/*Possible values from @I2C_ACK_Ctrl*/
	uint16_t I2C_FMDUTYCYCL;/* Possible values from @I2C_FM_DUTY_CYCLE*/


}I2C_Config_t;


/* Adding the variables for handling the interrupt */
typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;

	/* variables Used in Interrupt mode */
	uint8_t      *pTxBuffer; /* To store the user Tx buffer address */
	uint8_t      *pRxBuffer;/* To store the user Rx buffer address */
	uint32_t      TxLen;/*  To store the user Tx buffer length */
	uint32_t      RxLen;/*  To store the user Rx buffer length */
	uint8_t      TxRxState;/* Store the state of device . Get the details from @I2C Application states*/
	uint8_t      DevAddr; /* to store the slave/device address */
	uint32_t      RxSize;/* To store the receive data size */
	uint8_t      SrValue;/* To store the repeated start value*/

}I2C_Handle_t;

#endif

#undef COMMON_HEADER_FILES

/*I2C clock general and fast speed
 * @I2C_SCL_Speed
 * */
#define I2C_SCL_SPEED_ST_MODE               100000  /* standard mode speed i.e 100KHz*/
#define I2C_SCL_SPEED_FAST_MODE_400K          400000 /* Fast mode speed i.e 400KHz*/
/* added additionally */
#define I2C_SCL_SPEED_FAST_MODE_200K          200000 /* Fast mode speed i.e 200KHz*/

/* I2C Ack Control.
 * @I2C_ACK_Ctrl
 */
#define I2C_ACK_EN                          1
#define I2C_ACK_DI                          0

/* I2C Ack Control.
 * @I2C_FM_DUTY_CYCLE
 */
#define I2C_FM_DUTY2                        0
#define I2C_FM_DUTY16_9                     1


/* I2C status flags for I2C_SR1 Register*/
#define I2C_FLAG_SB                 (1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR               (1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF                (1 << I2C_SR1_BTF)
#define I2C_FLAG_ADD10              (1 << I2C_SR1_ADD10)
#define I2C_FLAG_STOPF              (1 << I2C_SR1_STOPF)
#define I2C_FLAG_RxNE               (1 << I2C_SR1_RXNE)
#define I2C_FLAG_TXE                (1 << I2C_SR1_TXE)
#define I2C_FLAG_BERR               (1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO               (1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF                 (1 << I2C_SR1_AF)
#define I2C_FLAG_OVR                (1 << I2C_SR1_OVR)
#define I2C_FLAG_TIMEOUT            (1 << I2C_SR1_TIMEOUT)



//some flag set and reset
#define I2C_FLAG_SET                1
#define I2C_FLAG_RESET              0


//@I2C Application states
/* taken from : */
#define I2C_READY                    0
#define I2C_BUSY_IN_RX               1
#define I2C_BUSY_IN_TX               2


//I2C Repeated start condition flags
//@I2C Repeated Start
#define I2C_DISABLE_SR               0
#define I2C_ENABLE_SR                1


/* I2C application event Macros */
#define I2C_EVENT_TX_COMPLETE        0
#define I2C_EVENT_RX_COMPLETE        1
#define I2C_EVENT_STOPF              2

/* I2C application Error Macros */
#define I2C_ERROR_BERR               3
#define I2C_ERROR_ARLO               4
#define I2C_ERROR_AF                 5
#define I2C_ERROR_OVR                6
#define I2C_ERROR_TIMEOUT            7


/* I2C Application Repeat Start Macros */
#define I2C_ENABLE_SR               1
#define I2C_DISABLE_SR              0


/* I2C master read or Write data from slave flag */
#define I2C_MASTER_TRANSMITTER      0
#define I2C_MASTER_RECEIVER         1

/* Timeout when the blocking api driver is used */
#define I2C_MASTER_TIMOUT           1000

/* Control bit for repeat start condition */
#define REPEATE_START_CONDITION_DISABLE            0
#define REPEATE_START_CONDITION_ENABLE             1



/********************************************************************************************************************
 *             API's Supported by this driver
 *
 */
/* Peripheral Clock  */
void I2C_PeriClock_Ctrl(I2C_RegDef_t *pI2Cx,uint8_t ENOrDI);//enable and disable the peripheral clock

/*Init and Deinit*/
void I2C_Init(I2C_Handle_t *pI2CHandle); //initialize the GPIO Port and Pin
void I2C_DEInit(I2C_RegDef_t *pI2Cx);//deinitialize the I2C Port i.e reset state

/*Master Sending and Receiving Data to/from slave*/
Error_Status I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,uint32_t len,uint8_t SlaveAddr,uint8_t RepeatedStartConditionCtrl);
Error_Status I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,uint32_t maxlen,uint8_t SlaveAddr);


/*IRQ Configuration and ISR Handling*/
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENOrDI);//to configure the Interrupt i.e IRQ no of I2C
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);//to configure the priority of I2C
/* Api's related to IRQ Handling */
//void I2C_EVENT_IRQHandling(I2C_Handle_t *pI2CHandle);
//void I2C_ERR_IRQHandling(I2C_Handle_t *pI2CHandle);



/* Master sending and receiving data via the Interrupt*/
/* Return the state of device*/
uint8_t I2C_MasterSendData_Interrupt(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,uint32_t len,uint8_t SlaveAddr,uint8_t SrValue);
uint8_t I2C_MasterReceiveData_Interrupt(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,uint32_t len,uint8_t SlaveAddr,uint8_t SrValue);



//enable or Disable the I2C Peripheral. This must be done after the configurations in I2C_CR1 is done.
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi);

//checking if I2C Communication ends or not
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint32_t FlagName);

/* Application callback for interrupt based API's */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2C1LCDHandle, uint8_t AppEv);


//void Initialise_SlaveAddr(I2C_Handle_t *pI2CHandle,uint8_t SlaveAddr);
//void Deinitialize_SlaveAddr(I2C_Handle_t *pI2CHandle);


//void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,uint32_t maxlen,uint8_t SlaveAddr);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

uint32_t RCC_GetPCLK1Value(void);




#endif /* INC_STM32F44XX_I2C_DRIVER_H_ */
