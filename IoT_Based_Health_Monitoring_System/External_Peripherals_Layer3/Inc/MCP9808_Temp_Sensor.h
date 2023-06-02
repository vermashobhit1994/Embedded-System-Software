/*
 * MCP9808_Temp_Sensor.c
 *
 *  Created on: 16-Jul-2021
 *      Author: vermas
 */

/* Description : Header file for MCP9808 digital temperature sensor */

#ifndef __MCP9808_TEMP_SENSOR__
#define __MCP9808_TEMP_SENSOR__

#include<float.h>
#include<stdint.h>
#include"Error_Handling.h"
#include"stm32f44xx_i2c_driver.h"
#include<stdbool.h>
#include<string.h>

#include"delay.h"


#include"MCP9808_Config.h"

//Here ADDR0 = ADDR1 = ADDR2 = 0
//#define MCP9808_SLAVE_ADDR      0x18


#define FLOAT_MIN        -41.00


#define N_DECIMAL_POINTS_PRECISION     100

/* Address for each register to select the register using the
 * Register Pointer register.
 * Unnamed enum
 */
typedef enum
{

	MCP9808_REG_ADDR_CONFIG = 0x01,
	MCP9808_REG_ADDR_ALERT_TEMP_UPPER,
	MCP9808_REG_ADDR_ALERT_TEMP_LOWER,
	MCP9808_REG_ADDR_CRITICAL_TEMP_TRIP,
	MCP9808_REG_ADDR_AMBIENT_TEMP,
	MCP9808_REG_ADDR_MANUFACTURE_ID,
	MCP9808_REG_ADDR_DEVICE_ID,
	MCP9808_REG_ADDR_RESOLUTION

}MCP9808_REG_ADDR;


/* structure used to write the actual data to register */
typedef struct
{

	uint16_t MCP9808_REG_CONFIG;
	uint16_t MCP9808_REG_ALERT_TEMP_UPPER;
	uint16_t MCP9808_REG_ALERT_TEMP_LOWER;
	uint16_t MCP9808_REG_CRITICAL_TEMP_TRIP;
	uint16_t MCP9808_REG_AMBIENT_TEMP;
	uint16_t MCP9808_REG_MANUFACTURE_ID;
	uint16_t MCP9808_REG_DEVICE_ID;
	uint8_t  MCP9808_REG_RESOLUTION ;

}MCP9808_REG;


/*
// bit definition corresponding to Config Register
// Here taken as structure so that we can write more than one bit
typedef struct
{
	uint8_t MCP9808_REG_CONFIG_ALERT_MODE:1;
	uint8_t MCP9808_REG_CONFIG_ALERT_POL:1;
	uint8_t MCP9808_REG_CONFIG_ALERT_SEL:1;
	uint8_t MCP9808_REG_CONFIG_ALERT_CTRL:1;
	uint8_t MCP9808_REG_CONFIG_ALERT_STAT:1;
	uint8_t MCP9808_REG_CONFIG_INT_CLEAR:1;
	uint8_t MCP9808_REG_CONFIG_TUPPER_TLOWER_LOCK:1;
	uint8_t MCP9808_REG_CONFIG_TCRIT_LOCK:1;
	uint8_t MCP9808_REG_CONFIG_SHDN:1;
	uint8_t MCP9808_REG_CONFIG_THYST:2;
}MCP9808_REG_CONFIG_BITS;
*/

/*Bit position Macro for configuration register via enum */
typedef enum
{
	MCP9808_REG_CONFIG_BIT_POSITION_ALERT_MODE=0,
	MCP9808_REG_CONFIG_BIT_POSITION_ALERT_POL,
	MCP9808_REG_CONFIG_BIT_POSITION_ALERT_SEL,
	MCP9808_REG_CONFIG_BIT_POSITION_ALERT_CTRL,
	MCP9808_REG_CONFIG_BIT_POSITION_ALERT_STATUS,
	MCP9808_REG_CONFIG_BIT_POSITION_INT_CLEAR,
	MCP9808_REG_CONFIG_BIT_POSITION_TUPPER_TLOWER_LOCK,
	MCP9808_REG_CONFIG_BIT_POSITION_TCRIT_LOCK,
	MCP9808_REG_CONFIG_BIT_POSITION_SHDN,
	MCP9808_REG_CONFIG_BIT_POSITION_THYST
}MCP9808_REG_CONFIG_BIT_POSITION;




/* Bit corresponding to Ambient temperature(T_A) register
 * Here used Macro since only one bit is used.*/
#define MCP9808_REG_BIT_POSITION_SIGN                       12
#define MCP9808_REG_T_A_BIT_POSITION_TLOWER                 13
#define MCP9808_REG_T_A_BIT_POSITION_TUPPER                 14
#define MCP9808_REG_T_A_BIT_POSITION_TCRITICAL              15


/* values corresponding to resolution Register */
#define	    MCP9808_REG_RESOLUTION_0_5                    0
#define 	MCP9808_REG_RESOLUTION_0_25                   1
#define 	MCP9808_REG_RESOLUTION_0_125                  2
#define	    MCP9808_REG_RESOLUTION_0_0625                 3




#define	MCP9808_MODE_CONTINUOUS_CONVERSION         0
#define MCP9808_MODE_SHUTDOWN                      1



#define	MCP9808_ALERT_OUTPUT_DI              0
#define	MCP9808_ALERT_OUTPUT_EN              1



#define	    MCP9808_ALERT_OUTPUT_POLARITY_LOW      0
#define 	MCP9808_ALERT_OUTPUT_POLARITY_HIGH     1



#define	MCP9808_ALERT_OUTPUT_MODE_COMPARATOR        0
#define	MCP9808_ALERT_OUTPUT_MODE_INTERRUPT             1


#define	    MCP9808_ALERT_OUTPUT_TUP_TLOW_TCRITICAL  0
#define 	MCP9808_ALERT_OUTPUT_TA_TCRITICAL        1

/* Alert Output Status macro*/
#define MCP9808_ALERT_OUTPUT_STATUS_OFF         0
#define	MCP9808_ALERT_OUTPUT_STATUS_ON          1


#define 	MCP9808_CRITICAL_TEMP_LOCK_OFF      0
#define  	MCP9808_CRITICAL_TEMP_LOCK_ON       1



#define	   MCP9808_TEMP_LIMIT_LOCK_OFF             0
#define	   MCP9808_TEMP_LIMIT_LOCK_ON              1




/* structure to store the application configuration parameters*/
typedef struct
{

	I2C_Handle_t *pI2CHandle;


	float Lower_Alert_Temp_In_Celsius;
	float Higher_Alert_Temp_In_Celsius;
	float Critical_Temp_In_Celsius;

	//slave address
	uint8_t MCP9808_SlaveAddr;

	/* Configure related to Configuration register*/
	uint8_t TempResolutionVal;

	bool AlertOutputCtrl;
	bool AlertOutputPolarity;
	bool AlertOuputMode;
	bool AlertOutputSelect;


	/* To prevent accidental rewrite to TLower , TUpper, Tcritical*/
	bool CriticalTempLockStatus;
	bool TempLimitLockStatus;

}MCP9808_Param_Config_t;



/*
typedef struct
{

	//GPIO port and pin for sda
	GPIO_RegDef_t *MCP9808_PORT_SDA;
	uint8_t MCP9808_GPIOPin_SDA;

	//GPIO port and pin for scl
	GPIO_RegDef_t *MCP9808_PORT_SCL;
	uint8_t MCP9808_GPIOPin_SCL;


	//GPIO port and pin for Alert
	GPIO_RegDef_t *MCP9808_PORT_ALERT;
	uint8_t MCP9808_GPIOPin_ALERT;

	I2C_RegDef_t *MCP9808_I2Cx;

}MCP9808_GPIOPins_I2Cx_Config;
*/

/* global declaration for I2C handle structure for temperature sensor*/
I2C_Handle_t Temperature_Sensor_I2CHandle;

/* global declaration for MCP9808_Param_Config_t structure for temperature sensor*/
MCP9808_Param_Config_t Temperature_SensorParam;


/* Prototype for API's used */

void MCP9808_Param_Init(void);

void MCP9808_Temp_Sensor_Init(MCP9808_Param_Config_t *pParamConfig);
void MCP9808_Convert_Temperature_To_Register_Data(float Data, uint16_t *pRegisterData);
float MCP9808_Calculate_Avg_Temperature_Value(MCP9808_Param_Config_t *pParamReg, uint32_t No_Of_Samples);
float MCP9808_Read_TempValue_In_Celsius(MCP9808_Param_Config_t *ParamConfig);

Error_Status MCP9808_Read_Reg(I2C_Handle_t *pI2CHandle, MCP9808_REG_ADDR RegAddr, uint8_t *pRxBuffer, uint8_t MCP9808_SlaveAddr);
Error_Status MCP9808_Sensor_Write_Register(I2C_Handle_t *pI2CHandle,  MCP9808_REG_ADDR MCP9808_RegAddr,uint16_t data, uint8_t MCP9808_SlaveAddr);

float MCP9808_Convert_Celsius_To_Farenheit(float TempCelsius);




#endif


