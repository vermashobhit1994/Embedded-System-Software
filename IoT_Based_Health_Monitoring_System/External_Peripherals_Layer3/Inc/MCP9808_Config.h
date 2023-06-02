/*
 * MCP9808_Pins_Port_Config.h
 *
 *  Created on: 20-Jul-2021
 *      Author: vermas
 */

/* file that modify the pin and port used .
 * To use the user specified port and pins for SCL and SDA line
 * for MCP9808 temperature sensor first Init with __MCP808_INIT__
 * as below
 * #define __MCP808_INIT__ */

#ifndef MCP9808_TEMP_SENSOR_INC_MCP9808_CONFIG_H_
#define MCP9808_TEMP_SENSOR_INC_MCP9808_CONFIG_H_


/* Default port and pins for SDA and SCL line*/
#ifndef __MCP808_INIT__
#define __MCP808_INIT__

#define MCP9808_GPIO_PORT_SCL             GPIOB
#define MCP9808_GPIO_PIN_SCL              GPIO_PIN_NO_10

#define MCP9808_GPIO_PORT_SDA             GPIOB
#define MCP9808_GPIO_PIN_SDA              GPIO_PIN_NO_11

//alternate function mode number
#define MCP9808_ALT_FUNC_MODE             4

//I2C line used
#define MCP9808_I2Cx                      I2C2

//Slave address
#define MCP9808_SLAVE_ADDR                0x18

#endif

/* default value for temperature range */
#ifndef __MCP9808_PARAM_INIT__
#define __MCP9808_PARAM_INIT__

#define TEMP_HIGH_LIMIT              100
#define TEMP_LOW_LIMIT               -20
#define TEMP_CRITICAL                70
#define TEMP_RESOLUTION              MCP9808_REG_RESOLUTION_0_125
#define TEMP_ALERT_O_P_CTRL          MCP9808_ALERT_OUTPUT_EN
#define TEMP_ALERT_O_P_POLARITY      MCP9808_ALERT_OUTPUT_POLARITY_LOW
#endif

#endif /* MCP9808_TEMP_SENSOR_INC_MCP9808_CONFIG_H_ */
