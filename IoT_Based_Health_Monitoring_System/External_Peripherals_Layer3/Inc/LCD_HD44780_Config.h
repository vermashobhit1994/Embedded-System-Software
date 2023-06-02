/*
 * LCD_HD44780_Config.h
 *
 *  Created on: 20-Jul-2021
 *      Author: vermas
 */

/* Description : Configuration parameters relatead to I2C LCD */

#ifndef LCD_HD44780_I2C_DRIVER_INC_LCD_HD44780_CONFIG_H_
#define LCD_HD44780_I2C_DRIVER_INC_LCD_HD44780_CONFIG_H_

////////////////////////////////////////////////////////////////
/////////////////// Default Pins Used in I2C /////////
/////////////////// Can be changed as per user need /////////

#ifndef __LCD_I2C_HD44780_INIT__
#define __LCD_I2C_HD44780_INIT__

#define LCD_I2Cx_SCL_PORT               GPIOB
#define LCD_I2Cx_SCL_PIN                GPIO_PIN_NO_6

#define LCD_I2Cx_SDA_PORT               GPIOB
#define LCD_I2Cx_SDA_PIN                GPIO_PIN_NO_7

#define LCD_I2Cx                      I2C1

#define LCD_I2Cx_ALTERNATE_FUNC_MODE      4

#endif


#ifndef __LCD_I2C_HD44780_PARAMS_INIT__
#define __LCD_I2C_HD44780_PARAMS_INIT__

//Macro related to I2C LCD
#define SLAVE_ADDR                    0x27
#define LCD_NO_OF_LINES               LCD_LINE_2
#define LCD_NO_OF_COLUMNS             LCD_COULUMN_16
#define LCD_NO_OF_CHAR_FONT           LCD_CHARACTER_FONT_5_8

#endif


#endif /* LCD_HD44780_I2C_DRIVER_INC_LCD_HD44780_CONFIG_H_ */
