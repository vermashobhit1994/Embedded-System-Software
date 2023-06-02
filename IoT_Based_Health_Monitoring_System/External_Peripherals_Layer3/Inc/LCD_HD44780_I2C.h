/*
 * LCD_HD44780.h
 *
 *  Created on: 20-Jan-2021
 *      Author: vermas
 */







#ifndef INC_LCD_HD44780_H_
#define INC_LCD_HD44780_H_

#ifdef COMMON_HEADER_FILES
#include"../../stm32f44xx_driver_Layer1/Inc/delay.h"


#endif

#include<stdint.h>//for uint8_t
#include<string.h>//for strlen()

#include"LCD_HD44780_Config.h"

/* Pins connections corresponding to LCD and PCF8574 module
 * PCF8574 -> P0 Pin - LCD RS Pin
 * PCF8574 -> P1 Pin - LCD R/W Pin
 * PCF8574 -> P2 Pin - LCD EN Pin    //used for Starting data read/write
 * PCF8574 -> P3 Pin - +Vcc via 4.7K resistor. (Must always high to turn Backlight ON)
 * PCF8574 -> P4 Pin - LCD D4 Pin
 * PCF8574 -> P5 Pin - LCD D5 Pin
 * PCF8574 -> P6 Pin - LCD D6 Pin
 * PCF8574 -> P7 Pin - LCD D7 Pin
 */



/* Bit Macros corresponding to Pins connected to LCD via PCF8574 Module  */

/* Lower 4 bits.*/
#define LCD_BIT_RS                 ((uint8_t)0x01U)//since RS pin connected to P0 Pin
#define LCD_BIT_RW                 ((uint8_t)0x02U)//since R/W pin connected to P1 Pin
#define LCD_BIT_E                  ((uint8_t)0x04U)//since R/W pin connected to P1 Pin
#define LCD_BIT_BACKIGHT_ON        ((uint8_t)0x08U)//since connected to P3 Pin
#define LCD_BIT_BACKIGHT_OFF       ((uint8_t)0x00U)//since connected to P3 Pin.


// check Set CGRAM addres entry of Table No 6 on Page 24 of LCD HD44780 datasheet
//Here ACG = 0, Note : It is 7 bit only
#define LCD_BIT_SETCGRAMADDR       ((uint8_t)0x40U)

// check Set DDRAM addres entry of Table No 6 on Page 24 of LCD HD44780 datasheet
//Here ADD = 0, Note : It is 6bit only
#define LCD_BIT_SETDDRAMADDR       ((uint8_t)0x80U)



//Busy Flag i.e the DB7 bit
#define LCD_BIT_BUSY_FLAG          ((uint8_t)0x80U)

/************ Bits Corresponding to commands *******************************************/
//Check the entry of Display ON/OFF control in table no 6 of datasheet
#define LCD_BIT_DISPLAY_CTRL        ((uint8_t)0x08U)
#define LCD_BIT_DISPLAY_ON          ((uint8_t)0x04U)
#define LCD_BIT_DISPLAY_OFF         ((uint8_t)0x00U)
#define LCD_BIT_CURSOR_ON           ((uint8_t)0x02U)
#define LCD_BIT_CURSOR_OFF          ((uint8_t)0x00U)
#define LCD_BIT_CURSOR_BLINK_ON     ((uint8_t)0x01U)
#define LCD_BIT_CURSOR_BLINK_OFF    ((uint8_t)0x00U)

//Check for Clear display entry in table no 6 of datasheet
#define LCD_BIT_DISPLAY_CLEAR       ((uint8_t)0x01U)//clear the display

//Check fo



// Return Home entry in table no 6 of datasheet
#define LCD_BIT_CURSOR_HOME         ((uint8_t)0x02U)//cursor is at 1st row and 1st column

//Check for Entry mode set entry in table no 6 of datasheet
#define LCD_BIT_ENTRY_MODE                                 ((uint8_t)0x04U)//entry mode control bit
#define LCD_BIT_WITH_READ_WRITE_RIGHT                      ((uint8_t)0x02U)//move the cursor direction to right
#define LCD_BIT_WITH_READ_WRITE_LEFT                      ((uint8_t)0x00U)//move the cursor direction to right
#define LCD_BIT_WITH_READ_WRITE_SHIFT_INCREMENT            ((uint8_t)0x01U)//move the cursor direction to left
#define LCD_BIT_WITH_READ_WRITE_SHIFT_DECREMENT              ((uint8_t)0x00U)//shift the display while read /write data from LCD


//check for Cursor or display shift entry in table no 6 of datasheet
//Here No change of DDRAM contents.
#define LCD_BIT_CURSOR_DISPLAY_SHIFT                       ((uint8_t)0x10U)//cursor or display shift control bit
#define LCD_BIT_WITHOUT_READ_WRITE_DISPLAY_SHIFT           ((uint8_t)0x08U)//shift the display
#define LCD_BIT_WITHOUT_READ_WRITE_CURSOR_MOVE             ((uint8_t)0x00U)//shift the cursor
#define LCD_BIT_WITHOUT_READ_WRITE_DIR_LEFT                ((uint8_t)0x00U)//shift the cursor / display to left
#define LCD_BIT_WITHOUT_READ_WRITE_DIR_RIGHT               ((uint8_t)0x04U)//shift the cursor /display to right

////////////////// //// bits for Function set entry in table no 6 of datasheet
// check Function set entry of Table No 6 on Page 24 of LCD HD44780 datasheet
#define LCD_BIT_FUNCTION_SET       ((uint8_t)0x20U)//control bit for function set field
//DL -> 0 -> 4 bit mode, 1 -> 8 bit mode. Since it's I2C so only work in 4 bit mode
#define LCD_BIT_DATA_LENGTH_4BITS             ((uint8_t)0x00U)/* To use either 4 pins or 8 pins. Here using only high 4 bits. */

//no of line bits
//N = 1 -> 2 lines , 0 -> 1 line
#define LCD_BIT_1LINE              ((uint8_t)0x00U)
#define LCD_BIT_2LINE              ((uint8_t)0x08U)
#define LCD_BIT_4LINE              LCD_BIT_2LINE

//character font bits
//F = 1 -> 5x10 dots, 0 -> 5x8 dots
#define LCD_BIT_5x8DOTS            ((uint8_t)0x00U)
#define LCD_BIT_5x10DOTS           ((uint8_t)0x04U)

/*************************************************************************************/

// Structure to hold I2C LCD Parameters
typedef struct {
    I2C_Handle_t *pi2handle;  // Pointer to I2C struct
    uint8_t no_of_lines;       // Lines of the display
    uint8_t no_of_columns;     // no of Columns
    uint8_t slaveaddr;         // I2C slave address shifted left by 1
    uint8_t backlight_status;  // Backlight status can be ON or OFF.

    // Display on/off, cursor on/off, blinking cursor on/off control bits
    //check the Display on/off control field of table no 6 of datasheet.
    uint8_t displayCtrlBits;

    // Entry mode set bits for cursor direction(right or left) and display shift
    //check the Entry mode set field of table no 6 of datasheet.
    uint8_t entryModeBits;

    //for shifting the cursor or display or both without change the DDRAM contents
    //check the Cursor or display shift entry in table 6 of datasheet.
    uint8_t cursorOrDisplayShiftBits;

}LCDParams;


/* Create a enum to store the commands for LCD*/
typedef enum {
    LCD_BACKLIGHT = 0,//Control the LCD backlight. Can be ON or OFF
    LCD_DISPLAY,//Control the LCD Display. Can be ON or OFF
    LCD_CLEAR,//Clear the LCD
    LCD_CURSOR,//Cursor can be move right or left.
    LCD_CURSOR_BLINK,//Cursor blink can be turn ON or OFF.
    LCD_CURSOR_HOME,//Return the cursor to starting position.
    LCD_CURSOR_SHIFT_WITH_READ_WRITE_DIR_LEFT,//move cursor left when read/write data
    LCD_CURSOR_SHIFT_WITH_READ_WRITE_DIR_RIGHT,//move cursor right when read /write data
    LCD_DISPLAY_SHIFT_WITH_READ_WRITE,//shift display when read /write data


	LCD_DISPLAY_SHIFT_WITHOUT_READ_WRITE,//
	LCD_CURSOR_SHIFT_WITHOUT_READ_WRITE,//

	LCD_CURSOR_OR_DISPLAY_SHIFT_WITHOUT_READ_WRITE_DIR_RIGHT,//
	LCD_CURSOR_OR_DISPLAY_SHIFT_WITHOUT_READ_WRITE_DIR_LEFT//

} LCD_HD44780_I2C_Cmds;

//To set and clear the LCD Parameters
//To turn on and off the lcd display , cursor , cursor blink etc
typedef enum {
    LCD_PARAM_UNSET = 0,
    LCD_PARAM_SET
} LCD_HD44780_I2C_Params_Actions;

/* Enum to store the direction for text to be rotated */
typedef enum
{
	DIRECTION_CLOCKWISE,
	DIRECTION_ANTICLOCKWISE,
	DIRECTION_NONE//no rotation of text
}RotateDirection;


/* --------------------- Macros corresponding to commands given to user------------- */
//display controls
#define LcdI2CDisplayON()                  LCD_I2C_HD44780_Send_Cmd(LCD_DISPLAY, LCD_PARAM_SET)
#define LcdI2CDisplayOFF()                 LCD_I2C_HD44780_Send_Cmd(LCD_DISPLAY, LCD_PARAM_UNSET)
#define LcdI2CDisplayShiftReadWriteON()    LCD_I2C_HD44780_Send_Cmd(LCD_DISPLAY_SHIFT_WITH_READ_WRITE, LCD_PARAM_SET)
#define LcdI2CDisplayShiftReadWriteOFF()   LCD_I2C_HD44780_Send_Cmd(LCD_DISPLAY_SHIFT_WITH_READ_WRITE, LCD_PARAM_UNSET)

//cursor direction controls
#define LcdI2CCursorDirToRight()      LCD_I2C_HD44780_Send_Cmd(LCD_CURSOR_SHIFT_WITH_READ_WRITE_DIR_RIGHT, LCD_PARAM_SET)
#define LcdI2CCursorDirToLeft()       LCD_I2C_HD44780_Send_Cmd(LCD_CURSOR_SHIFT_WITH_READ_WRITE_DIR_LEFT, LCD_PARAM_SET)

//time consuming commands controls i.e display clear and return to home.
#define LcdI2CDisplayClear()          LCD_I2C_HD44780_Send_Cmd(LCD_CLEAR, LCD_PARAM_SET)
#define LcdI2CCursorHome()            LCD_I2C_HD44780_Send_Cmd(LCD_CURSOR_HOME, LCD_PARAM_SET)


//backlight controls
#define LcdI2CBacklightON()            LCD_I2C_HD44780_Backlight_Ctrl(LCD_BIT_BACKIGHT_ON)
#define LcdI2CBacklightOFF()           LCD_I2C_HD44780_Backlight_Ctrl(LCD_BIT_BACKIGHT_OFF)

//Cursor contolrs
#define LcdI2CCursorON()              LCD_I2C_HD44780_Send_Cmd(LCD_CURSOR, LCD_PARAM_SET)
#define LcdI2CCursorOFF()             LCD_I2C_HD44780_Send_Cmd(LCD_CURSOR, LCD_PARAM_UNSET)
#define LcdI2CCursorBlinkON()         LCD_I2C_HD44780_Send_Cmd(LCD_CURSOR_BLINK, LCD_PARAM_SET)
#define LcdI2CCursorBlinkOFF()        LCD_I2C_HD44780_Send_Cmd(LCD_CURSOR_BLINK, LCD_PARAM_UNSET)

//Text Direction controls
//#define LcdI2CRotateTextClockWise()
//#define LcdI2CRotateTextAntiClockWise()


/* User Define Macros*/
#define LCD_LINE_1                      1
#define LCD_LINE_2                      2

#define LCD_COULUMN_16                  16
#define LCD_CHARACTER_FONT_5_8          LCD_BIT_5x8DOTS
#define LCD_CHARACTER_FONT_5_10         LCD_BIT_5x10DOTS





/* **************** function prototypes *******************************************/
//Initialization functions
__attribute__((weak)) void I2Cx_GPIO_Pins_Init(void);
__attribute__((weak)) void I2Cx_Init(I2C_Handle_t *pI2C1Handle, uint8_t SlaveAddr);
uint8_t LCD_I2C_HD44780_Init(I2C_Handle_t *phi2c, uint8_t SlaveAddr, uint8_t NoOfLines, uint8_t Columns, uint8_t NoOfCharacterFonts );


//Functionalities related functions
uint8_t LCD_I2C_HD44780_Backlight_Ctrl(uint8_t cmd);
uint8_t LCD_I2C_HD44780_Send_Cmd(LCD_HD44780_I2C_Cmds lcd_cmd, LCD_HD44780_I2C_Params_Actions lcd_action);
uint8_t LCD_I2C_HD44780_PrintChar(uint8_t ch);
void LCD_I2C_HD44780_PrintString(char *str);

void LCD_I2C_HD44780_RotateText_WithoutReadWrite(uint8_t *pData, uint8_t len, RotateDirection direction, uint32_t shiftnum);
uint8_t LCD_I2C_HD44780_LoadCustomCharToCGRAM(uint8_t *CustomPattern, uint8_t Location) ;

uint8_t LCD_I2C_HD4470_DisplayCustomCharFromCGRAM(uint8_t Location);

uint8_t LCD_I2C_HD44780_WriteByte(uint8_t rsRwBits, uint8_t *pData);

void I2C_LCD_Init(void);

#endif /* INC_LCD_HD44780_H_ */
