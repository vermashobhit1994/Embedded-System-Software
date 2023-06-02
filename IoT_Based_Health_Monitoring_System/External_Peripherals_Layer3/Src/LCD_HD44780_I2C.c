


#include"LCD_HD44780_I2C.h"

/* create a global variable for I2C LCD */
I2C_Handle_t I2C_LCD_Handle;



/* Description : I2C LCD library with PCF8574 I/O expander*/

/* Functionalities Implemented :
 * 1. Write character to LCD.
 * 2. Write String to LCD.
 * 3. Go to a specific Cursor position
 * 4. Write Custom character.
 * 5. Read Data from LCD
 * 6. Scroll the text Right or Left.
 */

/* LCD Parameters structure variable*/
static LCDParams lcdParams;

//Buffer to store the commands to be send
uint8_t lcd_cmd_buffer[6] ={0};

//void LCD_HD44780_Send_Cmd(uint8_t cmd,I2C_Handle_t *I2C1Handle);



/* Pins mapping
 * PB6 -> I2C1_SCL
 * PB7 -> I2C1_SDA
 */
#ifdef __LCD_I2C_HD44780_INIT__


/* *********************************Documentation Section **********************************************
 * @fn                           : I2Cx_GPIO_Pins_Init
 * @brief                        : IDRnitialize the Pins for I2Cx peripheral selected
 * @return                       : None
 * @Note                         : This is default configuration if not done by user.
 *                                 Specified as weak so that user can override this function
 */
__attribute__((weak)) void I2Cx_GPIO_Pins_Init(void)
{
    GPIO_Handle_t I2CPins;
    //select the gpio port
    I2CPins.pGPIOx = LCD_I2Cx_SCL_PORT;

    //select the mode
    I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = LCD_I2Cx_ALTERNATE_FUNC_MODE;//choose the AF4
    I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;
    I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_OPEN_DRAIN;//due to I2C
    I2CPins.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_PULL_UP;//use the INternal pull up for SDA and SCL
    I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;//choose any arbitrary value

    //configure Pins for SCL
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = LCD_I2Cx_SCL_PIN;
    //put the values in Registers of GPIOx
    GPIO_Init(&I2CPins);

    //configure Pins for SDA
    I2CPins.pGPIOx = LCD_I2Cx_SDA_PORT;
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = LCD_I2Cx_SDA_PIN;

    //put the values in Registers of GPIOx
    GPIO_Init(&I2CPins);

}

/* *********************************Documentation Section **********************************************
 * @fn                           :
 * @brief                        :
 * @return                       :
 * @Note                         : This is default configuration if not done by user.
 *                                 Specified as weak so that user can override this function
 */
__attribute__((weak)) void I2Cx_Init(I2C_Handle_t *pI2C1LCDHandle, uint8_t SlaveAddr)
{

	//Initialize the GPIO pins to behave as I2C1
	//Here pins initialization for I2C is done first
	I2Cx_GPIO_Pins_Init();

	pI2C1LCDHandle -> pI2Cx = LCD_I2Cx;//select the I2C1

	pI2C1LCDHandle -> I2C_Config.I2C_ACKCtrl = I2C_ACK_EN;
	pI2C1LCDHandle -> I2C_Config.I2C_DeviceAddress = SlaveAddr;
	pI2C1LCDHandle -> I2C_Config.I2C_FMDUTYCYCL = I2C_FM_DUTY2 ;
	pI2C1LCDHandle -> I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_ST_MODE ;

    I2C_Init(pI2C1LCDHandle);

    //enable the I2Cx peripheral
    I2C_PeripheralControl(pI2C1LCDHandle -> pI2Cx, ENABLE);
}


#endif




/* *********************************Documentation Section **********************************************
 * @fn                           : LCD_I2C_HD44780_WriteByte
 * @brief                        : Write the 8 bit data to LCD
 * @param[in1]                   : Value of RS and R/W bits
 * @param[in2]                   : Pointer to data to be written.
 * @return                       : 1 -> success , 0 -> Failure.
 * @Note                         : Write cmd or data, First the higher 4 bits is written then lower 4 bits are written.
 */
uint8_t LCD_I2C_HD44780_WriteByte(uint8_t rsRwBits, uint8_t *pData)
{
	//higher 4 bits
	uint8_t data_u = *pData & 0xF0;
	//lower 4 bits
	uint8_t data_l = ((*pData ) & 0x0F)<<4 ;

	//higher 4 bits
	lcd_cmd_buffer[0] = rsRwBits | LCD_BIT_E | lcdParams.backlight_status | data_u;

	lcd_cmd_buffer[1] = rsRwBits | lcdParams.backlight_status | data_u;

	/* Lower 4 bits*/
	lcd_cmd_buffer[2] = rsRwBits | LCD_BIT_E | lcdParams.backlight_status | data_l;

	lcd_cmd_buffer[3] = rsRwBits | lcdParams.backlight_status | data_l;

	I2C_MasterSendData(lcdParams.pi2handle, lcd_cmd_buffer, 4, lcdParams.slaveaddr, REPEATE_START_CONDITION_DISABLE);

	//delay to write the command
	if(rsRwBits == 0x00)
		delay_us(45);

	return 1;
}


/* *********************************Documentation Section **********************************************
 * @fn                           : LCD_I2C_HD44780_GotoRowColumn
 * @brief                        : To go to cursor to particular row and column
 * @param[in1]                   : Row , valid values 1 and 2
 * @param[in1]                   : Column, valid values 1 to 16
 * @return                       :
 * @Note                         :
 */
void LCD_I2C_HD44780_GotoRowColumn(uint8_t Row, uint8_t Column)
{
	if(Column > 16 && Column < 1)
		return;
	//Here we're assuming that lines start from 0

	//starting address for 4 lines if LCD contains the maximum of 4 lines
	uint8_t LineStartAddr[4] = {0x00,0x40, 0x14, 0x54 };


	//if user specify the lines that exceed max value so, take the max value -1
	//since array index for line offset start from 0
	if (Row >= lcdParams.no_of_lines)
		Row = lcdParams.no_of_lines-1;

	//since in user column start from 1
	Column -=1;

	//Writing address to 7 bit DDRAM
	uint8_t lcd_cmd = LCD_BIT_SETDDRAMADDR | (Column + LineStartAddr[Row]);

	//check the set DDRAM address entry of table no 6 of datasheet
	LCD_I2C_HD44780_WriteByte(0x00,&lcd_cmd );
}


/* *********************************Documentation Section **********************************************
 * @fn                           : LCD_I2C_HD44780_PrintChar
 * @brief                        : Write the 1 byte data to LCD
 * @param[in1]                   : Character to be written.
 * @return                       : 1 -> Success , 0 -> failure
 * @Note                         : Here we're writing the data to DDRAM
 */
uint8_t LCD_I2C_HD44780_PrintChar(uint8_t ch)
{
	//RS = 1 since we're using DR register
	return LCD_I2C_HD44780_WriteByte(LCD_BIT_RS, &ch);
}


/* *********************************Documentation Section **********************************************
 * @fn                           : GetCurrentDDRAMADDR
 * @brief                        :
 * @param[in1]                   :
 * @return                       :
 * @Note                         :
 */
void GetCurrentDDRAMADDR(uint8_t *RxBuffer , int no_of_bytes)
{


	//Send the data with R/W bit = 1 to indicate read from LCD
	uint8_t data[2] = {0};
	data[0] = LCD_BIT_RW | LCD_BIT_E | LCD_BIT_BACKIGHT_ON;
	data[1] = LCD_BIT_RW | LCD_BIT_BACKIGHT_ON;
	I2C_MasterSendData(lcdParams.pi2handle, data, 2, lcdParams.slaveaddr, REPEATE_START_CONDITION_DISABLE);


	//Now reading the BF bit and Address counter (Current DDRAM address)

	//I2C_MasterReceiveData(lcdParams.pi2handle, RxBuffer,1, lcdParams.slaveaddr);
	//RxBuffer++;
	//I2C_MasterReceiveData(lcdParams.pi2handle, RxBuffer,1, lcdParams.slaveaddr);


	//uint8_t i = 0;
	//while(i < no_of_bytes)
	//{
		//printf("%d ",RxBuffer[i]);
		//i++;
	//}

}


/* *********************************Documentation Section **********************************************
 * @fn                           : LCD_I2C_HD44780_PrintString
 * @brief                        : Write the data to LCD byte by byte
 * @param[in1]                   : string to be written.
 * @return                       :
 * @Note                         :
 */
void LCD_I2C_HD44780_PrintString(char *str)
{

	int i = 0;
	while(i < strlen(str))
	{
		//shifting of display must be ON only when current
		//DDRAM address is at boundary+1 i.e
		//either the 1st line boundary address+1 when only 1 line is chosen i.e 0x10H i.e 16th address
		//to 4FH .
		//or at the second line boundary address+1 i.e 56H i.e 16th address to 47H
		if(i  == 16 )
		{
			//activate the display shift once the character limit exceed
			LcdI2CDisplayShiftReadWriteON();
			//delay_ms(500);
		}

		LCD_I2C_HD44780_PrintChar(str[i]);
		delay_ms(100);
		i++;
	}
	//when the control comes here then the shift becomes on so, need to turn it off
	//turn off the shifting only when the boundary address is reached
	//If single line chosen then boundary a
	LcdI2CDisplayShiftReadWriteOFF();

	//getting the current DDRAM address
	uint8_t RxBuffer[2]= {0};
	(void)RxBuffer;
	//GetCurrentDDRAMADDR(RxBuffer, 2);

}

/*Why is it written in this way?*/
/* *********************************Documentation Section **********************************************
 * @fn                           : LCD_I2C_HD44780_Send_Cmd
 * @brief                        : Write data to lcd params structure and also do clear display, return home commands
 * @param[in1]                   : Any cmd value from LCD_HD44780_I2C_Cmds enum
 * @param[in2]                   : Set or Clear the parameters.
 * @return                       : 1 -> success , 0 -> Failure.
 * @Note                         :
 */
uint8_t LCD_I2C_HD44780_Send_Cmd(LCD_HD44780_I2C_Cmds lcd_cmd, LCD_HD44780_I2C_Params_Actions lcd_action)
{
	//to store the lcd command code
	uint8_t lcd_cmd_code = 0;

	//if we set certain bit for commands
	if(lcd_action == LCD_PARAM_SET)
	{
		switch(lcd_cmd)
		{
			/* commands corresponding to display on/off control entry in table 6 of datasheet.*/
			//if cursor is ON then display also must be ON
			case LCD_CURSOR:
				lcdParams.displayCtrlBits |= LCD_BIT_CURSOR_ON | LCD_BIT_DISPLAY_ON;
				break;

			//if cursor blink is ON  then display also must be ON
			case LCD_CURSOR_BLINK:
				lcdParams.displayCtrlBits |= LCD_BIT_CURSOR_BLINK_ON | LCD_BIT_DISPLAY_ON;
				break;

			case LCD_DISPLAY:
				lcdParams.displayCtrlBits |= LCD_BIT_DISPLAY_ON;
				break;

			/**********************************************************************/

			/* Time taking commands i.e Clear display and Return Home */
			case LCD_CLEAR:
				lcd_cmd_code = LCD_BIT_DISPLAY_CLEAR;

				//Write the command
				if( LCD_I2C_HD44780_WriteByte(0x00, &lcd_cmd_code))
				{
					delay_ms(2);//time taken to execute cmd
					return 1;
				}
				else
					return 0;

			case LCD_CURSOR_HOME:
				lcd_cmd_code = LCD_BIT_CURSOR_HOME;
				//Write the command
				if( LCD_I2C_HD44780_WriteByte(0x00, &lcd_cmd_code))
				{
					delay_ms(2);//time taken to execute cmd
					return 1;
				}
				else
					return 0;
			/****************************************************************************/

			/* commands corresponding to Entry mode set entry in table 6 of datasheet.*/
			/* These commands work only when we write or read data from LCD*/
			case LCD_CURSOR_SHIFT_WITH_READ_WRITE_DIR_RIGHT:
				lcdParams.entryModeBits |= LCD_BIT_WITH_READ_WRITE_RIGHT;
				break;
			case LCD_CURSOR_SHIFT_WITH_READ_WRITE_DIR_LEFT:
				lcdParams.entryModeBits |= LCD_BIT_WITH_READ_WRITE_LEFT;
				break;
			case LCD_DISPLAY_SHIFT_WITH_READ_WRITE:
				lcdParams.entryModeBits |= LCD_BIT_WITH_READ_WRITE_SHIFT_INCREMENT;
				break;


			/**********************************************************************/

			/* commands corresponding to Cursor or display shift entry in table 6 of datasheet.*/
			/* These commands work when no write or read data from/to LCD*/
			case LCD_DISPLAY_SHIFT_WITHOUT_READ_WRITE:
				lcdParams.cursorOrDisplayShiftBits |= LCD_BIT_WITHOUT_READ_WRITE_DISPLAY_SHIFT  ;
				break;

			case LCD_CURSOR_SHIFT_WITHOUT_READ_WRITE:
				lcdParams.cursorOrDisplayShiftBits |= LCD_BIT_WITHOUT_READ_WRITE_CURSOR_MOVE;
				break;
			case LCD_CURSOR_OR_DISPLAY_SHIFT_WITHOUT_READ_WRITE_DIR_LEFT:
				lcdParams.cursorOrDisplayShiftBits |= LCD_BIT_WITHOUT_READ_WRITE_DIR_LEFT;
				break;

			case LCD_CURSOR_OR_DISPLAY_SHIFT_WITHOUT_READ_WRITE_DIR_RIGHT:
				lcdParams.cursorOrDisplayShiftBits |= LCD_BIT_WITHOUT_READ_WRITE_DIR_RIGHT;
				break;

			default://if any command not found
				return 0;

		}
	}
	//if parameters are to be cleared
	else
	{
		switch(lcd_cmd)
		{
			/* commands corresponding to display on/off control entry in table 6 of datasheet.*/
			case LCD_DISPLAY:
				lcdParams.displayCtrlBits &= ~LCD_BIT_DISPLAY_ON;
				break;
			case LCD_CURSOR:
				lcdParams.displayCtrlBits &= ~LCD_BIT_CURSOR_ON;
				break;
			case LCD_CURSOR_BLINK:
				lcdParams.displayCtrlBits &= ~LCD_BIT_CURSOR_BLINK_ON;
				break;
			/**********************************************************************/

			/* commands corresponding to Entry mode set entry in table 6 of datasheet.*/
			case LCD_CURSOR_SHIFT_WITH_READ_WRITE_DIR_RIGHT:
				lcdParams.entryModeBits &= ~LCD_BIT_WITH_READ_WRITE_RIGHT  ;
				break;
			case LCD_CURSOR_SHIFT_WITH_READ_WRITE_DIR_LEFT:
				lcdParams.entryModeBits &= ~LCD_BIT_WITH_READ_WRITE_LEFT;
				break;
			case LCD_DISPLAY_SHIFT_WITH_READ_WRITE:
				lcdParams.entryModeBits &= ~LCD_BIT_WITH_READ_WRITE_SHIFT_INCREMENT;
				break;

			/**********************************************************************/

			//commands for cursor or display shift entry in table no 6 of datasheet


			default://if any command not found
				return 0;
		}

	}

	/* Now lets send the command */
	switch (lcd_cmd) {

		//setting the DB3 bit of display on off control after selecting functionalities from display or cursor or blink
		case LCD_DISPLAY:
		case LCD_CURSOR:
		case LCD_CURSOR_BLINK:
			//turn On the display along with mode bit i.e DB3
			lcd_cmd_code = LCD_BIT_DISPLAY_CTRL | lcdParams.displayCtrlBits;
			break;

		/*setting the DB2 bit of entry mode set after selecting functionalities
		 * from display or cursor shift direction during read or write operation
		 */
		case LCD_CURSOR_SHIFT_WITH_READ_WRITE_DIR_LEFT:
		case LCD_CURSOR_SHIFT_WITH_READ_WRITE_DIR_RIGHT:
		case LCD_DISPLAY_SHIFT_WITH_READ_WRITE:
			//set the entry mode control alongwith entry bits
			lcd_cmd_code = LCD_BIT_ENTRY_MODE | lcdParams.entryModeBits;
			break;

		/*setting the DB2 bit of entry mode set after selecting functionalities
		* from display or cursor shift direction during read or write operation
		*/
		case LCD_DISPLAY_SHIFT_WITHOUT_READ_WRITE:
		case LCD_CURSOR_SHIFT_WITHOUT_READ_WRITE:
		case LCD_CURSOR_OR_DISPLAY_SHIFT_WITHOUT_READ_WRITE_DIR_RIGHT:
		case LCD_CURSOR_OR_DISPLAY_SHIFT_WITHOUT_READ_WRITE_DIR_LEFT:
			lcd_cmd_code = LCD_BIT_CURSOR_DISPLAY_SHIFT | lcdParams.cursorOrDisplayShiftBits;
			break;

		default:
			break;
	}
	return LCD_I2C_HD44780_WriteByte( (uint8_t)0x00, &lcd_cmd_code);
}

/* *********************************Documentation Section **********************************************
 * @fn                           : LCD_I2C_HD44780_RotateText_WithoutReadWrite
 * @brief                        : Rotate the text when no reading or write of data
 * @param[in1]                   : Pointer to data to be rotated.
 * @param[in2]                   : Number of character in string
 * @param[in3]                   : Direction to be rotate (clockwise or anticlockwise)
 * @param[in4]                   : Number of times shift is to be done.
 * @return                       :
 * @Note                         :
 */
void LCD_I2C_HD44780_RotateText_WithoutReadWrite(uint8_t *pData, uint8_t len, RotateDirection direction, uint32_t shiftnum)
{
	uint32_t  i =0 ;
	uint32_t OneCycleNum = 16;//number of times shift to be done in each cycle

	if(direction == DIRECTION_CLOCKWISE)
	{

		//by default cursor woud move.
		LCD_I2C_HD44780_Send_Cmd(LCD_DISPLAY_SHIFT_WITHOUT_READ_WRITE, LCD_PARAM_SET);

		//shift the cursor and display to right
		//assume string put at 1st row and 1st column
		while (i < shiftnum)
		{
			int j = 0;
			while(j <= OneCycleNum)
			{

				if( !LCD_I2C_HD44780_Send_Cmd(LCD_CURSOR_OR_DISPLAY_SHIFT_WITHOUT_READ_WRITE_DIR_RIGHT, LCD_PARAM_SET) )
				{
					break;
				}




				j++;
				delay_ms(500);
			}
			j=0;

			while(j<=39-16)
			{
				LCD_I2C_HD44780_Send_Cmd(LCD_CURSOR_OR_DISPLAY_SHIFT_WITHOUT_READ_WRITE_DIR_LEFT, LCD_PARAM_SET);
				j++;

				delay_ms(500);
			}

			/*
			j = 0;
			LCD_I2C_HD44780_Send_Cmd(LCD_CURSOR_SHIFT_WITHOUT_READ_WRITE, LCD_PARAM_SET);
			while(j <= 16)
			{
				LCD_I2C_HD44780_Send_Cmd(LCD_CURSOR_OR_DISPLAY_SHIFT_WITHOUT_READ_WRITE_DIR_LEFT, LCD_PARAM_SET);
				j++;
				delay_ms(500);
			}
			*/



			i++;
		}
	}
	else if (direction == DIRECTION_ANTICLOCKWISE)
	{

	}

}


/* *********************************Documentation Section **********************************************
 * @fn                           : LCD_I2C_HD44780_DisplayCustomChar
 * @brief                        : Function to load the custom character to CGRAM
 * @param[in1]                   : Custom pattern array
 * @param[in2]                   : Location where to write.
 * @return                       : Return the location of custom character where we can write.
 * @Note                         :
 */

uint8_t LCD_I2C_HD44780_LoadCustomCharToCGRAM(uint8_t *CustomPattern, uint8_t Location) {

    // Stop, if trying to load to incorrect cell/location
	//here we've only 8 locations from 0 to 7
	//Here the CGRAM address range -> 0x40 - 0x8F for 5x8 dots
    if (Location > 7) {
        return 0;
    }

    //since for each location 8 bits are used so, we add them to CGRAM address
    //Location << 3 add them to current CGRAM address.
    /*here address for custom character range are :
    0x40 - 0x47 -> 1st character
    0x48 - 0x4F -> 2nd character
    0x50 - 0x57 -> 3rd character
    0x58 - 0x5F -> 4th character
    0x60 - 0x67 -> 5th character
    0x68 - 0x6F -> 6th character
    0x70 - 0x77 -> 7th character
    0x78 - 0x7F -> 8th character
    */
    uint8_t lcdCommand = LCD_BIT_SETCGRAMADDR + (Location << 8);


    //Send the command for setting the CGRAM address to LCD
    //R/W bit = 0, RS = 0
    if (LCD_I2C_HD44780_WriteByte((uint8_t)0x00, &lcdCommand) == 0) {
        return 0;
    }

    //Writing the custom character array into CGRAM address
    //R/W bit = 0, RS = 1 (for DR)
    for (uint8_t i = 0; i < 8; ++i) {
    	//select the DR
        if (LCD_I2C_HD44780_WriteByte(LCD_BIT_RS, &CustomPattern[i]) == 0) {
            return 0;
        }
    }


    return Location+1;//return the next empty location
}

/* *********************************Documentation Section **********************************************
 * @fn                           :
 * @brief                        :
 * @param[in1]                   : Location range from 0 to 7
 * @param[in2]                   :
 * @return                       :
 * @Note                         :
 */
uint8_t LCD_I2C_HD4470_DisplayCustomCharFromCGRAM(uint8_t Location)
{
	if(Location > 7)
	{
		return 0;
	}
	uint8_t lcdCommand = 0;

	(void) lcdCommand;

	LCD_I2C_HD44780_WriteByte(LCD_BIT_RS, &Location);


	return 1;




}



// Step1 : Initialize the LCD
/* *********************************Documentation Section **********************************************
 * @fn                           : LCD_I2C_HD44780_Init
 * @brief                        : Turn display on and initialize it parameters.
 * @param[in1]                    : Pointer to I2C Handler Structure
 * @param[in2]                    : 7 bit Address of slave.
 * @param[in3]                    : No of Lines for LCD i.e One line or 2 lines
 * @param[in4]                    : Maximum No of characters that can be displayed for each line.
 * @param[in5]                    : No of dots used to display each character. i.e 5x8 or 5x10.
 * @return                       : 1 -> LCD successfully Init, 0 -> error occured.
 * @Note                         :
 */
uint8_t LCD_I2C_HD44780_Init(I2C_Handle_t *phi2c, uint8_t SlaveAddr, uint8_t NoOfLines, uint8_t Columns,uint8_t NoOfCharacterFonts )
{
	/************** Temporary variables to store the data */
	//to store the command to be sent to LCD
	uint8_t lcd_cmd_code = 0;
	//to store the higher order 4 bits
	uint8_t lcd_higher_4bits = 0;
	//to store the lower order 4 bits
	uint8_t lcd_lower_4bits = 0;
	(void ) lcd_lower_4bits;//to avoid warning.
	/********************************************************/

	//Initialize the I2C Pins and peripheral
	I2Cx_Init(phi2c,SlaveAddr);

	/* Initialize the I2C structure parameters*/
	lcdParams.pi2handle = 0;
	lcdParams.pi2handle = phi2c;
	lcdParams.backlight_status = LCD_BIT_BACKIGHT_ON;
	lcdParams.no_of_columns = Columns;
	lcdParams.no_of_lines = NoOfLines;
	//since LSB bit R/W(BAR) , 1 -> Read , 0 -> Write and here we're writing slave address
	lcdParams.slaveaddr = SlaveAddr ;



    //Step1 : Wait for more than 15ms
	delay_ms(40);//wait for 20ms

	//Step2 : Function set with Data length (DL) = 8 bits. Check Fig17 in datasheet.
	//DB4= 1, DB5 = 1 i.e 0x03 since used in 4 bit mode so 0x03<< 4
	//sending the Higher 4 bits then lower 4 bits and then make the EN = 0 to start transfer of data
	//Here backlight is OFF
	lcd_cmd_code = 0x03 ;

	//move the lower 4 bits to higher 4 bits since lower 4bits are 0
	lcd_higher_4bits = (lcd_cmd_code<< 4) & 0xF0;

	//sequence of data transfer
	lcd_cmd_buffer[0] = LCD_BIT_E | lcd_higher_4bits;//EN = 1, RS = 0, R/W = 0
	lcd_cmd_buffer[1] = lcd_higher_4bits;//EN = 0, RS = 0, R/W = 0
	if (I2C_MasterSendData(lcdParams.pi2handle, lcd_cmd_buffer, 2, lcdParams.slaveaddr, REPEATE_START_CONDITION_DISABLE) != STATUS_OK)
		return 0;

	//Step3 : Wait for > 4.1ms
	delay_ms(5);

	//Step4 : Set the interface to 8 bits long
	lcd_cmd_code = 0x03;
	//move the lower 4 bits to higher 4 bits
	lcd_higher_4bits = (lcd_cmd_code<< 4) & 0xF0;

	lcd_cmd_buffer[0] = LCD_BIT_E | lcd_higher_4bits;//EN = 1, RS = 0, R/W = 0
	lcd_cmd_buffer[1] = lcd_higher_4bits;//EN = 0, RS = 0, R/W = 0
	I2C_MasterSendData(lcdParams.pi2handle, lcd_cmd_buffer, 2, lcdParams.slaveaddr, REPEATE_START_CONDITION_DISABLE);


	//Step5 : Wait for >100us
	delay_us(102);

	//Step6 : Set the interface to 8 bits long
	lcd_cmd_code = 0x03;
	//move the lower 4 bits to higher 4 bits
	lcd_higher_4bits = (lcd_cmd_code<< 4) & 0xF0;

	lcd_cmd_buffer[0] = LCD_BIT_E | lcd_higher_4bits;//EN = 1, RS = 0, R/W = 0
	lcd_cmd_buffer[1] = lcd_higher_4bits;//EN = 0, RS = 0, R/W = 0
	I2C_MasterSendData(lcdParams.pi2handle, lcd_cmd_buffer, 2, lcdParams.slaveaddr, REPEATE_START_CONDITION_DISABLE);

	/* Note: If not wait for busy flag then following steps isn't done */
	delay_us(4500);//for busy state since it takes 10 ms to complete

	/* wait for busy Flag (BF flag)
	 *
	 */
	/*
	//first send the command to indicate read of data
	uint8_t RxBuffer[2] = {0};//create an array to store the 4 bit data as received twice
	do
	{
		lcd_cmd_buffer[0] = LCD_BIT_E | LCD_BIT_RW ;
		lcd_cmd_buffer[1] = LCD_BIT_RW;
		I2C_MasterSendData(lcdParams.pi2handle, lcd_cmd_buffer, 2, SlaveAddr);

		I2C_MasterReceiveData(lcdParams.pi2handle, RxBuffer, 1, lcdParams.slaveaddr);

	}while(RxBuffer[0] & (LCD_BIT_BUSY_FLAG));	//checking the BF flag
	*/



	//Step7 : Checking the Busy flag (DB7 bit)
	//Here until it is 1 -> internal operation ongoing
	//Busy flag = 0 -> Internal operation done
	uint8_t lcd_busy_flag_status = 0;
	(void)lcd_busy_flag_status;


	//Step8 : Set the interface data length = 4bit
	lcd_cmd_code = 0x20;
	lcd_higher_4bits = lcd_cmd_code ;
	lcd_cmd_buffer[0] = LCD_BIT_E | lcd_higher_4bits;//EN = 1, RS = 0, R/W = 0
	lcd_cmd_buffer[1] = lcd_higher_4bits;//EN = 0, RS = 0, R/W = 0
	I2C_MasterSendData(lcdParams.pi2handle, lcd_cmd_buffer, 2, lcdParams.slaveaddr, REPEATE_START_CONDITION_DISABLE);
	//delay_us(4500);

	//done by following function since backlight must be ON

	//step9 : Configure the number of display lines and character font with interface length = 4 bits
	lcd_cmd_code = LCD_BIT_FUNCTION_SET ;

	//setting the number of lines. It is not working for single line
	if(lcdParams.no_of_lines > 1)
		lcd_cmd_code |= LCD_BIT_2LINE;

	else
		lcd_cmd_code |= LCD_BIT_1LINE;

	//Character font i.e either 5x8 dots or 5x10 dots
	if(NoOfCharacterFonts == LCD_BIT_5x10DOTS )
		lcd_cmd_code |= LCD_BIT_5x10DOTS;//else automatically 5x8 dots value is chosen.
	else
		lcd_cmd_code |= LCD_BIT_5x8DOTS;


	//lcd_cmd_code = LCD_BIT_FUNCTION_SET | LCD_BIT_DATA_LENGTH_4BITS | LCD_BIT_1LINE | LCD_BIT_5x8DOTS ;//set the interface data length = 4bits
	LCD_I2C_HD44780_WriteByte (0x00,&lcd_cmd_code);

	//step10 : controlling the display field
	//lcd_cmd_code = LCD_BIT_DISPLAY_CTRL | LCD_BIT_DISPLAY_OFF |LCD_BIT_CURSOR_OFF|LCD_BIT_CURSOR_BLINK_OFF;
	//LCD_I2C_HD44780_WriteByte (0x00,&lcd_cmd_code); //Display on/off control --> D=0,C=0, B=0  ---> display off
	LcdI2CDisplayOFF() ;//display OFF, cursor OFF and blink OFF



	//Step 11:Clear the display
	//delay_ms(1);
	//lcd_cmd_code = LCD_BIT_DISPLAY_CLEAR;
	//LCD_I2C_HD44780_WriteByte (0x00,&lcd_cmd_code);  // clear display
	//delay_us(2000);
	LcdI2CDisplayClear();

	//Step 12 : Configure the Entry mode
	//lcd_cmd_code = LCD_BIT_ENTRY_MODE | LCD_BIT_WITH_READ_WRITE_LEFT | LCD_BIT_WITH_READ_WRITE_SHIFT_DECREMENT;
	//lcd_cmd_code = LCD_BIT_ENTRY_MODE | LCD_BIT_WITH_READ_WRITE_RIGHT ;
	//LCD_I2C_HD44780_WriteByte(0x00,&lcd_cmd_code); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	//delay_us(4500)	;
	//lcd_cmd_code = LCD_BIT_ENTRY_MODE | LCD_BIT_CURSOR_DIRECTION_RIGHT;
	LcdI2CCursorDirToRight();
	//LcdI2CCursorDirToLeft();




	//Step13 : Display ON , cursor ON
	//LcdI2CCursorON();
	LcdI2CCursorBlinkON();
	//lcd_cmd_code = LCD_BIT_DISPLAY_CTRL | LCD_BIT_DISPLAY_ON | LCD_BIT_CURSOR_ON | LCD_BIT_CURSOR_BLINK_ON;
		//	LCD_I2C_HD44780_WriteByte(0x00,&lcd_cmd_code); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
			//delay_us(4500);



	//Step14 : Make the cursor to go to home position
	LcdI2CCursorHome();
	//lcd_cmd_code = LCD_BIT_CURSOR_HOME;
	//LCD_I2C_HD44780_WriteByte(0x00,&lcd_cmd_code); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
	//delay_us(2000);

/*
	*/

 	return 1;

}

/* *********************************Documentation Section **********************************************
 * @fn                           :
 * @brief                        :
 * @param[in1]                   :
 * @return                       :
 * @Note                         :
 */
uint8_t LCD_I2C_HD44780_Backlight_Ctrl(uint8_t cmd)
{
	lcdParams.backlight_status = cmd;
	I2C_MasterSendData(lcdParams.pi2handle, &lcdParams.backlight_status, sizeof(lcdParams.backlight_status), lcdParams.slaveaddr, REPEATE_START_CONDITION_DISABLE);
	return 1;
}

/* *********************************Documentation Section **********************************************
 * @fn                           :
 * @brief                        :
 * @param[in1]                   :
 * @return                       :
 * @Note                         :
 */
void I2C_LCD_Init(void)
{



	//also init the I2C1 as default with PB6 -> SCL , PB7 -> SDA
	LCD_I2C_HD44780_Init(&I2C_LCD_Handle,SLAVE_ADDR, LCD_NO_OF_LINES,LCD_NO_OF_COLUMNS,LCD_NO_OF_CHAR_FONT  );

}



/*
void LCD_HD44780_Send_Data(uint8_t value,I2C_Handle_t *I2C1Handle)
{
	//Variables to store the lower and higher nibble
	int8_t data_MSB =0,data_LSB = 0;

	//To store the RS, RW and EN values
	uint8_t data_init[4] = {0};

	data_MSB = value & 0xF0;
	//move the LSB nibble to MSB nibble
	data_LSB = (value << 4)& 0xF0;

	//putting upper 4 bits then lower 4 bits in array data_init
	//Here high to low pulse on EN bit to send the command
	//RS = 0 to send the command.
	data_init[0] = data_MSB | 0x0D;//EN = 1, RS = 1
	data_init[1] = data_MSB | 0x09;//EN = 0, RS = 1
	data_init[2] = data_LSB | 0x0D;//EN = 1, RS = 0
	data_init[3] = data_LSB | 0x09;//EN = 0, RS = 0


	//Send the whole array to LCD
	I2C_MasterSendData(I2C1Handle, data_init, sizeof(data_init),lcdParams.slaveaddr);



}



// Send the MSB 4 bit data then 4 lower bit data
void LCD_HD44780_Send_Cmd(uint8_t cmd,I2C_Handle_t *I2C1Handle)
{


	  char data_u, data_l;
		uint8_t data_t[4];
		data_u = (cmd&0xf0);
		data_l = ((cmd<<4)&0xf0);
		data_t[0] = data_u|0x0C;  //en=1, rs=0
		data_t[1] = data_u|0x08;  //en=0, rs=0
		data_t[2] = data_l|0x0C;  //en=1, rs=0
		data_t[3] = data_l|0x08;  //en=0, rs=0



	//Send the whole array to LCD
	I2C_MasterSendData(I2C1Handle, data_t, 4,lcdParams.slaveaddr);



}


//function that will do the Init at reset of LCD
//see the page 46 of datasheet
void LCD_HD44780_Init(I2C_Handle_t I2CHandle)
{
	TIMER6_Init();


	uint8_t txbuffer = 0;

	delay_ms(40);

	txbuffer = 0x03 ;
	//LCD_Send_Cmd(txbuffer,I2CHandle);
	I2C_MasterSendData(&I2CHandle, &txbuffer, sizeof(txbuffer),slaveaddr);


	delay_ms(5);

	txbuffer = 0x03 ;//initialize for 8 bit data length with EN = 1
	//LCD_Send_Cmd(txbuffer,I2CHandle);
	I2C_MasterSendData(&I2CHandle, &txbuffer, sizeof(txbuffer),slaveaddr);

	delay_us(150);

	txbuffer = 0x03 ;//initialize for 8 bit data length with EN = 1
	//LCD_Send_Cmd(txbuffer,I2CHandle);
	I2C_MasterSendData(&I2CHandle, &txbuffer, sizeof(txbuffer),slaveaddr);


	//Step2 : Set the data length as 4 bit
	txbuffer = 0x20 ;//initialize for 4  bit data length with 2 display lines and 5x8 dots.
	LCD_HD44780_Send_Cmd(txbuffer,I2CHandle);

	//no of lines = 2,5x8 display, 4 bit data length
	txbuffer = 0x28;
	LCD_HD44780_Send_Cmd(txbuffer,I2CHandle);

	//display OFF
	txbuffer = 0x08;
	LCD_HD44780_Send_Cmd(txbuffer,I2CHandle);

	//clear the display
	txbuffer = 0x01;
	LCD_HD44780_Send_Cmd(txbuffer,I2CHandle);
	delay_ms(3);




	//Set cursor position as increment with display ON
	txbuffer = 0x06;
	LCD_HD44780_Send_Cmd(txbuffer,I2CHandle);

	//Set cursor position as increment
	//display ON
	txbuffer = 0x0C;
	LCD_HD44780_Send_Cmd(txbuffer,I2CHandle);


}

*/


















