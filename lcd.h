/*
 * IncFile1.h
 *
 * Created: 02/09/2017 03:28:15 م
 *  Author: user
 */ 


#ifndef LCD_H_
#define LCD_H_

#include <avr/sleep.h>
#include <inttypes.h>
#include <avr/io.h>
#include<stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>
#define sbi(reg,pin) reg|=_BV(pin)
#define cbi(reg,pin) reg&=~_BV(pin)
#define tbi(reg,pin) reg^=_BV(pin)
#define is_high(reg,pin) (((reg)&_BV(pin))==1)

#define data_pinsmode		DDRD
#define databus			PORTD
#define control_pinsmode	DDRC
#define control_bus		PORTC

#define rs PC0
#define rw PC1
#define en PC2
#define maxlines	2
#define maxchars	16
#define firstline	0x80
#define secondline 0xc0
#define blankspace ' '

// Proto types:
void init_sensor();
uint16_t adc_read(uint8_t adcx);
void init_servo();
void lcd_init_4bits();
void send_cmd_4bits( char cmd);
void send_char_4bits( char dat);

void send_str_4bits(char *string);
void goto_XY_4bits(uint8_t x,uint8_t y);
void send_str_4bits_withXY(uint8_t x,uint8_t y,char *string);
void send_int_withXY(uint8_t x,uint8_t y,int value,uint8_t no_digits);

void lcd_init_8bits();
void send_cmd_8bits(unsigned char cmd);
void send_char_8bits(unsigned char);


void clear_screen_4bits();
void check_busy();
void flashing();



/*
LCD_Init()
Description  :This function is used to initialize the lcd in 4-bit mode
Function name: LCD_Init()
I/P Arguments: none
Return value : none
*/

/*
send_cmd()
Description  :This function sends a command to LCD in the following steps.
step1: Send the Higher Nibble of the I/P command to LCD.
step2: Select the Control Register by making RS low.
step3: Select Write operation making RW low.
step4: Send a High-to-Low pulse on Enable PIN with some delay_us.
step5: Send the Lower Nibble of the I/P command to LCD.
step6: Select the Control Register by making RS low.
step7: Select Write operation making RW low.
step8: Send a High-to-Low pulse on Enable PIN with some delay_us.
I/P Arguments: 8-bit command supported by LCD.
Return value : none
*/


/*
LCD_DataWrite()
Description:This function sends a character to be displayed on LCD in the following steps.
step1: Send the higher nibble of the character to LCD.
step2: Select the Data Register by making RS high.
step3: Select Write operation making RW low.
step4: Send a High-to-Low pulse on Enable PIN with some delay_us.
step5: wait for some time
step6: Send the lower nibble of the character to LCD.
step7: Select the Data Register by making RS high.
step8: Select Write operation making RW low.
step9: Send a High-to-Low pulse on Enable PIN with some delay_us.
Function name: LCD_DataWrite()
I/P Arguments: ASCII value of the char to be displayed.
Return value : none
*/
/*
LCD_DisplayNumber()
Description  :This function is used to display a 5-digit integer(0-65535).
ex:
if the number is 12345 then 12345 is displayed.
if the number is 123 then 00123 is displayed.
Function name: LCD_DisplayNumber()
I/P Arguments: unsigned int.
Return value	: none

void LCD_DisplayNumber(unsigned int num)
{
	LCD_DataWrite((num/10000)+0x30);
	num=num%10000;
	
	LCD_DataWrite((num/1000)+0x30);
	num=num%1000;
	
	LCD_DataWrite((num/100)+0x30);
	num=num%100;
	
	LCD_DataWrite((num/10)+0x30);
	
	LCD_DataWrite((num%10)+0x30);
	
}


*/

/*
LCD_ScrollMessage()
Description  :This function scrolls the given message on the first line.""
16 chars are displayed at atime.
Pointer is incremented to skip a char each time to give the illusion of moving chars.
If the chars are less than 16, then the BlankSpaces are displayed.
I/P Arguments: char *msg_ptr (msg_ptr -> pointer to the string to be scrolled)
Return value	: none

void LCD_ScrollMessage(char *msg_ptr)
{
	unsigned char i,j;
	send_cmd(0x0c);			 //Disable the Cursor
	for(i=0;msg_ptr[i];i++)        //Loop to display the complete string
	{                            //each time 16 chars are displayed and
		//pointer is incremented to point to next char
		
		LCD_GoToLineOne();                   //Move the Cursor to first line
		
		for(j=0;j<LCDMaxChars && msg_ptr[i+j];j++) //loop to Display first 16 Chars
		LCD_DataWrite(msg_ptr[i+j]);                 //or till Null char
		
		for(j=j; j<LCDMaxChars; j++)               //If the chars are below 16
		LCD_DataWrite(BlankSpace);              //then display blank spaces
		
		_delay_ms(500);
	}
	send_cmd(0x0E);			  //Enable the Cursor
}
*/

/*
LCD_DisplayString()
Description  :This function is used to display the ASCII string on the lcd.
The string_ptr points to the first char of the string and traverses till the end(NULL CHAR).
Each time a char is sent to LCD_DataWrite funtion to display.
I/P Arguments: String(Address of the string) to be displayed.
Return value	: None
*/

/*
LCD_GoToXY()
Description  :This function moves the Cursor to specified position
I/P Arguments: char row,char col
row -> line number(line1=0, line2=1),For 2line LCD the I/P argument should be either 0 or 1.
col -> char number.For 16-char LCD the I/P argument should be betwen 0-15.
Return value	: none

void LCD_GoToXY(char row, char col)
{
	char pos;
	
	if(row<LCDMaxLines)
	{
		pos= LineOne | (row << 6); // take the line number
		//row0->pos=0x80  row1->pos=0xc0
		
		if(col<LCDMaxChars)
		pos= pos+col;            //take the char number
		//now pos points to the given XY pos
		
		send_cmd(pos);	       // Move the Cursor to specified Position
	}
}

*/

/*
LCD_DisplayRtcTime()
Description  :This function display hour,min,sec read from DS1307.
I/P Arguments: char hour,char min,char sec(hour,min,sec should be packed BCD format,as read from DS1307)
Return value	: none

void LCD_DisplayRtcTime(char hour,char min,char sec)
{
	LCD_DataWrite(((hour>>4) & 0x0f) + 0x30);
	LCD_DataWrite((hour & 0x0f) + 0x30);
	LCD_DataWrite(':');
	
	LCD_DataWrite(((min>>4) & 0x0f) + 0x30);
	LCD_DataWrite((min & 0x0f) + 0x30);
	LCD_DataWrite(':');
	
	LCD_DataWrite(((sec>>4) & 0x0f) + 0x30);
	LCD_DataWrite((sec & 0x0f) + 0x30);
	
}


LCD_DisplayRtcDate()
Description  :This function display day,month,year read from DS1307.
I/P Arguments: char day,char month,char year(day,month,year should be packed BCD format,as read from DS1307)
Return value	: none

void LCD_DisplayRtcDate(char day,char month,char year)
{
	LCD_DataWrite(((day>>4) & 0x0f) + 0x30);
	LCD_DataWrite((day & 0x0f) + 0x30);
	LCD_DataWrite('/');
	
	LCD_DataWrite(((month>>4) & 0x0f) + 0x30);
	LCD_DataWrite((month & 0x0f) + 0x30);
	LCD_DataWrite('/');
	
	LCD_DataWrite(((year>>4) & 0x0f) + 0x30);
	LCD_DataWrite((year & 0x0f) + 0x30);
	
}
*/

//*********4 Bit-mode********

/*
LCD_Init()
Description  :This function is used to initialize the lcd in 4-bit mode
Function name: LCD_Init()
I/P Arguments: none
Return value : none

void LCD_Init()
{
	_delay_ms(50);
	data_pinsmode = 0xff;  // Configure both databus and controlbus as output
	send_cmd(0x02);	       //Initilize the LCD in 4bit Mode
	send_cmd(0x28);
	send_cmd(0x0E);	      // Display ON cursor ON
	send_cmd(0x01);	      // Clear the LCD
	send_cmd(0x80);	      // Move the Cursor to First line First Position
	
}
*/

/*
LCD_Clear()
Description  :This function clears the LCD and moves the cursor to first Position
I/P Arguments: none.
Return value	: none

void send_cmd( char cmd)
{
	
	databus=(cmd & 0xf0);        // Send the Higher Nibble of the command to LCD
	control_bus &=~(1<<rs);  // Select the Command Register by pulling RS LOW
	control_bus &=~(1<<rw);  // Select the Write Operation  by pulling RW LOW
	control_bus |=1<<en;     // Send a High-to-Low Pusle at Enable Pin
	_delay_us(1);
	control_bus &=~(1<<en);
	
	_delay_us(10);				// wait for some time
	
	databus=((cmd<<4) & 0xf0);   // Send the Lower Nibble of the command to LCD
	control_bus &=~(1<<rs);  // Select the Command Register by pulling RS LOW
	control_bus &=~(1<<rw);  // Select the Write Operation  by pulling RW LOW
	control_bus |=1<<en;     // Send a High-to-Low Pusle at Enable Pin
	_delay_us(1);
	control_bus &=~(1<<en);
	
	_delay_ms(1);
}
*/

/*
LCD_Init()
Description  :This function is used to initialize the lcd in 8-bit mode
Function name: LCD_Init()
I/P Arguments: none.
Return value	: none


void LCD_DataWrite( char dat)
{
	databus=(dat & 0xf0);	  // Send the Higher Nibble of the Data to LCD
	control_bus |=1<<rs;	  // Select the Data Register by pulling RS HIGH
	control_bus &=~(1<<rw);	  // Select the Write Operation  by pulling RW LOW
	control_bus |=1<<en;	  // Send a High-to-Low Pusle at Enable Pin
	_delay_us(1);
	control_bus &=~(1<<en);
	_delay_us(10);
	
	databus=((dat <<4) & 0xf0); // Send the Lower Nibble of the Data to LCD
	control_bus |=1<<rs;	   // Select the Data Register by pulling RS HIGH
	control_bus &=~(1<<rw);	   // Select the Write Operation  by pulling RW LOW
	control_bus |=1<<en;	   // Send a High-to-Low Pusle at Enable Pin
	_delay_us(1);
	control_bus &=~(1<<en);
	_delay_ms(1);
	
}

*/


#endif /* INCFILE1_H_ */