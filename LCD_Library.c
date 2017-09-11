/*
 * LCD_Library.c
 *
 * Created: 01/09/2017 11:57:58 م
 *  Author: user
 */ 

#include "lcd.h"
void init_sensor()
{  
   //voltage referance:(with AVCC reference):
    sbi(ADMUX,REFS0);   
    //cbi(ADMUX,REFS1);

    //select ADC5 Channel:
    sbi(ADMUX,MUX0);
    cbi(ADMUX,MUX1);
    sbi(ADMUX,MUX2);
    cbi(ADMUX,MUX3);

    cbi(ADMUX,ADLAR);    //right adjustment

    //prescaler selector: (128 division factor):
    sbi(ADCSRA,ADPS0);
    sbi(ADCSRA,ADPS1);
    sbi(ADCSRA,ADPS2);
    sbi(ADCSRA,ADEN);	   //enable ADC

    //consumption reduction:
        //disable digital inputs:
        DIDR0=0xff;
        //POWER reduction:
        sbi(SMCR,SE);   //ENABLE SLEEP MODE
        sbi(SMCR,SM0);  //ADC noise cancellation
	
}

uint16_t adc_read(uint8_t adcx) 
{
	/* adcx is the analog pin we want to use.  ADMUX's first few bits are
	 * the binary representations of the numbers of the pins so we can
	 * just 'OR' the pin's number with ADMUX to select that pin.
	 * We first zero the four bits by setting ADMUX equal to its higher
	 * four bits. */
	ADMUX	&=	0xf0;
	ADMUX	|=	adcx;

	/* This starts the conversion. */
	ADCSRA |= _BV(ADSC);

	/* This is an idle loop that just wait around until the conversion
	 * is finished.  It constantly checks ADCSRA's ADSC bit, which we just
	 * set above, to see if it is still set.  This bit is automatically
	 * reset (zeroed) when the conversion is ready so if we do this in
	 * a loop the loop will just go until the conversion is ready. */
	while ( (ADCSRA & _BV(ADSC)) );

	/* Finally, we return the converted value to the calling function. */
	return ADC;
}


void init_servo()
{
    //set OC1A FOR generating PWM
    sbi(DDRB,PB1);
    cbi(PORTB,PB1);

    //SELECT MODE:
        //fast PWM: (with top = ICR1 & update its value @ BOTTOM , TOV flag set on TOP)
        sbi(TCCR1A,WGM11);
        cbi(TCCR1A,WGM10);
	sbi(TCCR1B,WGM13);
	sbi(TCCR1B,WGM12);
        // non inverting mode : to control on servo motion @ last 2ms in period
        sbi(TCCR1A,COM1A0);
	sbi(TCCR1A,COM1A1);

    //PRECALING (8):(to have 40000 cycle per second which means that every ms has 40000 cycles)
    sbi(TCCR1B,CS11);
    //sbi(TCCR1B,CS10);
    //setting top value equal 39999 @which starting a new clock
    ICR1=39999;
}

void lcd_init_4bits()
{
      data_pinsmode=0xff;			// Configure both databus and controlbus as output
      control_pinsmode=0x07;
      _delay_ms(20);
      send_cmd_4bits(0x30);
      _delay_ms(5);
      send_cmd_4bits(0x30);
      _delay_ms(1);
      send_cmd_4bits(0x30);
      _delay_ms(1);
      send_cmd_4bits(0x02);	       //Initilize the LCD in 4bit Mode
     _delay_ms(1);
     send_cmd_4bits(0x28);
     _delay_ms(1);
     send_cmd_4bits(0x06);  //entry mode set: increment cursor & without shifting entire display
     _delay_ms(1);
     send_cmd_4bits(0x14);		//cursor or display shift: only cursor shifted right
     _delay_ms(1);
     send_cmd_4bits(0x0E);	      // Display ON cursor ON
     _delay_ms(1);
     send_cmd_4bits(0x40);		//enable CGRAM
     _delay_ms(1);
     send_cmd_4bits(0x80);	      // Move the Cursor to First line First Position
     //send_cmd_4bits(0x01); 
}

void send_cmd_4bits( char cmd)
{
   	check_busy();
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
	flashing();				//flash light 
	databus=0;
}
void send_char_4bits( char dat)
{
   	check_busy();
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
	flashing();				//flash light 
	databus=0;
	
}

void lcd_init_8bits()
{
   data_pinsmode=0xff;
   control_pinsmode=0x07;
	
   _delay_ms(15);		//wait till power source reach 4.5 volt
   send_cmd_8bits(0x06);  //entry mode set: increment cursor & without shifting entire display
   _delay_ms(1);
   send_cmd_8bits(0x0E);		//Display : entire display is on & cursor is on with blinking
  _delay_ms(1);
  send_cmd_8bits(0x14);		//cursor or display shift: only cursor shifted right
  send_cmd_8bits(0x38);		//function set: 8 bit mode with 2 lines display mode 5x8 dots
   _delay_ms(1); 
   send_cmd_8bits(0x40);		//enable CGRAM
   _delay_ms(1);
   send_cmd_8bits(0x80);		//enable DDRAM address
}




void send_cmd_8bits(unsigned char cmd)
{
	check_busy();
	databus=cmd;
	cbi(control_bus,rw);	//write mode
	cbi(control_bus,rs);	//command mode
	sbi(control_bus,en);	//enable displaying
	_delay_ms(1);
	cbi(control_bus,en);
	flashing();				//flash light 
	databus=0;
}
void send_char_8bits(unsigned char Char)
{
	check_busy(); 
	databus=Char;
	cbi(control_bus,rw);	//write mode
	sbi(control_bus,rs);	//data mode
	sbi(control_bus,en);	//enable displaying
	_delay_ms(1);
	cbi(control_bus,en);
	cbi(control_bus,rs);
	flashing();				//flash light
	databus=0;
}
void send_str_4bits(char *string)
{
	while(*string!='\0')
	{
		send_char_4bits(*string++);
	}
}
void goto_XY_4bits(uint8_t x,uint8_t y)
{
	if (y==1)
	{
		send_cmd_4bits(firstline+x);
	}
	else if(y==2)
	{
		send_cmd_4bits(secondline+x);
	}
}


void send_str_4bits_withXY(uint8_t x,uint8_t y,char *string)
{
	goto_XY_4bits(x,y);
	send_str_4bits(string);
}


void send_int_withXY(uint8_t x,uint8_t y,int value,uint8_t no_digits)
{
	char stringToDisplay[no_digits];
	itoa(value,stringToDisplay,10);
	send_str_4bits_withXY(x,y,stringToDisplay);
	//send_str_4bits(" ");
}

void check_busy()
{
	data_pinsmode=0;
	sbi(control_bus,rw);	//read mode
	cbi(control_bus,rs);	//command mode
	while(databus >= 0x80)			//check for busy (when it's <0x80 it's not busy  
	{
		flashing();		//flashing on and off
	}
	
	data_pinsmode=0xff;	
}
void flashing()
{
	sbi(control_bus,en);
	asm volatile ("nop");	//"nop is assembler command for delay
	asm volatile ("nop");
	cbi(control_bus,en);
}


void clear_screen_4bits()
{
	send_cmd_4bits(0x01);
	_delay_ms(2);
}

