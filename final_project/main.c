// main.c 
// Written by Ibrahim Alarifi
// Written on 12/12/2019


//  HARDWARE SETUP:
//  Bushbuttons and Seven-Seg:
//  	PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  	PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  	PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  	PORTB bit 7 goes to the PWM transistor base.
//  Bargraph:
//  	reglclk		-> PORTD bit 1
//  	srclk		-> PORTB bit 1 (sclk)
//  	sdin		-> PORTB bit 2 (mosi)
//  	oe_n		-> PORTB bit 7 (active low)
//  	gnd		-> gnd
//  	vdd 		-> vcc
//  Encoders:
//  	sh/ld 		-> PORTE, bit 6
//  	sck		-> PORTB bit 1 (sclk)
//  	sout		-> PORTB bit 3 (miso)
//  	clkin and gnd	-> gnd
//  	vdd		-> vcc
//  Op-Amp:
//  	alarm_sig	-> PORTC bit 5 and bit 6
//  	Volume		-> PORTE bit 3
//  	left/right	-> Lin/Rin
//  Photocell:
//  	ADC6		-> PORTF bit 7 
//  LM73:
//  	vcc		-> vcc
//  	gnd		-> gnd
//  	sda		-> PORTE, bit 1 (TWI, SDA)
//  	sck		-> PORTE, bit 0 (TWI, SCK)
//  Mega48:
//  	Tx		-> Rx0 (RS_232 Tx)
//  	Rx		-> Rx0 (RS_232 Rx)
//  	gnd		-> gnd
//  si4734 ** NEEDS LEVEL SHIFTER 3.3V - 5V **
//  	SCLK		-> PORTE bit 0, SCK
//  	SDIO		-> PORTE bit 1, SDA
//  	GPO2		-> PORTE bit 7, INT7
//  	vin		-> vcc
//  	ENBL		-> vcc
//  	gnd		-> gnd
//  	RST		-> PORTE bit 2
//  	Lout/Rout	-> Op-Amp
//


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include "hd44780.h"
#include "lm73_functions.h"
#include "twi_master.h"
#include "uart_functions.h"
#include "si4734.h"

// holds the values to control the decoder (74HC138) of the 7-segment, to turn on specific digits
uint8_t digit[6] = {
	0x00,	// 1st digit 
	0x10,	// 2nd digit
	0x30,	// 4th digit
	0x40,	// 5th digit
	0x20,	// 3rd digit (colon)
	0x70 	// off state
};

// holds the state of each button: 0=off, 1=on
volatile uint8_t dstate[4] = {1,1,1,1};

// holds the current value of each digit, initially all digits are zero
volatile uint8_t dig_value[4] = {
	0,
	0,
	2,
	1
};

// decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[11] = {
	0b11000000,	// zero
	0b11111001,	// one 
	0b10100100,	// two
	0b10110000,	// three
	0b10011001,	// four
	0b10010010,	// five
	0b10000010,	// six
	0b11111000,	// seven
	0b10000000,	// eight
	0b10010000,	// nine
	0b11111100	// colon
};

// radio stuff 
extern enum radio_band{FM, AM, SW};
extern volatile uint8_t STC_interrupt;
volatile enum radio_band current_radio_band = FM;
volatile uint16_t current_fm_freq = 8810;
volatile uint16_t current_am_freq;
volatile uint16_t current_sw_freq;
volatile uint8_t changeflag = 0;
volatile uint32_t vol_level = 0x0000;
volatile uint8_t radio_en = 0; //



//declare time keeping variables
volatile uint8_t tcount=0; // counter used in TCNT0
volatile uint8_t colflag=1; // initially on
volatile uint8_t blinkflag=0; // blinking feature when setting time/alarm
volatile uint8_t setflag=0; // detrmine whether user is in set mood
volatile uint8_t tick=0; // 1 tick = 1 sec

// declare variables that is outside scope of main
volatile uint16_t total = 720;	// keep track of increment and total
volatile uint16_t atotal = 0;	// setting alarm time
volatile uint8_t set_alarm = 0; // alarm flag
volatile uint8_t readbuff;	// reads data from SPDR
volatile uint8_t enc1[2];	// new and old value of encouder 1
volatile uint8_t enc2[2];	// new and old value of encouder 2
volatile uint8_t Fsign = 10;	// 1->increment, 0->decrement
volatile uint8_t Ssign = 10;	// 1->increment, 0->decrement
volatile uint16_t snooze = 100;	// snooze keeper
volatile uint16_t adc_result;	// holds ADC value
volatile uint8_t  rcv_rdy = 0;	// determine whether recv is complete
char              rx_char; 	// recv a char via uart
uint8_t		 alarm_mood =0;

// this flag keeps track of modes
///////////fmode info////////////////
// both 0x00 -> Time display -> NONE
// both 0xFF -> Both are pressed (do nothing) -> BOTH
// [0] 0xFF -> Set Clock -> CLOCK_C
// [1] 0xFF -> Set Alarm -> ALARM_C
////////////////////////////////////
volatile uint8_t fmode[2] = {0,0};
uint8_t freq_tune = 0; 

uint8_t dcount;	// display counter
char alarm_c [] = "ALARM | "; // string to write to lcd
char radio_c [] = "RADIO | "; // string to write to lcd
char alarm_s [] = "STARM";  // string to write to lcd
char clock_s [] = "CLOCK"; // string to write to lcd
char tone_s [] = "RTONE"; // string to write to lcd
char both [] = "BOTH!"; // string to lcd
char lcd_string[16];  // holds a string to refresh the LCD
char temp_lcd[16];  // holds a string to refresh the LCD for temprature
char temp_array[11];  // holds a string of the onboard temprature
volatile char remote_array[6];  // holds a string to recieved from uart
char remote_temp[7];  // holds a string to refresh the LCD for temprature

extern uint8_t lm73_wr_buf[2]; // twi write
extern uint8_t lm73_rd_buf[2]; // twi read

enum states {NONE, CLOCK_C, ALARM_C, BOTH, TONE}; // State machine
enum states state = NONE;

//***********************************************************************
//                           init_tcnt2
//***********************************************************************
//initalize timer/counter two to PWM mode for dimming control
void init_tcnt2(){
	TCCR2 = (1<<WGM21) | (1<<WGM20) | (1<<COM21) | (1<<COM20) | (1<<CS20); // PWM, no prescale, set on match
	OCR2 = 0xFF; // start at minimum duty cycle
} // init_tcnt2

//***********************************************************************
//                           init_tcnt1
//***********************************************************************
//initalize timer/counter one to CTC mode for alarm sound
void init_tcnt1(){
	TCCR1A = (0<<COM1A1) | (0<<COM1A0); // CTC mode, OC1A diconnected 
	TCCR1B = (1<<WGM12) | (1<<CS10); // OCR1A as top, no prescale 
	TCCR1C = 0x00; // no forced compare 
	OCR1A = 0x0258; // initial value -> 15kHz
	TIMSK |= (1<<OCIE1A); // enable interrupt	
} // init_tcnt1

//***********************************************************************
//                           init_tcnt3
//***********************************************************************
//initalize timer/counter three to PWM mode for volume level
void init_tcnt3(){
	DDRE |= (1<<PE3);
	TCCR3A = (1<<COM3A1) | (1<<COM3A0) | (1<<WGM31); // PWM mode, set on match, clear on bottom
	TCCR3B = (1<<WGM33) | (1<<WGM32) | (1<<CS30); // no prescaling, ICR3 top, OCR3A match        
	TCCR3C = 0x00; //no forced compare 
	OCR3A = 0x0000; // pick volume from 0x1000, increase for lower volume
	ICR3 = 0xF000; // top value
} // init_tcnt3

//***********************************************************************
//                           init_adc
//***********************************************************************
//initalize adc for photocell readings
void init_adc(){
	DDRF |= (0<<PF7); // PF7 for input
	PORTF |= (0<<PF7); // disable pullup 
	ADMUX = (1<<REFS0) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0); // single-ended, input PORTF bit 7, right adjusted, 10 bits
	ADCSRA = (1<<ADEN) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // ADC enabled, enable interrupt,single shot mode
} // init_adc

//***********************************************************************
//                           init_tcnt0
//***********************************************************************
//initalize timer/counter zero to CTC mode
void init_tcnt0(){
	ASSR  |=  (1<<AS0); //run off external 32khz osc (TOSC)
	TIMSK |= (1<<TOIE0); //enable interrupt
	TCCR0 |= (1<<CS00); //normal mode, no prescale
} // init_tcnt0

//***********************************************************************
//                            spi_init                               
//**********************************************************************
void spi_init(void){
	DDRE  |= (1<<PE6); // sh/ld control
	DDRD  |= (1<<PD1); // rgclk control
	DDRF  |= (1<<PF3); // enable LCD
	PORTF &= ~(1<<PF3); // initially low
	SPCR   = (1<<SPE) | (1<<MSTR); //master mode, clk low on idle, leading edge sample
	SPSR   = (1<<SPI2X); //choose double speed operation
}//spi_init

//***********************************************************************************
//                                   segment_sum                                    
//  takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//  BCD segment code in the array dig_value for display.                       
//  array is loaded at exit as:  |digit3|digit2|digit1|digit0|
//***********************************************************************************
void segsum(uint16_t stotal) {	
	uint16_t min, hr;
	uint16_t buff, val, j;
	uint8_t i;

	if (state==TONE){ //  **For Radio**
		j = 10;
		for(i=0;i<4;i++){ // this loop divide total into 4-digits
			val = 0;
			buff = stotal/j; // get the ith number in total
			val = buff % 10; // modlu it by 10
			dig_value[i] = val; // store it in the array
			j = j*10; // move on to the next number
		}
		for(i=1;i<4;i++) // incremented digit control
			if(dig_value[i]>0) // if the digit has value 
				dstate[i]=1;	// enable it to turn on
		// decremented digit control
		if(dig_value[3]==0){ // if the digit is 0
			dstate[3]=0; // disable it, to turn off
			if(dig_value[2] == 0){ // move on to next digit
				dstate[2]=0;
				if(dig_value[1] == 0)
					dstate[1]=0;
			}
		}
	}

	else	// **For Clock**
	{
		min = stotal % 60; // get minutes
		hr = stotal/60;	// get hours
		dig_value[0] = min % 10; // LSB of minutes
		dig_value[1] = min / 10; // MSB of munutes
		dig_value[2] = hr % 10; // LSB of hours
		dig_value[3] = hr / 10; // MSB of hours
		for(i=0;i<4;i++) // turn on digits
			dstate[i]=1;
	}

}//segment_sum

//***************************************************************************************************
//                            chk_buttons                                       
//  Checks the state of the button number passed to it. It shifts in ones till   
//  the button is pushed. Function returns a 1 only once per debounced button    
//  push so a debounce and toggle function can be implemented at the same time.  
//  Adapted to check all buttons from Ganssel's "Guide to Debouncing"            
//  Expects active low pushbuttons on PINA port.  Debounce time is determined by 
//  external loop delay times 12.
//  **This function is heavily based on the debounce function provided in lab1 and lecture notes**
//***************************************************************************************************
uint8_t chk_buttons(uint8_t button, uint8_t portn) {
	static uint16_t state[7] = {0, 0, 0, 0, 0, 0, 0}; // holds present state of each button
	static uint16_t state2[4] = {0, 0, 0, 0 }; // holds present state of each button
	// this perform an or operation with the previous state value, !bit_is_clear output, and 0xE000
	// when the button is pressed, !bit_is_clear return 1 per debounce, with the last debounce value
	// it will start shifting and or-ing, untill state = 0xF000, then return 1 (button is pressed)
	if(portn == 0){ //** PORT A
	state[button] = (state[button] << 1) | (! bit_is_clear(PINA, button)) | 0xE000;
	if (state[button] == 0xF000) 
		return 1;
	else
		return 0;
	}
	else if(portn == 1){ //** PORT D
	state2[button] = (state2[button] << 1) | (! bit_is_clear(PIND, button+4)) | 0xE000;
	if (state2[button] == 0xF000) 
		return 1;
	else
		return 0;
	}
} // chk_buttons

//***********************************************************************
//                           volume level function 
//***********************************************************************
// return the volume level to display on bargraph
uint8_t vol_logic(){
	uint8_t volume;
	
	volume = ~(vol_level/0xff); // conver to 8-bit 
	volume = (volume>>4); // shift right to remove offset
	if((volume==0x01)||(volume==0x02)) // 1st lvl
		volume = 0b00000001; // bargraph value 
	else if((volume==0x03)||(volume==0x04)) // 2nd
		volume = 0b00000011;
	else if((volume==0x05)||(volume==0x06)) // 3rd 
		volume = 0b00000111;
	else if((volume==0x07)||(volume==0x08)) // 4th 
		volume = 0b00001111;
	else if((volume==0x09)||(volume==0x0A))
		volume = 0b00011111;
	else if((volume==0x0B)||(volume==0x0C))
		volume = 0b00111111;
	else if((volume==0x0D)||(volume==0x0E))
		volume = 0b01111111;
	else if(volume ==0x0F) // max lvl 
		volume = 0b11111111;

	return volume;
} // vol_logic

//***********************************************************************
//                           display function
//***********************************************************************
// display all the digits
void display_t(){
	// make PortA an output
	DDRA = 0xFF;

	if(state == TONE){ // ** For Radio **
		for(dcount=0; dcount<4; dcount++){
			// update PORTB to turn on the digit (ith digit)
			PORTB = digit[dcount];
			// update PORTA with the corresponding digit value (ith digit)
			if(dig_value[dcount] == 0 && dstate[dcount] == 0)
				PORTA = 0xFF;
			else
			{	
				if(dcount == 1) // display number plus the dot for 2nd digit 	
					PORTA = (0b01111111 & dec_to_7seg[dig_value[dcount]]);
				else
					PORTA = dec_to_7seg[dig_value[dcount]]; // other digits
			}
			// delay to balance on/off time
			_delay_ms(1);
		}
	}

	else
	{ // ** CLOCK **
		// a loop to display all digit 
		for(dcount=0; dcount<5; dcount++){ // loops for the number of digits
			if(dcount==4){ // display colon
				if(colflag==1){
					PORTA = dec_to_7seg[10]; // segmant enable
					PORTB = digit[dcount]; // digit enable
					_delay_us(800); // delay 800uS
				}
			}
			else{
				if(blinkflag==0){ // if the blinking is disable
					// update PORTA with the corresponding digit value (ith digit)
					if(dig_value[dcount] == 0 && dstate[dcount] == 0)
						PORTA = 0xFF;
					else 
						PORTA = dec_to_7seg[dig_value[dcount]];
					// update PORTB to turn on the digit (ith digit)
					PORTB = digit[dcount];
					// delay to balance on/off time
					_delay_ms(1);
				}
			}
		} // for
	}

} // display

//***********************************************************************
//                           radio on function
//***********************************************************************
// turn on the radio
void radio_on(){
	DDRE  |= (1 << PE2); //Port E bit 2 is active high reset for radio 
	PORTE |= (1 << PE2); //radio reset is on at powerup (active high)
	//hardware reset of Si4734
	PORTE &= ~(1<<PE7); //int2 initially low to sense TWI mode
	DDRE  |= 0x80;      //turn on Port E bit 7 to drive it low
	PORTE |=  (1<<PE2); //hardware reset Si4734 
	_delay_us(100);
	PORTE &= ~(1<<PE2); //release reset 
	_delay_us(5);
	DDRE  &= ~(0x80);   //now Port E bit 7 becomes input from the radio interrupt
	fm_pwr_up(); //powerup the radio as appropriate
	OCR3A = vol_level; // restore volume 
	radio_en = 1; // en flag 
} // radio on

//***********************************************************************
//                           encouder function
//***********************************************************************
// encoder read function
void enc_read(uint8_t bar){
	Fsign = 10;
	Ssign = 10;
	PORTE &= ~(1<<PE6);	// SD/LD low to latch input to flip flop
	asm volatile ("nop");
	PORTE |= (1<<PE6);	// SD/LD high to allow shifting input to output pin
	SPDR = bar; // send the mode display to the bargraph
	while (bit_is_clear(SPSR,SPIF)){} //spin till SPI data has been sent 
	PORTD |= (1<<PD1);            //send rising edge to regclk on HC595
	PORTD &= ~(1<<PD1);            //send falling edge to regclk on HC595
	readbuff = SPDR;	// read input from encouder
	enc1[0] = (readbuff & 0b00000011); // get encouder 1 A and B bits 
	enc2[0] = (readbuff & 0b00001100); // get encouder 2 A and B bits

	if(enc1[0] != enc1[1]){ // 1st encoder
		if((enc1[1]==0b00000010 && enc1[0]==0b00000000)){ // clockwise
			Fsign = 1; // increment
			enc1[0]=0x0F;
			enc1[1]=0x0F;
		}
		else if((enc1[1]==0b00000000 && enc1[0]==0b00000010)){ // counter clockwise
			Fsign = 0; // decrement
			enc1[0]=0x0F;
			enc1[1]=0x0F;
		}
	}
	enc1[1] = enc1[0]; // update old value

	if(enc2[0] != enc2[1]){ // 2nd encoder
		if((enc2[1]==0b00001000 && enc2[0]==0b00000000)){ // clockwise
			Ssign = 1; // increment
			enc2[0]=0x0F;
			enc2[1]=0x0F;
		}
		else if((enc2[1]==0b00000000 && enc2[0]==0b00001000)){ // counter clockwise
			Ssign = 0; // decrement
			enc2[0]=0x0F;
			enc2[1]=0x0F;
		}
	}
	enc2[1] = enc2[0]; // update old value
} // enc_read

//***********************************************************************
//                     ISR for ADC
//***********************************************************************
ISR(ADC_vect){
	adc_result = ADC; // read adc 
	adc_result = (adc_result * 5)/1024; // convert to 0-5 volts value 
	switch(adc_result){
		case 0: // lower voltage level -> lower light in the room 
			OCR2 = 0x08; // higher duty cycle 
			break;
		case 1:
			OCR2 = 0x44;
			break;
		case 2:
			OCR2 = 0x77;
			break;
		case 3:
			OCR2 = 0xAA;
			break;
		case 4: // higher voltage level -> higher light in the room
			OCR2 = 0xFF; // lower duty cycle 
			break;
		default:
			break;
	}
	ADCSRA |= (1<<ADSC); //poke ADSC and start conversion
} // ISR ADC

//***********************************************************************
//                     ISR for timer counter onw
//***********************************************************************
ISR(TIMER1_COMPA_vect) {
	OCR1A = OCR1A+100; // increment OCR1A to variate the frequency
	if(OCR1A > 0xF000) // if it reaches max frequency, roll over
		OCR1A = 0x0258;
	PORTC ^= ((1<<PC5)|(1<<PC6)); // toggle output pins
} // ISR tcnt1

//***********************************************************************
//                     ISR for recv from USART
//***********************************************************************
ISR(USART0_RX_vect){
	static  uint8_t  i;
	rx_char = UDR0;              //get character
	remote_array[i++]=rx_char;  //store in array 
	//if entire string has arrived, set flag, reset index
	if(rx_char == '\0'){
		rcv_rdy=1; 
		i=0;  
	}
} // ISR USART0

//******************************************************************************
// External interrupt 7 is on Port E bit 7. The interrupt is triggered on the
// rising edge of Port E bit 7.  The i/o clock must be running to detect the
// edge (not asynchronouslly triggered)
//******************************************************************************
ISR(INT7_vect){
	STC_interrupt = TRUE;
} // ISR INT7

//***********************************************************************
//                     ISR for timer counter zero
//                     (1/32668)*255*128=0.999 sec
//***********************************************************************
ISR(TIMER0_OVF_vect){
	++tcount;
	if(tcount==32) // 1/4 a sec passed
		if(setflag == 1) // if setflag is enabled (user is choosing alarm or time)
			blinkflag=1; // toggle blinking feature

	if(tcount==64){ // half a second passed
		colflag=0; // colon is off
		if(setflag == 1)
			blinkflag=0; // toggle blinking feature
	}
	if(tcount==32) // 3/4 a sec passed
		if(setflag == 1) // if setflag is enabled
			blinkflag=1; // toggle blinking feature

	if(tcount==128){ // one second passed
		++tick; // increment a second
		colflag=1; // colon is on
		if(setflag == 1) // if setflag is enabled
			blinkflag=0; // toggle blinking feature
		uart_putc('s'); // request temprature from the remote
		uart_putc('\0'); // send terminator
		tcount=0;
	}
	if(snooze==tick) // if the snooze is done 
		snooze=100; // reset the snooze
	if(tick == 60){ // 60 second has passed
		++total; // increment total
		tick=0;
		if(total == 1440) // when 24:00 is reached, roll over
			total=0;
		segsum(total);
	}

} // TC0 ISR


//***********************************************************************************
uint8_t main()
{
	uint16_t lm73_temp;  //a place to assemble the temperature from the lm73
	uint8_t i, barkeeper, alarmflag, moodflag, lcdflag=0;

	DDRD &= ~(0xF0); // higher 4 pins for input
	PORTD |= (0xF0); // higher 4 pins pullups
	EICRB |= (1<<ISC71) | (1<ISC70); // extrl int7 on raising edge
	EIMSK |= (1<<INT7); // enable via the mask
	DDRA = 0xFF; 	// PortA as output
	DDRB = 0xFF; 	// PortB as output
	uart_init(); // initalize uart
	spi_init();  //initalize SPI
	init_tcnt0(); //initalize timer counter zero
	init_tcnt2(); // initalize timer counter two
	init_tcnt1(); // initialize timer counter one
	init_tcnt3(); // initilize timer counter three
	lcd_init(); // initilize the LCD
	init_twi(); // initalize TWI (twi_master.h)  
	clear_display(); // clear LCD
	init_adc(); // initilize ADC
	total = 720; 	// time initially 12:00
	ADCSRA |= (1<<ADSC); // poke ADSC to start the first conversion
	sei();  //enable global interrupts

	update_lm73(); // initalize lm73 (lm73.h)
	uart_putc('s'); // get initial temp at remote
	uart_putc('\0'); // terminator
	while(1){	
		memset(temp_lcd, '\0', sizeof(temp_lcd)); // Clear out the buffer again for reuse
		memset(lcd_string, '\0', sizeof(lcd_string)); // Clear out the buffer again for reuse
		twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2); //read temperature data from LM73 (2 bytes) 
		lm73_temp = recv_lm73(); // call recieve function (lm73.h)
		lm73_temp_convert(temp_array, lm73_temp, 1); // (lm73.h)
		strcat(temp_array, "C    "); // add index	
		strcat(temp_lcd, temp_array); // write temp to lcd 
		if(rcv_rdy == 1){ // if remote temp is updated
			strcpy(remote_temp,remote_array); // copy new message
			strcat(remote_temp, "C"); // write type to lcd 
			rcv_rdy = 0; // clear flag
		}
		strcat(temp_lcd, remote_temp); // write remote temp to lcd 
		// set PorTA as input
		DDRA = 0x00;
		// activate pull-ups
		PORTA = 0xFF; 
		asm volatile ("nop");
		asm volatile ("nop");
		asm volatile ("nop");
		// enable tristate buffer for pushbutton switches and disable tcnt2 PWM mood
		TCCR2 ^= ((1<<WGM21) | (1<<WGM20) | (1<<COM21) | (1<<COM20) | (1<<CS20)); // PWM, 1 prescale, set on match
		PORTB = digit[5];
		// zero out values at the beginning of the loop
		i = 0;
		while(i<4){	// loops to check 4 buttons of PORTD
			switch(chk_buttons(i,1)){	// switch statement to check if it's pressed or not
				case 0: break;	// if not
				case 1: 	// if button is pressed
					switch(i){	// i is the index (which button was detected)
						case 0: current_fm_freq = 8870; // PD4, preset station 1 	
							break;
						case 1: current_fm_freq = 9990; // PD5, 2nd station	
							break;	
						case 2: current_fm_freq = 10710; // PD6, 3rd station
							break;
						case 3: current_fm_freq = 10790; // PD7, 4th 
							break;
						default: break;
					} // inner switch
					if(radio_en == 1)
						fm_tune_freq(); // update freq
					break; // end of case 1
				default: break;
			} // outter switch
			++i;
		} // while

		i = 0;
		while(i<7){	// loops to check buttons for PORTA
			switch(chk_buttons(i,0)){	// switch statement to check if it's pressed or not
				case 0: break;	// if not
				case 1: 	// if button is pressed
					switch(i){	// i is the index (which button was detected)
						case 0: fmode[i] = ~(fmode[i]); // PA0, clock set mode
							break;
						case 1: fmode[i] = ~(fmode[i]); // PA1, alarm set mode	
							break;	
						case 2: set_alarm = ~(set_alarm); // PA2, turn on/off the alarm
							break;
						case 3: if(alarmflag ==1){ // PA3, snooze
								snooze = tick + 10; // get current seconds + 10
								if(snooze > 60) // if it exceed 60 roll over
									snooze = snooze % 60;
							}
							else // if the alarm is not ringing
								snooze=100;
							break;
						case 4: freq_tune = ~(freq_tune); // PA4, frequency tone 
							break;
						case 5: if (radio_en == 0){ // PA5, radio enable 
								radio_on();
							}
							else if(radio_en == 1){ // disable radio
								radio_pwr_dwn();
								radio_en=0;
								OCR3A = 0xF000; // mute 
							}
							break;
						case 6: alarm_mood = ~(alarm_mood); // PA6, alarm mode 
							break;

						default: break;
					} // inner switch
					break; // end of case 1
				default: break;
			} // outter switch
			++i;
		} // while	
		// put back tcnt2 to PWM mood
		TCCR2 ^= ((1<<WGM21) | (1<<WGM20) | (1<<COM21) | (1<<COM20) | (1<<CS20)); // PWM, 1 prescale, set on match
		// state managment
		if ((freq_tune == 0xFF)&&(setflag == 0)){ // radio managment
			state = TONE;
		}
		else
		{ // clock managment 
			if ((fmode[0]==0)&&(fmode[1]==0)) // if both buttons unpressed
				state = NONE;
			else if ((fmode[0]==0xFF)&&(fmode[1]==0)) // button S0 is pressed
				state = CLOCK_C;
			else if ((fmode[0]==0)&&(fmode[1]==0xFF)) // button S1 is pressed
				state = ALARM_C;
			else if((fmode[0]==0xFF)&&(fmode[1]==0xFF))
				state = BOTH;
		}
		if(set_alarm == 0xFF){ // if alarm button is set
			barkeeper = 0x80; // display on bargraph	
			if((atotal == total)&&(snooze ==100)){ // if alarm is ringing
				if(moodflag == 1)
					clear_display();	
				moodflag=0;
				if(colflag==0) // if colon is off 
					barkeeper|=0xFF; // update the value of the bargraph
				if((alarm_mood==0) && (alarmflag==0)){
					radio_pwr_dwn();
					DDRC |= ((1<<PC5)|(1<<PC6)); // turn on the alarm sound
				}
				else if((alarm_mood == 0xFF)&&(alarmflag==0))
					radio_on(); 	
				DDRE |= (1<<PE3); // increase volume
				OCR3A = 0x0000; 
				alarmflag=1; // set alarm flag 
			}
			else{ // if alarm is off	
				if(alarm_mood == 0){ // alarm mood				
					strcat(lcd_string, alarm_c);	
					DDRC &= ~((1<<PC5)|(1<<PC6)); // turn off sound signal
					if(alarmflag == 1)
						OCR3A = 0xF000; // mute 
					if((radio_en==1)&&(alarmflag==1))
						radio_on();
				}
				else if(alarm_mood == 0xFF){ // radio mood 
					strcat(lcd_string, radio_c); // update lcd 
					if(alarmflag == 1){
						radio_pwr_dwn(); // turn off radio 
						radio_en=0; // disable 
					}
				}
				alarmflag=0; // disable alarm flag
				moodflag = 1;
			}
		}
		else if(set_alarm==0){ // if set alarm is turn off
			barkeeper = 0; // display on bargraph
			if(alarm_mood == 0){				
				DDRC &= ~((1<<PC5)|(1<<PC6)); // turn off sound signal
				if(alarmflag == 1)
					OCR3A = 0xF000;
				if((radio_en==1)&&(alarmflag==1))
					radio_on();
				}
				else if(alarm_mood == 0xFF){ // radio mood 
					if(alarmflag == 1){
						radio_pwr_dwn();
						radio_en=0;
					}
				}
			if(moodflag == 1)
				clear_display();	
			moodflag=0;
			alarmflag=0;
		}

		switch (state) { // state machine
			case NONE: { // intial mood -> display time regularly 
					   if((lcdflag > 1)){ // if lcd is nor cleared
						   clear_display(); // clear lcd
						   lcdflag=0;
						   line2_col1(); // line 2
						   string2lcd(" "); // write
						   string2lcd(temp_lcd); // write

					   }	
					   line2_col1(); // line 2
					   string2lcd(" "); // write
					   string2lcd(temp_lcd); // write
					   cursor_home();
					   if(moodflag == 1)
						   string2lcd(lcd_string);
					   blinkflag=0; // no blinking 
					   setflag = 0; // disable set flag
					   segsum(total); // manage time
					   SPDR = barkeeper; // send the mode display to the bargraph
					   while (bit_is_clear(SPSR,SPIF)){} //spin till SPI data has been sent 
					   PORTD |= (1<<PD1);            //send rising edge to regclk on HC595
					   PORTD &= ~(1<<PD1);            //send falling edge to regclk on HC595
					   display_t(); // call diplay function
					   break;
				   }
			case TONE: { // set clock mood  
					   cursor_home(); // initiate the cursor
					   strcat(lcd_string, tone_s); // write to lcd 
					   string2lcd(lcd_string);
					   line2_col1(); // line 2
					   string2lcd(" "); // write
					   string2lcd(temp_lcd); // write
					   lcdflag=4; // change the flag
					   segsum(current_fm_freq); // update display 
					   enc_read(vol_logic()); // write to bargraph the volume and read from encoders
					   switch(Fsign){ // depending on direction of encoder **1st encoder** Frequency
						   case 0: // decrement 
							   current_fm_freq -= 20;	// decrement the freq
							   changeflag = 1;
							   if(current_fm_freq < 8810) // when 88.1 is reached, roll over
								   current_fm_freq = 10790;
							   break;
						   case 1:	// increment 
							   current_fm_freq += 20;	// increment the freq
							   changeflag = 1;
							   if(current_fm_freq > 10790) // when 107.9 is reached, roll over
								   current_fm_freq = 8810;
							   break;
						   default: break;
					   }
					   switch(Ssign){ // **second encoder** Volume
						   case 0: // decrement 
							   vol_level += 0x1000;	// decrease volume
							   if (vol_level > 0xF000)	// if out of bounderies, don't change			
								   vol_level = 0xF000;
							   if(radio_en ==1)
								   OCR3A = vol_level; // update volume
							   break;
						   case 1:	// increment 
							   vol_level -= 0x1000;	// increase volume	
							   if (vol_level > 0xF000)	// if out of bounderies, don't change 
								   vol_level = 0x0000;
							   if(radio_en ==1)
								   OCR3A = vol_level; // update volume
							   break;
						   default: break;
					   }
					   if((radio_en == 1)&&(changeflag == 1)){
						   fm_tune_freq();
						   changeflag = 0;
					   }
					   Ssign = 10; // reset direction
					   Fsign = 10; // reset direction
					   display_t(); // display digit
					   break;
				   }
			case CLOCK_C: { // set clock mood  
					      cursor_home(); // initiate the cursor
					      strcat(lcd_string, clock_s); // write to lcd 
					      string2lcd(lcd_string);
					      line2_col1(); // line 2
					      string2lcd(" "); // write
					      string2lcd(temp_lcd); // write
					      lcdflag=2; // change the flag
					      setflag=1; // enable set flag for digit to blink
					      segsum(total); // update time 
					      enc_read(0xF0); // write to bargraph and read from encoders
					      switch(Fsign){ // depending on direction of encoder
						      case 0: // decrement 
							      total -= 60;	// decrement the total by the value in incr
							      if(total > 1441) // when 00:00 is reached, roll over
								      total=1380;
							      break;
						      case 1:	// increment 
							      total += 60;	// increment the total by the value in incr	
							      // check on the total to make sure it's within the boundries
							      if(total > 1439) // when 24:00 is reached, roll over
								      total=0;
							      break;
						      default: break;
					      }
					      switch(Ssign){ // depending on direction of encoder
						      case 0: // decrement 
							      total -= 1;	// decrement the total by the value in incr
							      if(total > 1441) // when 00:00 is reached, roll over
								      total=1439;
							      break;
						      case 1:	// increment 
							      total += 4;	// increment the total by the value in incr	
							      // check on the total to make sure it's within the boundries
							      if(total > 1440) // when 24:00 is reached, roll over
								      total=0;
							      break;
						      default: break;
					      }
					      Ssign = 10; // reset direction
					      Fsign = 10; // reset direction
					      display_t(); // display digit
					      break;
				      }
			case ALARM_C: { // alarm set mood 
					      setflag=1; // set flag is set
					      cursor_home(); // initiate cursor 
					      strcat(lcd_string, alarm_s); // write to lcd 
					      string2lcd(lcd_string);
					      line2_col1(); // line 2
					      string2lcd(" "); // write
					      string2lcd(temp_lcd); // write
					      lcdflag=3; // change flag
					      segsum(atotal); // update alarm total
					      enc_read(0x0F); // write to bargraph and read encoder
					      switch(Fsign){ // depending on direction of encoder
						      case 0: // decrement 
							      atotal -= 60;	// decrement the total by the value in incr
							      if(atotal > 1441) // when 00:00 is reached, roll over
								      atotal=1380;
							      break;
						      case 1:	// increment 
							      atotal += 60;	// increment the total by the value in incr	
							      // check on the total to make sure it's within the boundries
							      if(atotal > 1439) // when 24:00 is reached, roll over
								      atotal=0;
							      break;
						      default: break;
					      }
					      switch(Ssign){ // depending on direction of encoder
						      case 0: // decrement 
							      atotal -= 1;	// decrement the total by the value in incr
							      if(atotal > 1441) // when 00:00 is reached, roll over
								      atotal=1439;
							      break;
						      case 1:	// increment 
							      atotal += 4;	// increment the total by the value in incr	
							      // check on the total to make sure it's within the boundries
							      if(atotal > 1440) // when 24:00 is reached, roll over
								      atotal=0;
							      break;
						      default: break;
					      }
					      Ssign = 10; // reset direction
					      Fsign = 10;
					      display_t(); // display 
					      break;
				      }
			case BOTH: { // Both mood, if both alarm set and clock set is pressed display clock only 
					   blinkflag=0; // no blink
					   cursor_home(); // line 1
					   strcat(lcd_string, both); // write to lcd 
					   string2lcd(lcd_string); 
					   line2_col1(); // line 2
					   string2lcd(" "); // write
					   string2lcd(temp_lcd); // write
					   lcdflag=4; // change flag 
					   segsum(total); // manage time 
					   SPDR = 0xFF; // send the mode display to the bargraph
					   while (bit_is_clear(SPSR,SPIF)){} //spin till SPI data has been sent
					   PORTD |= (1<<PD1);            //send rising edge to regclk on HC595
					   PORTD &= ~(1<<PD1);            //send falling edge to regclk on HC595
					   display_t();
					   break;
				   }
			default: break;
		} // switch
	}// while
}// main
