//Radio test code

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>
#include "twi_master.h"
#include "uart_functions.h"
#include "si4734.h"

extern enum radio_band{FM, AM, SW};
extern volatile uint8_t STC_interrupt;

volatile enum radio_band current_radio_band = FM;

uint16_t eeprom_fm_freq;
uint16_t eeprom_am_freq;
uint16_t eeprom_sw_freq;
uint8_t  eeprom_volume;

volatile uint16_t current_fm_freq = 8810;
volatile uint16_t current_am_freq;
volatile uint16_t current_sw_freq;
uint8_t  current_volume;


//....................................................................
// holds the values to control the decoder (74HC138) of the 7-segment, to turn on specific digits
uint8_t digit[6] = {
	0x00,	// 1st digit 
	0x10,	// 2nd digit
	0x30,	// 4th digit
	0x40,	// 5th digit
	0x20,	// 3rd digit (dots)
	0x70 	// off state
};

// holds the state of each button: 0=off, 1=on
volatile uint8_t dstate[4] = {1,0,0,0};

// holds the current value of each digit, initially all digits are zero
volatile uint8_t dig_value[4] = {
	0,
	0,
	0,
	0
};

// decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[10] = {
	0b11000000,	// zero
	0b11111001,	// one 
	0b10100100,	// two
	0b10110000,	// three
	0b10011001,	// four
	0b10010010,	// five
	0b10000010,	// six
	0b11111000,	// seven
	0b10000000,	// eight
	0b10010000	// nine
};

//declare variables that is outside scope of main
volatile uint16_t total=8810;	// keep track of increment and total
volatile uint8_t readbuff;	// reads data from SPDR
volatile uint8_t enc1[2];	// new and old value of encouder 1
volatile uint8_t enc2[2];	// new and old value of encouder 2
volatile uint8_t sign = 10;	// 1->increment, 0->decrement
volatile uint8_t vol = 10;	// 1->increment, 0->decrement
volatile uint8_t changeflag = 0; //
volatile uint32_t vol_level = 0x0000;
// this flag keeps track of mode both

uint8_t dcount;	// display counter


//***********************************************************************
//                           init_tcnt0
//***********************************************************************
//initalize timer/counter zero to CTC mode

void init_tcnt0(){

	TIMSK = (1<<TOIE0); //enable interrupt
	TCCR0 |=  (1<<CS02) | (1<<CS00);  //normal mode, prescale by 128
}

//***********************************************************************
//                            spi_init                               
//**********************************************************************
void spi_init(void){

	DDRE   |= (1<<PE6); // sh/ld control
	DDRD	= (1<<PD1); // rgclk control

	SPCR   = (1<<SPE) | (1<<MSTR); //master mode, clk low on idle, leading edge sample
	SPSR   = (1<<SPI2X); //choose double speed operation
}//spi_init

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

//***********************************************************************************
//                                   segment_sum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array dig_value for display.                       
//array is loaded at exit as:  |digit3|digit2|digit1|digit0|
void segsum(uint16_t total) {	
	uint16_t buff, val, j;
	uint8_t i;
	j = 10;
	for(i=0;i<4;i++){ // this loop divide total into 4-digits
		val = 0;
		buff = total/j; // get the ith number in total
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
}//segment_sum

//***********************************************************************
//                     ISR for timer counter zero
//***********************************************************************
ISR(TIMER0_OVF_vect){
	uint16_t incr;
	uint8_t i = 0; // loop counter
	// set PorTA as input
	DDRA = 0x00;	
	// activate pull-ups
	PORTA = 0xFF;
	// enable tristate buffer for pushbutton switches
	PORTB = digit[5];
	asm volatile ("nop");
	asm volatile ("nop");
	// zero out values at the beginning of the loop
	incr = 20;
	sign = 10;
	vol = 10;

	PORTE = (0<<PE6);	// SD/LD low to latch input to flip flop
	asm volatile ("nop");
	PORTE = (1<<PE6);	// SD/LD high to allow shifting input to output pin
	SPDR = incr; // send the mode display to the bargraph
	while (bit_is_clear(SPSR,SPIF)){} //spin till SPI data has been sent 
	PORTD |= (1<<PD1);            //send rising edge to regclk on HC595
	PORTD &= ~(1<<PD1);            //send falling edge to regclk on HC595
	readbuff = SPDR;	// read input from encouder
	enc1[0] = (readbuff & 0b00000011); // get encouder 1 A and B bits 
	enc2[0] = (readbuff & 0b00001100); // get encouder 2 A and B bits

	if(enc1[0] != enc1[1]){ // 1st encoder
		if((enc1[1]==0b00000010 && enc1[0]==0b00000000)){ // clockwise
			sign = 1; // increment
			changeflag = 1;
			enc1[0]=0x0F;
			enc1[1]=0x0F;
		}
		else if((enc1[1]==0b00000000 && enc1[0]==0b00000010)){ // counter clockwise
			sign = 0; // decrement
			changeflag = 1;
			enc1[0]=0x0F;
			enc1[1]=0x0F;
		}
	}
	enc1[1] = enc1[0]; // update old value

	if(enc2[0] != enc2[1]){ // 2nd encoder
		if((enc2[1]==0b00001000 && enc2[0]==0b00000000)){ // clockwise
			vol = 1; // increment
			changeflag = 1;
			enc2[0]=0x0F;
			enc2[1]=0x0F;
		}
		else if((enc2[1]==0b00000000 && enc2[0]==0b00001000)){ // counter clockwise
			vol = 0; // decrement
			changeflag = 1;
			enc2[0]=0x0F;
			enc2[1]=0x0F;
		}
	}
	enc2[1] = enc2[0]; // update old value

	switch(sign){
		case 0: // decrement 
			total -= incr;	// decrement the total by the value in incr
			if (total < 8810){	// if out of bounderies, roll over starting from 1023			
				total=10790;	// set the total
				segsum(total);
			}
			else { // if not out of bounderies, call segsum to update the values with the new ones
				segsum(total);
			}
			_delay_ms(8);
			break;
		case 1:	// increment 
			total += incr;	// increment the total by the value in incr	
			// check on the total to make sure it's within the boundries
			if (total > 10790){	// if out of bounderies, roll over starting from one
				total=8810;	// clear the total
				segsum(total);
			}
			else { // if not out of bounderies, call segsum to update the values with the new ones
				segsum(total);
			}
			_delay_ms(8);
			break;
		default: break;
	}
	switch(vol){
		case 0: // decrement 
			vol_level += 0x1000;	// decrement the total by the value in incr
			if (vol_level > 0xF000){	// if out of bounderies, roll over 			
				vol_level = 0xF000;	// set the total
			}
			_delay_ms(8);
			break;
		case 1:	// increment 
			vol_level -= 0x1000;	// increment the total by the value in incr	
			// check on the total to make sure it's within the boundries
			if (vol_level > 0xF000){	// if out of bounderies, roll over starting from one
				vol_level = 0x0000;	// clear the total
			}
			_delay_ms(8);
			break;
		default: break;
	}
	OCR3A = vol_level;

	// make PortA an output
	DDRA = 0xFF;
	asm volatile ("nop");
	if(dig_value[dcount] == 0 && dstate[dcount] == 0) // maintain previous state
		PORTA = 0xFF;
	else
	{	
		if(dcount == 1)	
			PORTA = (0b01111111 & dec_to_7seg[dig_value[dcount]]);
		else
			PORTA = dec_to_7seg[dig_value[dcount]];	
	}
	// update PORTB to turn on the digit (ith digit)
	PORTB = digit[dcount];


}
//***********************************************************************************


//Used in debug mode for UART1
//char uart1_tx_buf[40];      //holds string to send to crt
//char uart1_rx_buf[40];      //holds string that recieves data from uart


//******************************************************************************
// External interrupt 7 is on Port E bit 7. The interrupt is triggered on the
// rising edge of Port E bit 7.  The i/o clock must be running to detect the
// edge (not asynchronouslly triggered)
//******************************************************************************
ISR(INT7_vect){
	STC_interrupt = TRUE;
}
/***********************************************************************/


int main(){
	init_twi();
	DDRA = 0xFF; 	// PortA as output
	DDRB = 0xFF; 	// PortB as output
	PORTA = dec_to_7seg[0]; 	// Display 0 initially
	PORTB = digit[0];	// initially digit 0
	total = 8810;
	segsum(total);

	init_tcnt0(); //initalize timer counter zero
	spi_init();  //initalize SPI port
	init_tcnt3(); // initilize timer counter three
	

	//Setup audio output (max)
	PORTE |= (1 << PE3);



	EICRB |= (1<<ISC71) | (1<ISC70);
	EIMSK |= (1<<INT7);






	sei();

	DDRE  |= (1 << PE2); //Port E bit 2 is active high reset for radio 
	PORTE |= (1 << PE2); //radio reset is on at powerup (active high)
	//hardware reset of Si4734
	PORTE &= ~(1<<PE7); //int2 initially low to sense TWI mode
	DDRE  |= 0x80;      //turn on Port E bit 7 to drive it low
	PORTE |=  (1<<PE2); //hardware reset Si4734 
	_delay_us(200);
	PORTE &= ~(1<<PE2); //release reset 
	_delay_us(30);
	DDRE  &= ~(0x80);   //now Port E bit 7 becomes input from the radio interrupt
	fm_pwr_up(); //powerup the radio as appropriate
//	current_fm_freq = 8810; //arg2, arg3: 99.1Mhz, 200khz steps
//	fm_tune_freq(); //tune radio to frequency in current_fm_freq

	

	while(1){

		for(dcount=0; dcount<4; dcount++){
			// update PORTB to turn on the digit (ith digit)
			PORTB = digit[dcount];
			// update PORTA with the corresponding digit value (ith digit)
			if(dig_value[dcount] == 0 && dstate[dcount] == 0)
				PORTA = 0xFF;
			else
			{	
				if(dcount == 1)	
					PORTA = (0b01111111 & dec_to_7seg[dig_value[dcount]]);
				else
					PORTA = dec_to_7seg[dig_value[dcount]];
			}
			// delay to balance on/off time
			_delay_ms(1);
		}
		if(changeflag == 1) {
			current_fm_freq = total; // 9910 
			fm_tune_freq();
			changeflag = 0;
		}
	}

}
