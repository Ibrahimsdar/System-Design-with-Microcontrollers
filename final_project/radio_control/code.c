// lab3_code.c 
// Written by Ibrahim Alarifi
// Written on 10/28/2019


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

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

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
// this flag keeps track of mode both
///////////fmode info////////////////
// both 0x00 -> incr 1
// both 0xFF -> no incr
// [0] 0xFF -> incr 2
// [1] 0xFF -> incr 4
////////////////////////////////////
volatile uint8_t fmode[2] = {0,0};	

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

	DDRE   = (1<<PE6); // sh/ld control
	DDRD	 = (1<<PD1); // rgclk control

	SPCR   = (1<<SPE) | (1<<MSTR); //master mode, clk low on idle, leading edge sample
	SPSR   = (1<<SPI2X); //choose double speed operation
}//spi_init


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

//******************************************************************************
//                            chk_buttons                                      
//Checks the state of the button number passed to it. It shifts in ones till   
//the button is pushed. Function returns a 1 only once per debounced button    
//push so a debounce and toggle function can be implemented at the same time.  
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"            
//Expects active low pushbuttons on PINA port.  Debounce time is determined by 
//external loop delay times 12.
//****This function is heavily based on the debounce function provided in lab1 and lecture notes****
uint8_t chk_buttons(uint8_t button) {
	static uint16_t state[2] = {0, 0}; // holds present state of each button	
	// this perform an or operation with the previous state value, !bit_is_clear output, and 0xE000
	// when the button is pressed, !bit_is_clear return 1 per debounce, with the last debounce value
	// it will start shifting and or-ing, untill state = 0xF000, then return 1 (button is pressed)
	state[button] = (state[button] << 1) | (! bit_is_clear(PINA, button)) | 0xE000;
	if (state[button] == 0xF000) 
		return 1;
	else
		return 0;
}

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
			enc1[0]=0x0F;
			enc1[1]=0x0F;
		}
		else if((enc1[1]==0b00000000 && enc1[0]==0b00000010)){ // counter clockwise
			sign = 0; // decrement
			enc1[0]=0x0F;
			enc1[1]=0x0F;
		}
	}
	enc1[1] = enc1[0]; // update old value

	if(enc2[0] != enc2[1]){ // 2nd encoder
		if((enc2[1]==0b00001000 && enc2[0]==0b00000000)){ // clockwise
			sign = 1; // increment
			enc2[0]=0x0F;
			enc2[1]=0x0F;
		}
		else if((enc2[1]==0b00000000 && enc2[0]==0b00001000)){ // counter clockwise
			sign = 0; // decrement
			enc2[0]=0x0F;
			enc2[1]=0x0F;
		}
	}
	enc2[1] = enc2[0]; // update old value

	switch(sign){
		case 0: // decrement 
			total -= incr;	// decrement the total by the value in incr
			if (total < 8810){	// if out of bounderies, roll over starting from 1023			
				total=10810;	// set the total
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
			if (total > 10810){	// if out of bounderies, roll over starting from one
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


//***********************************************************************************
uint8_t main()
{
	DDRA = 0xFF; 	// PortA as output
	DDRB = 0xFF; 	// PortB as output
	PORTA = dec_to_7seg[0]; 	// Display 0 initially
	PORTB = digit[0];	// initially digit 0
	spi_init();  //initalize SPI port
	init_tcnt0();                    //initalize timer counter zero
	total = 8810;
	segsum(total);
	sei();  //enable global interrupts
	while(1){
		// a loop to display all digit 
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
	}// while
}// main
