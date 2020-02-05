// lab2_code.c 
// Skeleton code is made by R. Traylor, 9.12.08
// Modified by Ibrahim Alarifi
// Modified on 10/11/2019


//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <util/delay.h>

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
uint8_t dstate[4] = {1,0,0,0};

// holds the current value of each digit, initially all digits are zero
uint8_t dig_value[4] = {
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
	static uint16_t state[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // holds present state of each button
	
	// this perform an or operation with the previous state value, !bit_is_clear output, and 0xE000
	// when the button is pressed, !bit_is_clear return 1 per debounce, with the last debounce value
	// it will start shifting and or-ing, untill state = 0xF000, then return 1 (button is pressed)
	state[button] = (state[button] << 1) | (! bit_is_clear(PINA, button)) | 0xE000;
	if (state[button] == 0xF000) 
		return 1;
	else
		return 0;
}
//******************************************************************************

//***********************************************************************************
//                                   segment_sum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array dig_value for display.                       
//array is loaded at exit as:  |digit3|digit2|digit1|digit0|
void segsum(uint16_t incr) {
	uint8_t i = 0; // loop counter
	dig_value[0] += incr; // increment 1st digit in the array based on user's choice
	while (i == 0){ // 1st digit loop
		if(dig_value[0] > 9){	// if the value of 1st digit is higher than 9
			++dig_value[1];	// increment the 2nd digit
		        dstate[1] = 1;	// enable state	
			dig_value[0] -= 10; // decrement the 1st digit by 10
		}
		else	// if the value <= 9 exit the loop
			i = 1;
	}
	while (i == 1){ // 2nd digit loop
		if(dig_value[1] > 9){ // if the value of 2nd digit is higher than 9
			++dig_value[2]; // increment the 3rd digit
		        dstate[2] = 1;	// enable state	
			dig_value[1] -= 10; // decrement the 2nd digit by 10
		}
		else // if the value <= 9, exit the loop
			i = 0;
	}
	while (i == 0){ // 3rd digit
		if(dig_value[2] > 9){ // if the value of 3rd digit is higher than 9
			++dig_value[3]; // increment 4th digit
		        dstate[3] = 1;	// enable state	
			dig_value[2] -= 10; // decrement 3rd digit by 10
		}
		else // if the value <= 9, exit the loop
			i = 1;
	}
	// the forth digit doesn't need a loop since the counter roll over when it reaches max
	// which is in this case, max = 1023

}//segment_sum
//***********************************************************************************


//***********************************************************************************
uint8_t main()
{
	uint16_t total, incr = 1;	// keep track of increment and total
	uint8_t i, j = 0; 	// loops
	DDRA = 0xFF; 	// PortA as output
	DDRB = 0xFF; 	// PortB as output
	PORTA = dec_to_7seg[0]; 	// Display 0 initially
	while(1){
		// set PorTA as input
		DDRA = 0x00;
		// activate pull-ups
		PORTA = 0xFF; 
		// enable tristate buffer for pushbutton switches
		PORTB = digit[5];
		// zero out values at the beginning of the loop
		i = 0;
		incr = 0;
		while(i<8){	// loops to check on all the 8 buttons
			for(j=0;j<6;j++){	// loops 6 times per button, I choose 6 to allow time for button to be detected
				switch(chk_buttons(i)){	// switch statement to check if it's pressed or not
					case 0: break;	// if not
					case 1: 	// if button is pressed
						switch(i){	// i is the index (which button was detected
							case 0: incr = 1; _delay_ms(1);	// PA0, increment by 1 and delay
								break;
							case 1: incr = 2; _delay_ms(1);	// PA1, increment by 2 and delay
								break;
							case 2: incr = 4; _delay_ms(1);	// PA2, increment by 4 and delay
								break;
							case 3: incr = 8; _delay_ms(1);	// PA3, increment by 8 and delay
								break;
							case 4: incr = 16; _delay_ms(1); // PA4, increment by 16 and delay
								break;
							case 5: incr = 32; _delay_ms(1); // PA5, increment by 32 and delay
								break;
							case 6: incr = 64; _delay_ms(1); // PA6, increment by 64 and delay
								break;
							case 7: incr = 128; _delay_ms(1); // PA7, increment by 128 and delay
								break;
							default: break;
						} // inner switch
						break; // end of case 1
					default: break;
				} // outter switch
			} // for
			++i;
		} // while
		total = total + incr;	// increment the total by the value in incr
		// disable tristate buffer for pushbutton switches
		PORTB = digit[0];
		// check on the total to make sure it's within the boundries
		if (total > 1023){	// if out of bounderies, roll over starting from one
			dig_value[0] = 1;	// digit 1
			dig_value[1] = 0;	// digit 2
			dig_value[2] = 0;	// digit 3
			dig_value[3] = 0;	// digit 4
			dstate[1] = 0;
			dstate[2] = 0;
			dstate[3] = 0;
			total=1;	// clear the total
		}
		else // if not out of bounderies, call segsum to update the values with the new ones
			segsum(incr);	
		// make PortA an output
		DDRA = 0xFF;
		// a loop to display all digit 
		for(i=0; i<4; i++){
			// update PORTA with the corresponding digit value (ith digit)
			if(dig_value[i] == 0 && dstate[i] == 0)
				PORTA = 0xFF;
			else 
				PORTA = dec_to_7seg[dig_value[i]];
			// update PORTB to turn on the digit (ith digit)
			PORTB = digit[i];
			// delay to balance on/off time
			_delay_ms(1);
		}
	}// while
}// main
