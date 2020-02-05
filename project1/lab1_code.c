// lab1_code.c 
// R. Traylor
// 7.21.08
// Modified by Ibrahim Alarifi
// Modified on 09/28/2019

//This program increments a binary-codded-decimal (BCD) display of the number of button pushes on switch
//Range 0-99 
//S0 on the mega128 board.

#include <avr/io.h>
#include <util/delay.h>

//*******************************************************************************
//                            debounce_switch                                  
// Adapted from Ganssel's "Guide to Debouncing"            
// Checks the state of pushbutton S0 It shifts in ones till the button is pushed. 
// Function returns a 1 only once per debounced button push so a debounce and toggle 
// function can be implemented at the same time.  Expects active low pushbutton on 
// Port D bit zero.  Debounce time is determined by external loop delay times 12. 
//*******************************************************************************
int8_t debounce_switch() {
  static uint16_t state = 0; //holds present state
  state = (state << 1) | (! bit_is_clear(PIND, 0)) | 0xE000; 
  if (state == 0xF000) return 1;
  return 0;
}

//*******************************************************************************
// Check switch S0.  When found low for 12 passes of "debounce_switch(), increment
// PORTB.  This will make an incrementing count on the port B LEDS. 
//*******************************************************************************
int main()
{
DDRB = 0xFF;  //set port B to all outputs
static uint8_t LSB_Buff, MSB_Buff = 0; // lower and higher parts
static uint8_t PORTB_Buff = 0; // register that will be written to PORTB
while(1){     //do forever
 if(debounce_switch()) {
	 LSB_Buff++; // increment lower bits
	 // edge case 1
	 if(LSB_Buff>9){ // lower bits reached 9 "max", so increment higher bits and roll over
		 MSB_Buff++;
		 LSB_Buff = 0;
	 }
	 // edge case 2
	 if(MSB_Buff>9){ // that means we've reached 99, so start over
		 MSB_Buff = 0;
	 }
	 // asigning stuff
	 PORTB_Buff = 0; // clear the buffer
	 PORTB_Buff = (MSB_Buff << 4); // assign the higher bits by shifting 4-bits
	 PORTB_Buff = PORTB_Buff | LSB_Buff; // logic-or it with the lower bits to get the complete structure
	 PORTB = PORTB_Buff; // write to PORTB
 }  //if switch true for 12 passes, increment port B
  _delay_ms(2);                    //keep in loop to debounce 24ms
  } //while 
} //main
