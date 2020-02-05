// siren.c  - R. Traylor  - 10.31.2013
// 
// Setup TCNT1 to be a varaible frequency tone generator. Its made variable by dynamically 
// changing the compare register.
//
// Setup TCNT3 to interrupt the processor at a rate of 1000 times a second.  When the 
// interrupt occurs, the ISR for TCNTR3 changes the frequency of timer TCNT1 to affect 
// a continually changing audio frequency at PORTB bit 5.
//
// set OC1A (PB5) as the audio output signal PORTB (PB7) is the PWM signal for creating 
// the volume control.
//
// Timer TCNT3 is set to interrupt the processor at a rate of 1000 times 
// a second.  When the interrupt occurs, the ISR for TCNTR3 changes the
// frequency of timer TCNT1 to affect a continually changing audio
// frequency at PORTB bit 5.
//
// to download: 
// wget http://www.ece.orst.edu/~traylor/ece473/inclass_exercises/timers_and_counters/siren_skeleton.c

#include <avr/io.h>
#include <avr/interrupt.h>

ISR(TIMER1_COMPA_vect) {
	OCR1A = OCR1A+100;
	if(OCR1A > 0xF000)
		OCR1A = 0x0258;
	PORTC ^= (1<<PC5);
}

int main(){

  DDRC   = (1<<PC5);                       //set port B bit five and seven as outputs
  DDRE   = (1<<PE3);

//setup TCNT1

  TCCR1A = (0<<COM1A1) | (0<<COM1A0);                 //CTC mode with output pin on 

  TCCR1B = (1<<WGM12) | (1<<CS10);                         //use OCR1A as source for TOP, use clk/1

  TCCR1C = 0x00;                          //no forced compare 

  OCR1A = 0x0258;

  TIMSK = (1<<OCIE1A);

//setup TCNT3
// siren update frequency = (16,000,000)/(OCR3A) ~ set to about 1000 cycles/sec

  TCCR3A = (1<<COM3A1) | (1<<COM3A0) | (1<<WGM31);                           //CTC mode, output pin does not toggle 

  TCCR3B = (1<<WGM33) | (1<<WGM32) | (1<<CS30);                          //no prescaling      

  TCCR3C = 0x00;                          //no forced compare 

  OCR3A = 0x3000;                           //pick a speed from 0x1000 to 0xF000

  ICR3 = 0xF000;


   sei();     //set GIE to enable interrupts
  while(1){} //do forever
 
}  // main
