// adc_skel.c 
//A simple voltmeter with the ADC operating in 10 bit mode with no interrupts, 
//coarse ADC accuracy, no quiet mode. Skeletonized by R. Traylor 11.1.2016

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "hd44780.h"

uint8_t  i;              //dummy variable
uint16_t adc_result;     //holds adc result 

ISR(ADC_vect){
	adc_result = ADC;
	adc_result = (adc_result * 5)/1024;
	i = adc_result;
	switch(i){
		case 0:
			OCR2 = 0x33;
			break;
		case 1:
			OCR2 = 0x66;
			break;
		case 2:
			OCR2 = 0x99;
			break;
		case 3:
			OCR2 = 0xCC;
			break;
		case 4:
			OCR2 = 0xFF;
			break;
		default:
			break;
	}

	ADCSRA |= (1<<ADSC); //poke ADSC and start conversion
}

int main()
{
	//Initalize ADC and its ports
	DDRF  = (0<<PF7); //make port F bit 7 is ADC input  
	PORTF = (0<<PF7);  //port F bit 7 pullups must be off
	DDRB = 0xFF;
	DDRA = 0xFF;

	ADMUX = (1<<REFS0) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0); //writes 00111 to ADMUX (4:0) for single-ended, input PORTF bit 7, right adjusted, 10 bits

	ADCSRA = (1<<ADEN) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //ADC enabled, don't start yet, single shot mode 

	TCCR2 = (1<<WGM21) | (1<<WGM20) | (1<<COM21) | (1<<COM20) | (1<<CS22);

	OCR2 = 0xFF;

	ADCSRA |= (1<<ADSC); //poke ADSC and start conversion
	sei();
	
	PORTA = 0b10000000;
	PORTB |= 0x00;
	while(1){ 

	
	} //while
}//main
