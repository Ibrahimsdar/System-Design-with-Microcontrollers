// mega48.c
// Ibrahim Alarifi 12.03.2019
// This is the mega48 remote code. It recieve request signal from host, then send back temprature

#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include "uart_functions_m48.h"
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lm73_functions.h"
#include "twi_master.h"

uint8_t           i;;
volatile uint8_t  send_rq = 0;
volatile char              rx_char; 
uint8_t           send_seq=0;       // transmit sequence number
char              lcd_string[3];   // holds value of sequence number
char    lcd_string_array[16];  // holds a string to refresh the LCD
extern uint8_t lm73_wr_buf[2];
extern uint8_t lm73_rd_buf[2];



int main(){
	uint16_t lm73_temp;  // a place to assemble the temperature from the lm73
	init_twi();
	uart_init();

	sei();
	update_lm73();
	while(1){
		if(send_rq==1){
			twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2); //read temperature data from LM73 (2 bytes) 
			lm73_temp = recv_lm73(); // call recieve function 
			lm73_temp_convert(lcd_string_array, lm73_temp, 1); // convert it
			uart_puts(lcd_string_array); // send it 
			uart_putc('\0'); // send end terminator 
			send_rq=0; // clear flag
		}//if 
	}//while
}//main

ISR(USART_RX_vect){
	rx_char = UDR0;              //get character
	if(rx_char == 's') 	// if it's a request message
		send_rq=1; 	// request flag
}
