//UART Example for inclass coding
//Roger Traylor 12.4.12
//Connect two mega128 boards via rs232 and they should end to each
//other a message and a sequence number.
//
//Change the message you send to your partner for checkoff.
//
//You can test this code by a "loopback" if you connect rx to tx
//on the DB9 connector.
//

#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include "uart_functions_m48.h"
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lm73_functions.h"
#include "twi_master.h"

uint8_t           i;
volatile uint8_t  rcv_rdy;
volatile uint8_t  send_rdy = 0;
volatile char              rx_char; 
uint8_t           send_seq=0;         //transmit sequence number
char              lcd_string[3];      //holds value of sequence number
char    lcd_str_array[16];  //holds a string to refresh the LCD
char    lcd_string_array[16];  //holds a string to refresh the LCD
extern uint8_t lm73_wr_buf[2];
extern uint8_t lm73_rd_buf[2];



int main(){
	uint16_t lm73_temp;  //a place to assemble the temperature from the lm73
	init_twi();
	uart_init();
//	update_lm73();

	sei();
	while(1){
		//**************  start rcv portion ***************
		if(rcv_rdy==1){
			if(rx_char == 's'){

			}
			rcv_rdy=0;
		}//if 
		//**************  end rcv portion ***************

		//**************  start tx portion ***************
		twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2); //read temperature data from LM73 (2 bytes) 
		_delay_ms(2);    //wait for it to finish
		lm73_temp = recv_lm73(); // call recieve function 
		lm73_temp_convert(lcd_string_array, lm73_temp, 1);
		uart_puts(lcd_string_array);
		uart_putc('\0');
		for(i=0;i<=9;i++){_delay_ms(100);}
		//**************  end tx portion ***************
	}//while
}//main

ISR(USART_RX_vect){
	static  uint8_t  i;
	rx_char = UDR0;              //get character
	lcd_str_array[i++]=rx_char;  //store in array 
	//if entire string has arrived, set flag, reset index
	if(rx_char == '\0'){
		rcv_rdy=1; 
		lcd_str_array[--i]  = (' ');     //clear the count field
		lcd_str_array[i+1]  = (' ');
		lcd_str_array[i+2]  = (' ');
		i=0;  
	}
}
//************************************//

