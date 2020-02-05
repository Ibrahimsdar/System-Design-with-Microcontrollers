// lm73_functions.c       
// Created by Roger Traylor 11.28.2010
// Edited by Ibrahim Alarifi 12.03.2019

#include <util/twi.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include "lm73_functions.h"
#include "twi_master.h"

uint8_t lm73_wr_buf[2];
uint8_t lm73_rd_buf[2];
extern uint8_t lm73_wr_buf[2];
extern uint8_t lm73_rd_buf[2];


//*****************************************************************************
//			update lm73 setting
//****************************************************************************
// no pwr dwn, disbl alert, no one shot: config reg
// no timeout, max resolution: for ctl/status reg
// LM73_CONFIG_VALUE0  to config register
// LM73_CONFIG_VALUE1  to control/status registe	
void update_lm73(){

	lm73_wr_buf[0] = LM73_PTR_CONFIG; // load lm73_wr_buf[0] with config register address
	lm73_wr_buf[1] = LM73_CONFIG_VALUE0; // load lm_wr_buf[1] with written data
	twi_start_wr(LM73_ADDRESS, lm73_wr_buf, 2); // start the TWI write process
	_delay_ms(2);    // wait for the xfer to finishr
	
	lm73_wr_buf[0] = LM73_PTR_CTRL_STATUS; // load lm73_wr_buf[0] with ctrl/status register address
	lm73_wr_buf[1] = LM73_CONFIG_VALUE1; // load lm_wr_buf[1] with written data
	twi_start_wr(LM73_ADDRESS, lm73_wr_buf, 2); // start the TWI write process
	_delay_ms(2);    // wait for the xfer to finishr

	lm73_wr_buf[0] = LM73_PTR_TEMP; // load lm73_wr_buf[0] with temperature pointer address
	twi_start_wr(LM73_ADDRESS, lm73_wr_buf, 1); // start the TWI write process
	_delay_ms(2);    // wait for the xfer to finishr

}// update_lm73

//*****************************************************************************
//			recv_lm73 
//****************************************************************************
// recieve temp data, clean it up and return it
uint16_t recv_lm73(){
	uint16_t temp;

	temp = 0; // clear it 
	temp = lm73_rd_buf[0]; //save high temperature byte into lm73_temp
  	temp = (temp << 8); //shift it into upper byte 
  	temp |= lm73_rd_buf[1]; //"OR" in the low temp byte to lm73_temp

	return temp;
}// recv_lm73


//******************************************************************************
//			convert lm73 temp
//******************************************************************************
// converts raw data to actual tempreture
void lm73_temp_convert(char temp_digits[], uint16_t lm73_temp, uint8_t f_not_c){
	uint8_t sign;	
	uint8_t lm73_low, lm73_high;
	char MSByte[6];
	char LSByte[3];

	sign = (lm73_temp & (1 << (15))); // get the sign of the temp 1-> neg, 0-> pos
	lm73_temp = (lm73_temp << 1); // get rid of the sign bit
	lm73_low = lm73_temp; // get the lower byte of temp
	lm73_low = (lm73_low >> 3); // shift it back to fix index
	lm73_low *= 3; // get the decimal value around 0.3125 per bit -> rounded to 0.3
	lm73_high = (lm73_temp >> 8); // get the higher bits of the temp
	
	itoa(lm73_high, MSByte, 10); //convert to string in array with itoa() from avr-libc
	itoa(lm73_low, LSByte, 10); //convert to string in array with itoa() from avr-libc

	memset(temp_digits, '\0', sizeof(temp_digits)); // Clear out the buffer again for reuse

	if(sign == 1) // incase it's negative temp
		strcat(temp_digits, "-");
	strcat(temp_digits, MSByte); // high
  	strcat(temp_digits, "."); // decimal
	if(lm73_low < 10)
		strcat(temp_digits, "0");
  	strcat(temp_digits, LSByte); // low 
}//lm73_temp_convert
