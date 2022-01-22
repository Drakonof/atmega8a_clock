/*
 * atmega8a_clock.cpp
 *
 * Created: 19.11.2021 14:40:07
 * Author : Shimko Artyom (Drakonof)
 */ 
#define F_CPU 1000000

#include <avr/io.h>
#include <util/delay.h>

#include "ds1307.h"
#include "printf_.h"

int main(void) {
	//ds1307_handler ds1307_hnd = {};
	char d = 0x31;
	//size_t s = 1;
	
	printf__init(0);
	
    while (1) {
		
	  // printf_("%d:%d:%d\n", ds1307_hnd.hours, ds1307_hnd.minutes, ds1307_hnd.seconds);
	  printf_("%c", d);
	  _delay_ms(500);
    }
}

