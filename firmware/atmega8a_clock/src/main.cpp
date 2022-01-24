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
/*	
	uart_handler handle = {};
	handle.baud_rate = 4800;
	handle.data_bits = data_8;
	handle.parity = none;
	handle.stop_bits = stop_1;

	handle.do_unblocking_mode = false;
	uart_init(&handle);
*/	
	printf__init(0);
	
   sei();
   //printf_("%c%c%c", d, d, d);
  // printf_("%c%c%c", d, d, d);
  // printf_("%c%c%c", d, d, d);
	
    while (1) {
		
	  // printf_("%d:%d:%d\n", ds1307_hnd.hours, ds1307_hnd.minutes, ds1307_hnd.seconds);
      printf_("%c", d);
/*	 
	    if(handle.ready_rx)   
	        uart_read_data(&handle, &d, 1);
	
	    if(handle.ready_tx)  {
	        uart_write_data(&handle, &d, 1);
			_delay_ms(100);
		}
*/	   

		_delay_ms(500); 
    }
}

