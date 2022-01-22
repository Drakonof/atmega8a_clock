//todo: does need unblock mode?
#ifndef INC_PRINTF__H
#define INC_PRINTF__H

#include "uart.h"
#include "platform.h"

//#ifdef XPAR_XUARTPS_NUM_INSTANCES
#ifdef __cplusplus
extern "C" {
	#endif
	
typedef struct {
	uart_stop_bits    stop_bits;
	uart_parity_type  parity_type;
	uart_data_bits    data_bits;
	uint32_t          baud_rate;
	uint32_t          id;
} printf__inition;

status_t printf__custom_init(uart_handler init);
status_t printf__init(uint8_t id);
status_t putch_(char ch);
uint32_t printf_(const char *str,...);

//#endif
#ifdef __cplusplus
}
#endif

#endif /* INC_PRINTF__H */
