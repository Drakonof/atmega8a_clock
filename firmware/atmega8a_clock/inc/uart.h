#ifndef INC_UART_H
#define INC_UART_H

#    ifdef __cplusplus
    extern "C" {
#    endif
	
#include "platform.h"

typedef enum
{
	stop_1,
	stop_1d5,
	stop_2,
} uart_stop_bits;

typedef enum
{
	even,
	odd,
	space,
	mark,
	none_parity
} uart_parity_type;

typedef enum
{
	data_8,
	data_7 = 2,
	data_6
} uart_data_bits;

typedef enum
{
	normal,
	echo,
	local_loopback,
	remote_loopback
} uart_channel_mode;

typedef enum
{
	tx,
	rx,
	both
} uart_path_reset;

typedef struct
{
	volatile _Bool     ready;
//	ps_uart_id           id;
	volatile status_t      init;
	
	uart_stop_bits    stop_bits;
	uart_parity_type  parity_type;
	uart_data_bits    data_bits;
	uart_channel_mode channel_mode;
	uint32_t             baud_rate;
	_Bool do_unblocking_mode;
} uart_handler;

status_t uart_init(uart_handler *p_handle);
status_t uart_re_init(uart_handler *p_handle);
status_t uart_release(void);
status_t uart_read_data(void *p_data, size_t size);
_Bool uart_get_ready(status_t *p_init_status);
status_t uart_write_data(char *p_data, size_t size);//void

status_t uart_sleep(void);
status_t uart_waik(void);

//status_t uart_change_bool_param(uart_bool_params _Bool_param, _Bool new_value);
status_t uart_change_data_size(uart_data_bits new_data_size);
status_t uart_change_parity(uart_parity_type new_parity);
status_t uart_change_baud_rate(uint16_t new_baud_rate);// uint16_t??

#ifdef __cplusplus
}
#endif

#endif /* INC_UART_H */
