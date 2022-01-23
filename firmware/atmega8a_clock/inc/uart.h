#ifndef INC_UART_H
#define INC_UART_H

#    ifdef __cplusplus
    extern "C" {
#    endif
	
#include "platform.h"

typedef enum
{
	stop_1,
	stop_2,
} uart_stop_bits;

typedef enum 
{
	none,
	even = 2,
	odd
} uart_parity_type;

typedef enum
{
	data_5,
	data_6,
	data_7,
	data_8,
	data_9 = 7,
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
	rx,
	tx,
	both
} uart_path_reset;

typedef struct
{
	volatile bool     ready;
//	ps_uart_id           id;
	volatile bool      is_init;
	uart_stop_bits stop_bits;
	
	uart_parity_type  parity;
	uart_data_bits    data_bits;
	uart_channel_mode channel_mode;
	uint32_t             baud_rate;
	bool do_double_speed;
	bool do_falling_edge_polarity;
	bool do_mul_proc;
	bool do_unblocking_mode;
	bool do_synchronous;
} uart_handler;

status_t uart_init(uart_handler *p_handle);
status_t uart_re_init(uart_handler *p_handle);
status_t uart_release(uart_handler *p_handle);
status_t uart_read_data(uart_handler *p_handle, void *p_data, size_t size);
status_t uart_get_ready(uart_handler *p_handle, bool *p_is_init);
status_t uart_write_data(uart_handler *p_handle, char *p_data, size_t size);

status_t uart_sleep(uart_handler *p_handle);
status_t uart_waik(uart_handler *p_handle);

//status_t uart_change_bool_param(uart_bool_params _Bool_param, _Bool new_value);
status_t uart_change_data_size(uart_handler *p_handle, uart_data_bits new_data_size);
status_t uart_change_parity(uart_handler *p_handle, uart_parity_type new_parity);
status_t uart_change_baud_rate(uart_handler *p_handle);

#ifdef __cplusplus
}
#endif

#endif /* INC_UART_H */
