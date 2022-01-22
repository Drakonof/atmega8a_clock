//todo: sreg, watch dog
#ifndef INC_PLATFORM_H
#define INC_PLATFORM_H

#ifndef F_CPU
#define F_CPU	1000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "types.h"

#define AVR_ATMEGA8A
#define PRINTF_UNBLOCK_MODE_ENABLE false

typedef enum
{
	calib_50_100_mhz,
	calib_75_150_mhz = 0x7F,
	calib_100_200_mhz = 0xFF
} in_osc_calib_value;

typedef enum
{
	global_interrupt,
	moving_interrupt_vec,
	enabling_sleep
} platform_boolean_params;

typedef struct
{
	_Bool do_global_interrupt;
	_Bool do_moving_interrupt_vec;
	_Bool do_enabling_sleep;
	in_osc_calib_value calib_value;
} platform_handle;

status_t platform_init(platform_handle *handle);
status_t platform_re_init(platform_handle *handle);
status_t platform_release(void);

status_t platform_change_boolean_param(platform_boolean_params boolean_param, _Bool new_value);
status_t platform_change_in_osc_calib_value(in_osc_calib_value new_calib_value);

#endif /* INC_PLATFORM_H */