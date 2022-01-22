#ifndef INC_DS18B20_H
#define INC_DS18B20_H

#include "sleep.h"

#include "platform.h"

#include "ps_gpio.h"

typedef struct {
	_Bool is_device_here;

	uint8_t temperature;

	XStatus init;
    ps_gpio_handler gpio_handler;
} ds18b20_handler;

XStatus ds18b20_init(ds18b20_handler *p_handler);
XStatus ds18b20_write(ds18b20_handler *p_handler);
XStatus ds18b20_read(ds18b20_handler *p_handler);

#endif /* INC_DS18B20_H */
