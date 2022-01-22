//todo: it works only at block mode now and just can read temperature.
#include "ds18b20.h"

#define SEARCH_ROM		        0xF0
#define READ_ROM				0x33
#define MATCH_ROM				0x55
#define SKIP_ROM				0xCC
#define ALARM_SEARCH			0xEC

#define CONVERT      			0x44
#define WRITE_SCRATCHPAD	    0x4E
#define READ_SCRATCHPAD		    0xBE
#define COPY_SCRATCHPAD		    0x48
#define RECALL			        0xB8
#define READ_POWER_SUPPLY		0xB4

#define RESOLUTION_9_BIT		0x00
#define RESOLUTION_10_BIT		0x01
#define RESOLUTION_11_BIT		0x10
#define RESOLUTION_12_BIT		0x11

static XStatus write_command(ps_gpio_handler *p_gpio_handler, uint8_t rom,
                             uint8_t function, _Bool *is_device_here);
static XStatus initialization(ps_gpio_handler *p_gpio_handler, _Bool *dev_is_here);
static XStatus write_byte(ps_gpio_handler *p_gpio_handler, uint8_t byte);
static XStatus write_bit(ps_gpio_handler *p_gpio_handler, _Bool bit);
static XStatus read_byte(ps_gpio_handler *p_gpio_handler, uint8_t *byte) ;
static XStatus read_bit(ps_gpio_handler *p_gpio_handler, _Bool *bit);

XStatus ds18b20_init(ds18b20_handler *p_handler) {
	p_handler->init = XST_FAILURE;

	M_check_pointer(p_handler);

	p_handler->init = XST_SUCCESS;

	return XST_SUCCESS;
}

XStatus ds18b20_write(ds18b20_handler *p_handler) {
	M_check_pointer(p_handler);
	M_check_status(p_handler->init);

	return XST_SUCCESS;
}

XStatus ds18b20_read(ds18b20_handler *p_handler) {
	_Bool complete = false;
	uint8_t ms_nible = 0, ls_nible = 0;
	const uint8_t shift_value = 4;
	XStatus status = XST_FAILURE;

	M_check_pointer(p_handler);

	M_check_status(p_handler->init);

	status = write_command(&p_handler->gpio_handler, SKIP_ROM, WRITE_SCRATCHPAD,
			               &p_handler->is_device_here);
	M_check_status(status);

	status = write_command(&p_handler->gpio_handler, SKIP_ROM, CONVERT,
                           &p_handler->is_device_here);
	M_check_status(status);

	while(true == complete ) {
		status = ps_gpio_read(&p_handler->gpio_handler, &complete);
		if (XST_SUCCESS != status) {
			return status;
		}
	}

	status = write_command(&p_handler->gpio_handler, SKIP_ROM, READ_SCRATCHPAD,
                           &p_handler->is_device_here);
	M_check_status(status);

	status = read_byte(&p_handler->gpio_handler, &ls_nible);
	M_check_status(status);

	status = read_byte(&p_handler->gpio_handler, &ms_nible);
	M_check_status(status);

	p_handler->temperature = (ms_nible << shift_value) | (ls_nible >> shift_value);

	return XST_SUCCESS;
}

static XStatus write_command(ps_gpio_handler *p_gpio_handler, uint8_t rom,
		                     uint8_t function, _Bool *is_device_here) {
	XStatus status = XST_FAILURE;

	M_check_pointer(p_gpio_handler);
	M_check_pointer(is_device_here);

	status = initialization(p_gpio_handler, is_device_here);
	M_check_status(status);

	status = write_byte(p_gpio_handler,rom);
	M_check_status(status);

	return write_byte(p_gpio_handler, function);
}

static XStatus initialization(ps_gpio_handler *p_gpio_handler, _Bool *dev_is_here) {
	XStatus status = XST_FAILURE;

	const uint16_t init_usec_val = 480,
			       wait_responce_usec_val = 100,
				   rest_response_usec_val = 380;

	M_check_pointer(p_gpio_handler);
	M_check_pointer(dev_is_here);

	status = ps_gpio_write(p_gpio_handler, false);
	M_check_status(status);
	usleep(init_usec_val);

	status = ps_gpio_sleep(p_gpio_handler);
	M_check_status(status);
    usleep(wait_responce_usec_val);

    status = ps_gpio_read(p_gpio_handler, dev_is_here);
    M_check_status(status);

	usleep(rest_response_usec_val);

	return XST_SUCCESS;
}

static XStatus write_byte(ps_gpio_handler *p_gpio_handler, uint8_t byte) {
	uint8_t i = 0;
	const uint8_t bit_in_byte_val = 8;
	XStatus status = XST_FAILURE;

	M_check_pointer(p_gpio_handler);

	for (i = 0; i < bit_in_byte_val; i++) {
	    status = write_bit(p_gpio_handler, byte & (true << i));
	    M_check_status(status);
	}

	return XST_SUCCESS;
}

static XStatus write_bit(ps_gpio_handler *p_gpio_handler, _Bool bit) {
	const uint16_t bit_tick_val = 1, rest_tick_val = 65;
	XStatus status = XST_FAILURE;

	M_check_pointer(p_gpio_handler);

	status = ps_gpio_write(p_gpio_handler, false);
	M_check_status(status);
	usleep(bit_tick_val);

	if(true == bit) {
		status = ps_gpio_sleep(p_gpio_handler);
		M_check_status(status);
	}
	else {
		status = ps_gpio_write(p_gpio_handler, false);
		M_check_status(status);
	}
	usleep(rest_tick_val);

	status = ps_gpio_sleep(p_gpio_handler);
	M_check_status(status);

	return XST_SUCCESS;
}

static XStatus read_byte(ps_gpio_handler *p_gpio_handler, uint8_t *byte) {
	_Bool bit = false;
	uint8_t i;
	const uint8_t bit_in_byte_val = 8;
	XStatus status = XST_FAILURE;

	M_check_pointer(p_gpio_handler);
	M_check_pointer(byte);

	*byte = 0;

	for(i = 0; i < bit_in_byte_val; i++) {
		status = read_bit(p_gpio_handler, &bit);
		M_check_status(status);

		*byte |= (bit << i);
	}

	return XST_SUCCESS;
}

static XStatus read_bit(ps_gpio_handler *p_gpio_handler, _Bool *bit) {
	const uint16_t bit_tick_val = 1, response_tick_val = 9, rest_tick_val = 55;
	XStatus status = XST_FAILURE;

	M_check_pointer(p_gpio_handler);
	M_check_pointer(bit);

	status = ps_gpio_write(p_gpio_handler, false);
	M_check_status(status);
	usleep(bit_tick_val);

	status = ps_gpio_sleep(p_gpio_handler);
	M_check_status(status);
	usleep(response_tick_val);

	status = ps_gpio_read(p_gpio_handler, bit);
	M_check_status(status);
	usleep(rest_tick_val);

	return XST_SUCCESS;
}
