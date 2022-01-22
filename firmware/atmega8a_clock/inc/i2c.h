#ifndef INC_I2C_H
#define INC_I2C_H

#include "platform.h"

#define I2C_START_STATUS				0x08U
#define I2C_RE_START_STATUS				0x10U
#define I2C_ARBITRATION_LOST_STATUS		0x38U

#define I2C_WR_SLAWE_ADDR_OK_STATUS		0x18U
#define I2C_WR_SLAWE_ADDR_ER_STATUS		0x20U
#define I2C_DATA_WR_ACK_STATUS			0x28U
#define I2C_DATA_WR_NOT_ACK_STATUS		0x30U

#define I2C_RD_SLAWE_ADDR_OK_STATUS		0x40U
#define I2C_RD_SLAWE_ADDR_ER_STATUS		0x48U
#define I2C_DATA_RD_ACK_STATUS			0x50U
#define I2C_DATA_RD_NOT_ACK_STATUS		0x58U

typedef enum {
	wr_ready_fl,
	rd_ready_fl
} i2c_ready_flags;


typedef struct
{
	uint8_t self_address; // common acknowledge bit, multi master mode
	uint8_t bus_address;
	uint8_t bit_rate_value;
	uint8_t bit_rate_prescaler_value;
	uint8_t *rx_buffer;
	uint8_t *tx_buffer;
    size_t size;
	_Bool do_unblocking_mode;
	_Bool do_master_mode;
} i2c_handler;

status_t i2c_init(i2c_handler handle);
status_t i2c_re_init(i2c_handler handle);
status_t i2c_release(void);
status_t i2c_read(i2c_handler *p_i2c_hnd);
_Bool i2c_get_ready(i2c_handler *p_i2c_hnd,  i2c_ready_flags ready_flag, _Bool *ready);
status_t i2c_write(i2c_handler *p_i2c_hnd);

status_t i2c_change_bit_rate(uint8_t new_bit_rate_value, uint8_t new_bit_rate_prescaler_value);
status_t i2c_change_self_address(uint8_t new_self_address);

#endif 