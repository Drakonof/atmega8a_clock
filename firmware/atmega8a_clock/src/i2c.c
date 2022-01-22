//todo: multimaster/arbitrage, unblock read/write master, block/unblock read/write slave, twwc flag, math?
#include "i2c.h"

#define I2C_STATUS_MASK  0xF8U

static _Bool is_un_block_mode = false,
               is_maste_mode = false;
static volatile _Bool  ready_ = false;

static uint8_t init_ = 0;


//---------------------------------------------upper half---------------------------------------------//

static status_t init_driver_(i2c_handler handle, _Bool do_setting);
static void set_init_params_(i2c_handler handle, _Bool do_init);
static status_t read_un_block_mode_data (uint8_t *data, size_t size, uint8_t address);
static status_t read_block_mode_data (uint8_t *data, size_t size, uint8_t address);
static status_t write_un_block_mode_data (uint8_t *data, size_t size, uint8_t address);
static status_t write_block_mode_data (uint8_t *data, size_t size, uint8_t address);

//---------------------------------------------lower half---------------------------------------------//

static void set_bit_rate_(uint8_t bit_rate_value, uint8_t bit_rate_prescaler_value, _Bool do_setting);
static void set_self_address_(uint8_t self_address,_Bool do_setting);
static void set_interrupt_(_Bool do_setting);
static void set_enable_(_Bool do_setting);
static uint8_t set_start (void);
static uint8_t set_slave_address(uint8_t address);
static uint8_t get_data_word(uint8_t *i2c_status, _Bool is_not_last_word);
static void set_stop(void);
static uint8_t set_data_word(uint8_t data_word);

//---------------------------------------------upper half---------------------------------------------//

status_t i2c_init(i2c_handler handle)
{
	return init_driver_(handle, true);
}

static status_t init_driver_(i2c_handler handle, _Bool do_setting)
{
	ready_ = false;
	is_un_block_mode = false;
	is_maste_mode = false;

	set_interrupt_(false);

	if (true == do_setting)
	{
		is_maste_mode = handle.do_master_mode;

		if (false != handle.do_unblocking_mode)
		{
			is_un_block_mode = true;
			set_interrupt_(true);
		}
	}

	set_init_params_(handle, do_setting);
	set_enable_(do_setting); //??
	
	return 0;
}

static void set_init_params_(i2c_handler handle, _Bool do_setting)
{
	init_ = 0;

	set_bit_rate_(handle.bit_rate_value, handle.bit_rate_prescaler_value, false);

	if (false == handle.do_master_mode)
	{
		set_self_address_(handle.self_address, true);
	}

	if (true == do_setting)
	{
		set_bit_rate_(handle.bit_rate_value, handle.bit_rate_prescaler_value, true);

		if (false == handle.do_master_mode)
		{
			set_self_address_(handle.self_address, true);
		}

		init_ = 1;
	}
}

status_t i2c_re_init(i2c_handler handle)
{
	if (1 != init_)
	{
		return init_;
	}

	init_ = init_driver_(handle, true);

	return init_;
}

status_t i2c_release(void)
{
	i2c_handler handle = {};

	if (1 != init_)
	{
		return init_;
	}

	init_driver_(handle, false);

	init_ = 0;
	ready_ = false;

	return 0;
}

status_t i2c_read(i2c_handler *p_i2c_hnd)
{
	if (1 != init_)
	{
		return init_;
	}

	if((NULL == p_i2c_hnd->rx_buffer) || (0 == p_i2c_hnd->size))
	{
		return -1;
	}
	
	if(true == is_un_block_mode)
	{
		if (-1 == read_un_block_mode_data((uint8_t *) p_i2c_hnd->rx_buffer, p_i2c_hnd->size, p_i2c_hnd->bus_address))
		{
			return -1;
		}
	}
	else
	{
		if (-1 == read_block_mode_data((uint8_t *) p_i2c_hnd->rx_buffer, p_i2c_hnd->size, p_i2c_hnd->bus_address))
		{
			return -1;
		}
	}
	
	return 0;
}

//todo: all
static status_t read_un_block_mode_data (uint8_t *data, size_t size, uint8_t address)
{
	set_start();

	return 0;
}

static status_t read_block_mode_data (uint8_t *data, size_t size, uint8_t address)
{
	uint8_t i2c_status = 0;
	const uint8_t read_address_mask = 0x01;

	uint32_t i = 0;

	ready_ = false;

	i2c_status = set_start();

	if((I2C_START_STATUS != i2c_status) || 
	  (I2C_RE_START_STATUS != i2c_status))
	{
		return -1;
	}

	i2c_status = set_slave_address(address | read_address_mask);

	if(I2C_RD_SLAWE_ADDR_OK_STATUS != i2c_status)
	{
		return -1;
	}

	for(i = 0; i < (size - 1); i++)
	{
		data[i] = get_data_word(&i2c_status, true);

		if(I2C_DATA_RD_ACK_STATUS != i2c_status)
		{
			return -1;
		}
	}

	data[i] = get_data_word(&i2c_status, false);

	if(I2C_DATA_RD_NOT_ACK_STATUS != i2c_status)
	{
		return -1;
	}

	set_stop();
	ready_ = 1;

	return 0;
}

_Bool i2c_get_ready(i2c_handler *p_i2c_hnd,  i2c_ready_flags ready_flag, _Bool *ready)
{
	if (1 != init_)
	{
		*ready = init_;
		return false;
	}

	return ready_;
}

status_t i2c_write(i2c_handler *p_i2c_hnd)
{
	if (1 != init_)
	{
		return init_;
	}

	if((NULL == p_i2c_hnd->tx_buffer) || (0 == p_i2c_hnd->size))
	{
		return -1;
	}
	
	if(true == is_un_block_mode)
	{
		if (-1 == write_un_block_mode_data((uint8_t *) p_i2c_hnd->tx_buffer, p_i2c_hnd->size, p_i2c_hnd->bus_address))
		{
			return -1;
		}
	}
	else
	{
		if (-1 == write_block_mode_data((uint8_t *) p_i2c_hnd->tx_buffer, p_i2c_hnd->size, p_i2c_hnd->bus_address))
		{
			return -1;
		}
	}
	
	return 0;
}

static status_t write_un_block_mode_data(uint8_t *data, size_t size, uint8_t address)
{
	set_start();

	return 0;
}

static status_t write_block_mode_data (uint8_t *data, size_t size, uint8_t address)
{
	uint8_t i2c_status = 0;
	const uint8_t write_address_mask = 0xFE;

	uint32_t i = 0;

	ready_ = false;

	i2c_status = set_start();

	if((I2C_START_STATUS != i2c_status) ||
	(I2C_RE_START_STATUS != i2c_status))
	{
		return -1;
	}

	i2c_status = set_slave_address(address & write_address_mask);

	if(I2C_WR_SLAWE_ADDR_OK_STATUS != i2c_status)
	{
		return -1;
	}

	for(i = 0; i < (size - 1); i++)
	{
		i2c_status = set_data_word(data[i]);

		if(I2C_DATA_WR_ACK_STATUS != i2c_status)
		{
			return -1;
		}
	}

	set_stop();
	ready_ = 1;

	return 0;
}

status_t i2c_change_bit_rate(uint8_t new_bit_rate_value, uint8_t new_bit_rate_prescaler_value)
{
	if (1 != init_)
	{
		return init_;
	}

	set_bit_rate_(new_bit_rate_value, new_bit_rate_prescaler_value, true);

	return 0;
}

status_t i2c_change_self_address(uint8_t new_self_address)
{
	if (true == is_maste_mode)
	{
		return -1;
	}

	set_self_address_(new_self_address, true);

	return 0;
}

//---------------------------------------------lower half---------------------------------------------//

static void set_bit_rate_(uint8_t bit_rate_value, uint8_t bit_rate_prescaler_value, _Bool do_setting)
{
	TWBR = 0;
	TWSR = 0;

	if(true == do_setting)
	{
		TWBR = bit_rate_value;
		TWSR = bit_rate_prescaler_value;
	}
}

static void set_self_address_(uint8_t self_address, _Bool do_setting)
{
	TWAR = 0;

	if(true == do_setting)
	{
		TWAR = self_address;
	}
}

static void set_interrupt_(_Bool do_setting)
{
	TWCR &= ~(true << TWIE);

	if(true == do_setting)
	{
		TWCR |= (true << TWIE);
	}
}

static void set_enable_(_Bool do_setting)
{
	TWCR &= ~(true << TWEN);

	if(true == do_setting)
	{
		TWCR |= (true << TWEN);
	}
}

static uint8_t set_start (void)
{
	TWCR = (true << TWINT) | (true << TWSTA) | (true << TWEN);
	while(!(TWCR & (true << TWINT)));

	return (TWSR & I2C_STATUS_MASK);
}

static uint8_t set_slave_address(uint8_t address)
{
	TWDR = address;
	TWCR = (true << TWINT) | (true << TWEN);
	while(!(TWCR & (true << TWINT)));
	
	return (TWSR & I2C_STATUS_MASK);
}

static uint8_t get_data_word(uint8_t *i2c_status, _Bool is_not_last_word)
{
	TWCR = (true << TWINT) | (true << TWEN);

	if(true == is_not_last_word)
	{
		TWCR |= (true << TWEA);
	}

	while(!(TWCR & (true << TWINT)));

	*i2c_status = (TWSR & I2C_STATUS_MASK);
	
	return TWDR;
}

static void set_stop(void)
{
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

static uint8_t set_data_word(uint8_t data_word)
{
	TWDR = data_word;
	
	TWCR = (1 << TWINT) | (1 << TWEN);

	while(!(TWCR & (1 << TWINT)));

	return (TWSR & I2C_STATUS_MASK);
}
