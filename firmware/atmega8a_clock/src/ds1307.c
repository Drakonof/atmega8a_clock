//todo: get wave out, RAM, right am/pm, порядок и оптимизация
#include <string.h>

#include "ds1307.h"

#define BUS_ADDRESS          0x68U

#define DATA_BUFFER_SIZE     9U
#define AM_PM_GET_BIT_OFFSET 5U
#define AM_PM_SET_BIT_OFFSET 6U
#define SQUARE_RATE_OFFSET   0U
#define SQUARE_WAVE_OFFSET   4U
#define HOURS_MASK           0x3FU
#define HOURS_MASK_AM        0x1FU

#define WR_CONVERT(x)        ((x / 10 ) << 4) | (x % 10 )
#define RD_CONVERT(x)        ((x  >> 4 ) * 10)  + (x  & 0x0F)

typedef enum {
    tx_address_buf,
    tx_data_buf,
    tx_seconds = 1,
    tx_minutes,
    tx_hours,
    tx_day,
    tx_date,
    tx_month,
    tx_year,
    tx_control,
    tx_all
} tx_array;

typedef enum {
    rx_data_buf,
    rx_seconds = 0,
    rx_minutes,
    rx_hours,
    rx_day,
    rx_date,
    rx_month,
    rx_year,
    rx_control,
    rx_all
} rx_array;

static uint8_t rx_buffer[DATA_BUFFER_SIZE],
               tx_buffer[DATA_BUFFER_SIZE];

status_t ds1307_init(ds1307_handler *p_handler) {
	p_handler->init = -1;

	if (NULL == p_handler) {
		return -1;
	}

    memset(rx_buffer, 0, DATA_BUFFER_SIZE);
    memset(tx_buffer, 0, DATA_BUFFER_SIZE);

//todo: platform relatedness
/*#if (2U == XPAR_XIICPS_NUM_INSTANCES)
    if ((XPAR_XIICPS_0_DEVICE_ID != i2c_id) || (XPAR_XIICPS_1_DEVICE_ID != i2c_id)) {
        return XST_FAILURE;
    }
#elif (1U == XPAR_XIICPS_NUM_INSTANCES)
    if (XPAR_XIICPS_0_DEVICE_ID !=  p_handler->i2c_hnd.id) {
        return XST_FAILURE;
    }
#endif*/

    p_handler->i2c_hnd.bus_address = BUS_ADDRESS;
    p_handler->i2c_hnd.rx_buffer = rx_buffer;
    p_handler->i2c_hnd.tx_buffer = tx_buffer;

    p_handler->init = 1;

    return 0;
}

status_t ds1307_write(ds1307_handler *p_handler, ds1307_param param) {
    const uint8_t first_wr_field = 0;

    if (NULL == p_handler) {
	    return -1;
    }

    if (0 == p_handler->init) {
		return -1;
	}

    if (all != param) {
    	p_handler->i2c_hnd.size = 1;
        tx_buffer[tx_address_buf] = param;
    }
    else {
    	p_handler->i2c_hnd.size = DATA_BUFFER_SIZE - 1;
        tx_buffer[tx_address_buf] = first_wr_field;
    }

    switch (param) {
    case seconds:
        tx_buffer[tx_data_buf] = WR_CONVERT(p_handler->seconds);
    break;
    case minutes:
        tx_buffer[tx_data_buf] = WR_CONVERT(p_handler->minutes);
    break;
    case hours:
        tx_buffer[tx_data_buf] = (WR_CONVERT(p_handler->hours) |
                              (p_handler->do_am << AM_PM_SET_BIT_OFFSET));
    break;
    case day:
        tx_buffer[tx_data_buf] = WR_CONVERT(p_handler->day);
    break;
    case date:
        tx_buffer[tx_data_buf] = WR_CONVERT(p_handler->date);
    break;
    case month:
        tx_buffer[tx_data_buf] = WR_CONVERT(p_handler->month);
    break;
    case year:
        tx_buffer[tx_data_buf] = WR_CONVERT(p_handler->year);
    break;
    case control:
        tx_buffer[tx_data_buf] = ((p_handler->do_square_wave << SQUARE_WAVE_OFFSET) |
                                 (p_handler->square_rate << SQUARE_RATE_OFFSET));
    break;
    case all:
        tx_buffer[tx_seconds] = WR_CONVERT(p_handler->seconds);
        tx_buffer[tx_minutes] = WR_CONVERT(p_handler->minutes);
        tx_buffer[tx_hours] = (WR_CONVERT(p_handler->hours) | (p_handler->do_am << AM_PM_SET_BIT_OFFSET));
        tx_buffer[tx_day] = WR_CONVERT(p_handler->day);
        tx_buffer[tx_date] = WR_CONVERT(p_handler->date);
        tx_buffer[tx_month] = WR_CONVERT(p_handler->month);
        tx_buffer[tx_year] = WR_CONVERT(p_handler->year);
        tx_buffer[tx_control] = ((p_handler->do_square_wave << SQUARE_WAVE_OFFSET) |
                             (p_handler->square_rate << SQUARE_RATE_OFFSET));
    break;
    default:
        return -1;
    break;
    }

    return i2c_write(&p_handler->i2c_hnd);
}

status_t ds1307_read(ds1307_handler *p_handler, ds1307_param param) {
	_Bool ready = false;
    const uint8_t first_wr_field = 0;
    status_t status = -1;

    if (NULL == p_handler) {
	    return -1;
    }

    if (0 == p_handler->init) {
	    return -1;
    }


    p_handler->i2c_hnd.size = 1;

    if (all != param) {
        rx_buffer[tx_address_buf] = param;
    }
    else {
        rx_buffer[tx_address_buf] = first_wr_field;
    }

    status = i2c_write(&p_handler->i2c_hnd);
	
	if (0 == status) {
		return -1;
	}
    

    //block
    while(true != ready) {
    	i2c_get_ready(&p_handler->i2c_hnd, true, &ready);
    }

    if (all == param) {
    	p_handler->i2c_hnd.size = DATA_BUFFER_SIZE - 1;
    }

    return i2c_read(&p_handler->i2c_hnd);
}

status_t ds1307_get_data(ds1307_handler *p_handler, ds1307_param param) {
	if (NULL == p_handler) {
		return -1;
	}

	if (0 == p_handler->init) {
		return -1;
	}

    switch (param) {
    case seconds:
        p_handler->seconds = RD_CONVERT(rx_buffer[rx_data_buf]);
    break;
    case minutes:
        p_handler->minutes = RD_CONVERT(rx_buffer[rx_data_buf]);
    break;
    case hours:
        p_handler->is_am = (_Bool)((rx_buffer[rx_data_buf] >> AM_PM_GET_BIT_OFFSET) & 0x1); //mask needed
        p_handler->hours = RD_CONVERT(rx_buffer[rx_data_buf]);
    break;
    case day:
        p_handler->day = RD_CONVERT(rx_buffer[rx_data_buf]);
    break;
    case date:
        p_handler->date = RD_CONVERT(rx_buffer[rx_data_buf]);
    break;
    case month:
        p_handler->month = RD_CONVERT(rx_buffer[rx_data_buf]);
    break;
    case year:
		p_handler->year = RD_CONVERT(rx_buffer[rx_data_buf]);
	break;
	case control:
        p_handler->do_square_wave = (_Bool) (rx_buffer[rx_data_buf] >> SQUARE_WAVE_OFFSET);
        p_handler->square_rate = (ds1307_square_wave_rate) (rx_buffer[rx_data_buf] << SQUARE_RATE_OFFSET);
    break;
    case all:
        p_handler->seconds = RD_CONVERT(rx_buffer[rx_seconds]);
        p_handler->minutes = RD_CONVERT(rx_buffer[rx_minutes]);
        p_handler->is_am = (_Bool) ((rx_buffer[rx_hours] >> AM_PM_GET_BIT_OFFSET) & 0x1);//mask needed
        p_handler->hours = RD_CONVERT(rx_buffer[rx_hours]);
        p_handler->day = RD_CONVERT(rx_buffer[rx_day]);
        p_handler->date = RD_CONVERT(rx_buffer[rx_date]);
        p_handler->month = RD_CONVERT(rx_buffer[rx_month]);
        p_handler->year = RD_CONVERT(rx_buffer[rx_year]);
        p_handler->do_square_wave = (_Bool) (rx_buffer[rx_control] >> SQUARE_WAVE_OFFSET);
        p_handler->square_rate = (ds1307_square_wave_rate) (rx_buffer[rx_control] << SQUARE_RATE_OFFSET);
    break;
    default:
        return -1;
    break;
    }

    return 0;
}

status_t ds1307_get_ready(ds1307_handler *p_handler, ds1307_ready_flags ready_flag, _Bool *ready) {
	status_t status = -1;

	if (NULL == p_handler) {
		return -1;
	}

	if (0 == p_handler->init) {
		return -1;
	}

	status = i2c_get_ready(&p_handler->i2c_hnd, (i2c_ready_flags) ready_flag, ready);

	if (0 == status) {
		return -1;
	}

    return 0;
}
