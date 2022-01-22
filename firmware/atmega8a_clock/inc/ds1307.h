//todo: 1.stop/start functions 2. reinit 3. release
#ifndef INC_DS1307_H
#define INC_DS1307_H

#include "platform.h"

#include "i2c.h"

typedef enum {
    wr_ready_flag,
    rd_ready_flag
} ds1307_ready_flags;

typedef enum {
    rate_1Hz,
    rate_4KHz,
    rate_8KHz,
    rate_32KHz
} ds1307_square_wave_rate;

typedef enum {
    seconds,
    minutes,
    hours, // take both hours and am parameters
    day,
    date,
    month,
    year,
    control,
    all
} ds1307_param;

typedef struct {
    _Bool do_square_wave;
    ds1307_square_wave_rate square_rate;

    //for writing and reading regarding a data direction.
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    _Bool is_am;
    _Bool do_am;
    uint8_t day;
    uint8_t date;
    uint8_t month;
    uint8_t year;

    _Bool is_out_high; //-
	
    status_t init;
	
    i2c_handler i2c_hnd;
} ds1307_handler;

status_t ds1307_init(ds1307_handler *p_handler);
status_t ds1307_write(ds1307_handler *p_handler, ds1307_param param);
status_t ds1307_read(ds1307_handler *p_handler, ds1307_param param);
status_t ds1307_get_data(ds1307_handler *p_handler, ds1307_param param);
status_t ds1307_get_ready(ds1307_handler *p_handler, ds1307_ready_flags ready_flag, _Bool *ready) ;

#endif /* INC_DS1307_H */
