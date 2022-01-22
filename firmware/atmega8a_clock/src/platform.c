#include "platform.h"

static status_t init_ = 0;

//---------------------------------------------upper half---------------------------------------------//

static status_t init_driver_(platform_handle *p_handle, _Bool do_init);

//---------------------------------------------lower half---------------------------------------------//

static void set_global_interrupt_(_Bool do_global_interrupt, _Bool do_setting);
static void set_moving_interrupt_vec(_Bool do_moving_interrupt_vec, _Bool do_setting);
static void set_in_osc_calib_value(in_osc_calib_value calib_value, _Bool do_setting);
static void set_enabling_sleep(_Bool do_enabling_sleep, _Bool do_setting);

//---------------------------------------------upper half---------------------------------------------//

status_t platform_init(platform_handle *p_handle) {
	return init_driver_(p_handle, true);
}

static status_t init_driver_(platform_handle *p_handle, _Bool do_init) {
	if (NULL == p_handle) {
		return -1;
	}

	set_global_interrupt_(p_handle->do_global_interrupt, do_init);
	set_moving_interrupt_vec(p_handle->do_moving_interrupt_vec, do_init);
	set_in_osc_calib_value(p_handle->calib_value, do_init);
	set_enabling_sleep(p_handle->do_enabling_sleep, do_init);

	return 0;
}

status_t platform_re_init(platform_handle *p_handle) {
	if (1 != init_) {
		return init_;
	}

	init_ = init_driver_(p_handle, true);

	return init_;
}

status_t platform_release(void) {
	platform_handle handle = {};

	if (1 != init_) {
		return init_;
	}

	init_driver_(&handle, false);

	init_ = 0;

	return 0;

}

status_t platform_change_boolean_param(platform_boolean_params boolean_param, _Bool new_value) {
	if (1 != init_) {
		return init_;
	}

	switch (boolean_param) {
	case global_interrupt:
		set_global_interrupt_(new_value, true); // true and 0
	break;
	case moving_interrupt_vec:
		set_moving_interrupt_vec(new_value, true); // true and 0
	break;
	case enabling_sleep:
	    set_enabling_sleep(new_value, true); // true and 0
	break;
	default:
		return -1;
	break;
	}

	return 0;
}

status_t platform_change_in_osc_calib_value(in_osc_calib_value new_calib_value) {
	if (1 != init_) {
		return init_;
	}

	set_in_osc_calib_value(new_calib_value, true);

	return 0;
}

//---------------------------------------------lower half---------------------------------------------//

static void set_global_interrupt_(_Bool do_global_interrupt, _Bool do_setting) {
	cli();

	if (true == do_setting) {
		 sei();
	}
}

static void set_moving_interrupt_vec(_Bool do_moving_interrupt_vec, _Bool do_setting) {
	GICR &= ~((true << IVSEL) | (true << IVCE));

	if (true == do_setting) {
		GICR |= (true << IVSEL) | (true << IVCE);
	}
}

static void set_in_osc_calib_value(in_osc_calib_value calib_value, _Bool do_setting) {
	OSCCAL = 0;

	if (true == do_setting) {
		OSCCAL = calib_value;
	}
}

static void set_enabling_sleep(_Bool do_enabling_sleep, _Bool do_setting) {
	MCUCR &= ~(true << SE);

	if (true == do_setting) {
		MCUCR |= (true << SE);
	}
}