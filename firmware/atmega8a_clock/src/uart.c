//+
//todo: multimaster, block/unblock read/write slave, 9 bit, error statuses
#include "uart.h"

static _Bool is_un_block_mode_ = false, 
               is_synchronous = false,  
			   is_double_speed = false;

static volatile _Bool  ready_  = false;

static uint8_t pre_sleep_state_ = 0;
static uint8_t* p_data_ = NULL;

static status_t init_ = 0; 

static size_t size_ = 0, 
              data_counter_ = 0;

typedef enum
{
  u_rx,
  u_tx,
  u_both
} u_interrupts;

//---------------------------------------------upper half---------------------------------------------//

static status_t init_driver_(uart_handler *p_handle, _Bool do_init);
static status_t check_params_(_Bool check_synchronous, _Bool check_double_speed);
static void read_un_block_mode_data_(void);
static void read_block_mode_data_(void);
static void write_un_block_mode_data_(void);
static void write_block_mode_data_(void);

//---------------------------------------------lower half---------------------------------------------//

static inline void set_interrupt_(/*interrupts*/int interpt, _Bool do_setting);
static void set_double_speed_(_Bool do_double_speed,_Bool do_setting);
static void set_mul_proc_(_Bool do_mul_proc,_Bool do_setting);
static void set_synchronous_(_Bool do_synchronous, _Bool do_setting);
static void set_twice_stop_(_Bool do_twice_stop, _Bool do_setting);
static void set_falling_edge_polarity_(_Bool do_falling_edge_polarity, _Bool do_setting);

static void set_data_size_(int data_size, _Bool do_setting);
static void set_parity_(int parity, _Bool do_setting);
static void set_baud_rate_(uint16_t baud_rate, _Bool do_setting);

static void set_sleep_(void);
static void set_waik_(void);

static void set_write_(_Bool do_setting);
static void set_read_(_Bool do_setting);

static uint8_t get_data_word_(void);
static void set_data_word_(char data);

//---------------------------------------------upper half---------------------------------------------//

status_t uart_init(uart_handler *p_handle)
{
   return init_driver_(p_handle, true);
}

static status_t init_driver_(uart_handler *p_handle, _Bool do_init)
{
	ready_ = false;
	init_  = 0;
    is_un_block_mode_ = do_init & p_handle->do_unblocking_mode;

    if (NULL == p_handle)
	{
	    init_ = -1;
		return -1;
	}

	/*if (1 != check_params_(p_handle->do_synchronous, p_handle->do_double_speed))
	{
	    init_ = -1;
	    return -1;
	}*/

//	is_synchronous  = p_handle->do_synchronous;
//	is_double_speed = p_handle->do_double_speed;

    //set_double_speed_(p_handle->do_double_speed,do_init);
   // set_mul_proc_(p_handle->do_mul_proc,do_init);
   // set_synchronous_(p_handle->do_synchronous,do_init);
  //  set_twice_stop_(p_handle->do_twice_stop,do_init);
  //  set_falling_edge_polarity_(p_handle->do_falling_edge_polarity,do_init);
//	set_parity_(p_handle->parity, do_init);
    set_data_size_(p_handle->data_bits, do_init);
    set_baud_rate_(p_handle->baud_rate, do_init);

	if (true == do_init)
	{
		init_ = 1;
	}
	
	return 0;
}

static status_t check_params_(_Bool check_synchronous, _Bool check_double_speed)
{
    if (check_synchronous & check_double_speed)
	{
        return -1; 
	}

    return 0;
}

status_t uart_re_init(uart_handler *p_handle)
{
	if (0 != init_)
	{
		return init_;
	}

	return init_driver_(p_handle, true);
}

status_t uart_release(void)
{
	uart_handler handle = {};

	if (1 != init_)
	{
		return init_;
	}

	init_driver_(&handle, false);
	set_interrupt_(both, false);

	return 0;
}

status_t uart_read_data(void *p_data, size_t size)
{
    ready_ = false;

	p_data_ = NULL;
	size_ = 0;
	data_counter_ = 0;

	if (1 != init_)
	{
		return init_;
	}

	if((NULL == p_data) || (0 == size))
	{
		return -1;
	}

	p_data_ = p_data;
	size_ = size;

	if(true == is_un_block_mode_)
	{
		read_un_block_mode_data_();
	}
	else
	{
		 read_block_mode_data_();
		 ready_ = true;
	}

	return 0;
}

static void read_un_block_mode_data_(void)
{
   set_interrupt_(rx, true);
   set_read_(true);
}

static void read_block_mode_data_(void)
{
	uint32_t i = 0;

    set_read_(true);

	for(i = 0; i < size_; i++)
	{
		p_data_[i] = get_data_word_();
	}

    set_read_(false);
}

_Bool uart_get_ready(status_t *p_init_status)
{
 	if (1 != init_)
 	{
	    if (NULL != p_init_status)
		{
            *p_init_status = init_;
		}
 		
 		return false;
 	}

	return ready_;
}



status_t uart_write_data(char *p_data, size_t size)
{
    ready_ = false;

	p_data_ = NULL;
	size_ = 0;
	data_counter_ = 0;

	if (1 != init_)
	{
		return init_;
	}

	if((NULL == p_data) ||(0 == size))
	{
		return -1;
	}

	p_data_ = p_data;
	size_ = size;
	
	if(true == is_un_block_mode_)
	{
		write_un_block_mode_data_();
	}
	else
	{
		write_block_mode_data_();
		ready_ = true;
	}

	return 0;
}

static void write_un_block_mode_data_(void)
{
	set_interrupt_(tx, true);
    set_write_(true);
}

static void write_block_mode_data_(void)
{
	uint32_t i = 0;

    set_write_(true);

	for(i = 0; i < size_; i++)
	{
		set_data_word_(p_data_[i]);
	}

    set_write_(false);
}


/*status_t uart_change__Bool_param_(uart__Bool_params _Bool_param, _Bool new_value)
{
    if (1 != init_)
	{
		return init_;
	}

    switch (_Bool_param)
	{
	case double_speed:
		set_double_speed_(new_value,true);
	break;
	case mul_proc:
		set_mul_proc_(new_value,true);
	break;
	case synchronous:
		set_synchronous_(new_value,true);
	break;
    case twice_stop:
		set_twice_stop_(new_value,true);
	break;
    case falling_edge_polarity:
		set_falling_edge_polarity_(new_value,true);
	break;
	default:
		return -1;
	break;
	}

    return 0;
}*/

status_t uart_change_data_size(uart_data_bits new_data_size)
{
    if (1 != init_)
	{
		return init_;
	}

    set_data_size_(new_data_size, true);

    return 0;
}

status_t uart_change_parity(uart_parity_type new_parity)
{
    if (1 != init_)
	{
		return init_;
	}

    set_parity_(new_parity, true);

    return 0;
}

status_t uart_change_baud_rate(uint16_t new_baud_rate)
{
    if (1 != init_)
	{
		return init_;
	}

    set_baud_rate_(new_baud_rate, true);

    return 0;
}

status_t uart_sleep(void)
{
    if (1 != init_)
	{
		return init_;
	}

    set_sleep_();

    return 0;
}

status_t uart_waik(void)
{
    if (1 != init_)
	{
		return init_;
	}

    set_waik_();

    return 0;
}

//---------------------------------------------lower half---------------------------------------------//

static inline void set_interrupt_(int interpt, _Bool do_setting)
{
    UCSRB |= (false << UDRIE) | (false << TXCIE) | (false << RXCIE);

	if (true == do_setting)
	{
	    switch (interpt)
	    {
	    case rx:
	        UCSRB |= (true << RXCIE);
		break;
		case tx:
		    UCSRB |= (true << UDRIE);
		break;
		case both:
		    UCSRB |= (true << UDRIE) | (true << RXCIE);
		break;
		default:
		    UCSRB &= ~((true << UDRIE) | (true << TXCIE) | (true << RXCIE));
        break;
	    }
	}
}

static void set_double_speed_(_Bool do_double_speed, _Bool do_setting)
{
    UCSRA |= (false << U2X);

    if ((true == do_double_speed) && (true == do_setting))
	{
		UCSRA |= (true << U2X);
	}
}

static void set_mul_proc_(_Bool do_mul_proc, _Bool do_setting)
{
    UCSRA |= (false << MPCM);

    if((true == do_mul_proc) && (true == do_setting))
	{
		UCSRA |= (true << MPCM);
	}
}

static void set_synchronous_(_Bool do_synchronous, _Bool do_setting)
{
    UCSRC |= (true << URSEL) | (false << UMSEL);

    if ((true == do_setting) && (true == do_synchronous))
	{
		UCSRC |= (true << UMSEL);
	}
}

static void set_twice_stop_( _Bool do_twice_stop, _Bool do_setting)
{
    UCSRC |= (true << URSEL) | (false << USBS);

    if ((true == do_setting) &&  (true == do_twice_stop))
	{
		UCSRC |= (true << URSEL) | (true << USBS);
	}
}

static void set_falling_edge_polarity_(_Bool do_falling_edge_polarity, _Bool do_setting)
{
    UCSRC |= (true << URSEL) | (false << UCPOL);

    if ((true == do_setting) && (true == do_falling_edge_polarity))
	{
		UCSRC |= (true << URSEL) | (true << UCPOL);
	}
}

static void set_data_size_(int data_size, _Bool do_setting)
{
    UCSRC |= (true << URSEL) | (false << UCSZ1) | (false << UCSZ0);
    UCSRB |= (false << UCSZ2);

    if (true == do_setting)
	{
		switch (data_size)
        {
        case data_6:
            UCSRC |= (true << URSEL) | (true << UCSZ0);
        break;
        case data_7:
            UCSRC |= (true << URSEL) | (true << UCSZ1);
        break;
        case data_8:
            UCSRC |= (true << URSEL) | (true << UCSZ1) | (true << UCSZ0);
        break;
    /*    case data_size_9_bit:
            UCSRC |= (true << URSEL) | (true << UCSZ1) | (true << UCSZ0);
            UCSRB |= (true << UCSZ2);
        break;*/
        default:
            UCSRC |= (true << URSEL) | (false << UCSZ1) | (false << UCSZ0);
            UCSRB |= (false << UCSZ2);
        break;
        }
	}
}

static void set_parity_(int parity, _Bool do_setting)
{
    UCSRC |= (true << URSEL) | (false << UPM1) | (false << UPM0);

    if(true == do_setting)
	{
		switch (parity)
        {
        case none_parity:
            UCSRC |= (true << URSEL) |(false << UPM1) | (false << UPM0);
        break;
        case even:
            UCSRC |= (true << URSEL) | (true << UPM1);
        break;
        case odd:
            UCSRC |= (true << URSEL) | (true << UPM1) | (true << UPM0);
        break;
        default:
            UCSRC |= (true << URSEL) | (false << UPM1) | (false << UPM0);
        break;
        }
	}
}

static void set_baud_rate_(uint16_t baud_rate, _Bool do_setting)
{
	const uint8_t shift_value = 8;
	uint16_t baud_reg_val = 0;

	if ( true == is_synchronous)
	{
        baud_reg_val = (F_CPU / (16UL * baud_rate)) - 1;
	}
	else if (true == is_double_speed)
	{
	    baud_reg_val = (F_CPU / (16UL * baud_rate)) - 1;
	}
	else
	{
	    baud_reg_val = (F_CPU / (16UL * baud_rate)) - 1;
	}

	UBRRH = (false << URSEL) | ((uint8_t)(baud_reg_val >> shift_value));
	UBRRL = (uint8_t)baud_reg_val;
} 

static void set_sleep_(void)
{
    pre_sleep_state_ = UCSRB;

    UCSRB = 0;
}


static void set_waik_(void)
{
    UCSRB = pre_sleep_state_;
}

static void set_read_(_Bool do_setting)
{  
    UCSRB &= ~((true << RXEN) | (true << TXEN));
	if (do_setting)
    UCSRB |= (true << RXEN); 

}

static void set_write_(_Bool do_setting)
{
	UCSRB &= ~((true << RXEN) | (true << TXEN));

	if (do_setting)
	UCSRB |= (true << TXEN);
}

static uint8_t get_data_word_(void)
{
    while ( !(UCSRA & (true << RXC)));
    return UDR;
}

static void set_data_word_(char data)
{
    while ( !(UCSRA & (1 << UDRE)));
    UDR = data;
}

//---------------------------------------------interrupts------------------------------------------//

ISR(USART_UDRE_vect)
{
    if(UCSRB & (true << TXEN))
	{
	    UDR = p_data_[data_counter_++];

		if ( data_counter_ == size_ )
	    {
		    set_interrupt_(tx, false);
		    set_write_(false);
		    ready_ = true;
	    }
	}
}

ISR(USART_RXC_vect)
{
    p_data_[data_counter_++] = UDR;

    if ( data_counter_ == size_ )
    {
	    set_interrupt_(rx, false);
	    set_read_(false);
 	    ready_ = true;
    }
}

