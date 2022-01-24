//todo: multimaster, block/unblock read/write slave, 9 bit, error statuses, unblock echo
#include "uart.h"

static uint8_t pre_sleep_state_ = 0; 

volatile bool *p_ready_rx_ =  NULL, *p_ready_tx_ =  NULL;
static char *p_data_tx_ = NULL, *p_data_rx_ = NULL;  
static size_t size_tx_ = 0,  size_rx_ = 0,    
              data_counter_tx_ = 0, data_counter_rx_ = 0; 


//---------------------------------------------upper half---------------------------------------------//

static status_t init_driver_(uart_handler *p_handle, bool do_init);
static status_t check_params_(uint8_t parity, uint8_t stop_bits, uint8_t data_bits, uint16_t baud_rate);
static void read_unblock_mode_data_(void);
static void read_block_mode_data_(char *p_data, size_t size);
static void write_unblock_mode_data_(void);
static void write_block_mode_data_(char *p_data, size_t size);

//---------------------------------------------lower half---------------------------------------------//

//static inline void set_interrupt_(/*interrupts*/int interpt, bool do_setting);
//static void set_double_speed_(bool do_double_speed,bool do_setting);
//static void set_mul_proc_(bool do_mul_proc,bool do_setting);
//static void set_synchronous_(bool do_synchronous, bool do_setting);
//static void set_stop_bits_(uint8_t stop_bits, bool do_setting);
//static void set_falling_edge_polarity_(bool do_falling_edge_polarity, bool do_setting);

//static void set_data_size_(uint8_t data_bits, bool do_setting);
static void set_parity_(uint8_t parity, bool do_setting);
static void set_baud_rate_(bool do_double_speed, bool do_synchronous, uint16_t baud_rate, bool do_setting);

static void set_sleep_(void);
static void set_waik_(void);

static void set_write_(bool do_setting);
static void set_read_(bool do_setting);

static uint8_t get_data_word_(void);
static void set_data_word_(char data);

#define M_set_interrupt(interpt, do_setting) do {                            \
    UCSRB &= (false << UDRIE) | (false << TXCIE) | (false << RXCIE);         \
                                                                             \
	if (true == (do_setting)) {                                              \
	    switch (interpt) {                                                   \
	    case uart_rx:                                                        \
	        UCSRB |= (true << RXCIE);                                        \
		break;                                                               \
		case uart_tx:                                                        \
		    UCSRB |= (true << UDRIE);                                        \
		break;                                                               \
		case uart_both:                                                      \
		    UCSRB |= (true << UDRIE)| (true << RXCIE);                       \
		break;                                                               \
		default:                                                             \
		    UCSRB &= ~((true << UDRIE) | (true << TXCIE) | (true << RXCIE)); \
        break;                                                               \
	    }                                                                    \
	}                                                                        \
} while (0)
 
#define M_set_double_speed(do_double_speed, do_setting) do {     \
    UCSRA |= (false << U2X);                                     \
                                                                 \
    if ((true == (do_double_speed)) && (true == (do_setting))) { \
		UCSRA |= (true << U2X);                                  \
    }                                                            \                 
} while (0)

#define M_set_mul_proc(do_mul_proc, do_setting) do {         \
    UCSRA |= (false << MPCM);                                \
                                                             \
    if((true == (do_mul_proc)) && (true == (do_setting) )) { \
		UCSRA |= (true << MPCM);                             \
	}                                                        \
} while (0)

#define M_set_synchronous(do_synchronous, do_setting) do {      \
    UCSRC |= (true << URSEL) | (false << UMSEL);                \
                                                                \
    if ((true == (do_setting)) && (true == (do_synchronous))) { \
		UCSRC |= (true << UMSEL);                               \
	}                                                           \
} while (0)

#define M_set_stop_bits(stop_bits, do_setting) do {      \   
    UCSRC |= (true << URSEL) | (false << USBS);          \
                                                         \
    if ((true == (do_setting)) &&  (1 == (stop_bits))) { \
		UCSRC |= (true << URSEL) | (true << USBS);       \
	}                                                    \
} while (0)

#define M_set_falling_edge_polarity(do_falling_edge_polarity, do_setting) do { \
    UCSRC |= (true << URSEL) | (false << UCPOL);                               \
                                                                               \
    if ((true == (do_setting)) && (true == (do_falling_edge_polarity))) {      \
		UCSRC |= (true << URSEL) | (true << UCPOL);                            \
	}                                                                          \
} while (0)

#define M_set_data_size(data_bits, do_setting) do {                         \
    UCSRC |= (true << URSEL) | (false << UCSZ1) | (false << UCSZ0);         \
    UCSRB |= (false << UCSZ2);                                              \
                                                                            \
    if (true == (do_setting)) {                                             \
		switch (data_bits) {                                                \
        case data_6:                                                        \
            UCSRC |= (true << URSEL) | (true << UCSZ0);                     \
        break;                                                              \
        case data_7:                                                        \
            UCSRC |= (true << URSEL) | (true << UCSZ1);                     \
        break;                                                              \
        case data_8:                                                        \
            UCSRC |= (true << URSEL) | (true << UCSZ1) | (true << UCSZ0);   \
        break;                                                              \
        case data_9:                                                        \
            UCSRC |= (true << URSEL) | (true << UCSZ1) | (true << UCSZ0);   \
            UCSRB |= (true << UCSZ2);                                       \
        break;                                                              \
        default:                                                            \
            UCSRC |= (true << URSEL) | (false << UCSZ1) | (false << UCSZ0); \
            UCSRB |= (false << UCSZ2);                                      \
        break;                                                              \
        }                                                                   \
	}                                                                       \
} while (0)

static void set_parity_(uint8_t parity, bool do_setting)
{
    UCSRC |= (true << URSEL) | (false << UPM1) | (false << UPM0);

    if(true == do_setting)
	{
		switch (parity)
        {
        case none:
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

static void set_baud_rate_(bool do_double_speed, bool do_synchronous, uint16_t baud_rate, bool do_setting)
{
	const uint8_t shift_value = 8;
	uint16_t baud_reg_val = 0;

	if ( true == do_synchronous)
	{
        baud_reg_val = (F_CPU / (16UL * baud_rate)) - 1;
	}
	else if (true == do_double_speed)
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

static void set_read_(bool do_setting)
{  
    UCSRB &= ~((true << RXEN) | (true << TXEN));
	if (do_setting)
    UCSRB |= (true << RXEN); 

}

static void set_write_(bool do_setting)
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

//---------------------------------------------upper half---------------------------------------------//

status_t uart_init(uart_handler *p_handle) {
	if (NULL == p_handle) {
		return error;
	}
	
   return init_driver_(p_handle, true);
}

static status_t init_driver_(uart_handler *p_handle, bool do_init) {
	p_handle->ready_rx = false;
	p_handle->ready_tx = false;
	p_handle->is_init = false;
	
	if (error == check_params_(p_handle->parity, p_handle->stop_bits, p_handle->data_bits, p_handle->baud_rate)) {
		return error;
	}

    M_set_double_speed(p_handle->do_double_speed,do_init);
    M_set_mul_proc(p_handle->do_mul_proc,do_init);
    M_set_synchronous(p_handle->do_synchronous,do_init);
    M_set_stop_bits(p_handle->stop_bits,do_init);
    M_set_falling_edge_polarity(p_handle->do_falling_edge_polarity,do_init);
	set_parity_(p_handle->parity, do_init);
    M_set_data_size(p_handle->data_bits, do_init);
    set_baud_rate_(p_handle->do_double_speed, p_handle->do_synchronous, p_handle->baud_rate, do_init);
	
	// for platform
	if ((true == p_handle->do_unblocking_mode) && (true == do_init)) {
        p_ready_rx_ = &p_handle->ready_rx;
		p_ready_tx_ = &p_handle->ready_tx;
	}
	else {
		p_ready_rx_ = NULL;
		p_ready_tx_ = NULL;
	}

	if (true == do_init) {
		p_handle->is_init = true;
	}
	
	p_handle->ready_rx = p_handle->is_init;
	p_handle->ready_tx = p_handle->is_init;
	
	return ok;
}

static status_t check_params_(uint8_t parity, uint8_t stop_bits, uint8_t data_bits, uint16_t baud_rate) {
	uart_parity_type par = odd;
	uart_stop_bits stops  = stop_2;
	
	uart_data_bits data_num_8 = data_8,
	               data_num_9 = data_9;
				   
    const uint16_t max_baud = 4800;
	
	if (parity > par) {
		return error;
	}
	
	if (stop_bits > stops) {
		return error;
	}
	
	if ((data_bits > data_num_8) && (data_bits < data_num_9)) {
		return error;
	}
	
	if (data_bits > data_num_9) {
		return error;
	}
	
	if (baud_rate > max_baud) {
		return error;
	}
	
    return ok;
}

status_t uart_release(uart_handler *p_handle) {
	interrupts inter = uart_both;
	
	if (NULL == p_handle) {
		return error;
	}
	
    if (false == p_handle->is_init) {
	    return error;
    }
	
	if (error == init_driver_(p_handle, false)) {
		return error;
	}
	
	memset(p_handle, 0, sizeof(uart_handler));

	M_set_interrupt(inter, false);

	return ok;
}

status_t uart_read_data(uart_handler *p_handle, void *p_data, size_t size) {
	if (NULL == p_handle) {
		return error;
	}
	
	if (false == p_handle->is_init) {
		return error;
	}

	if ((NULL == p_data) || (0 == size)) {
		return error;
	}
	
	p_handle->ready_rx = false;

	if (true == p_handle->do_unblocking_mode) {
		size_rx_ = size;
		p_data_rx_ = p_data;
		data_counter_rx_ = 0;
		read_unblock_mode_data_();
	}
	else {
		read_block_mode_data_(p_data, size);
		p_handle->ready_rx = true;
	}

	return 0;
}

static void read_unblock_mode_data_(void) {
   M_set_interrupt(uart_rx, true);
   set_read_(true);
}

static void read_block_mode_data_(char *p_data, size_t size) {
	uint32_t i = 0;

    set_read_(true);

	for (i = 0; i < size; i++) {
		p_data[i] = get_data_word_();
	}

    set_read_(false);
}

status_t uart_write_data(uart_handler *p_handle, char *p_data, size_t size) {
	if (NULL == p_handle) {
		return error;
	}
	
	if (true != p_handle->is_init) {
		return error;
	}

	if ((NULL == p_data) ||(0 == size) ) {
		return error;
	}
	
	p_handle->ready_tx = false;
	
	if (true == p_handle->do_unblocking_mode) {
		size_tx_ = size;
		p_data_tx_ = p_data;
		data_counter_tx_ = 0;
		write_unblock_mode_data_();
	}
	else {
		write_block_mode_data_(p_data, size);
		p_handle->ready_tx = true;
	}

	return 0;
}

static void write_unblock_mode_data_(void) {
	M_set_interrupt(uart_tx, true);
    set_write_(true);
}

static void write_block_mode_data_(char *p_data, size_t size) {
	uint32_t i = 0;

    set_write_(true);

	for(i = 0; i < size; i++) {
		set_data_word_(p_data[i]);
	}

    set_write_(false);
}

status_t uart_sleep(uart_handler *p_handle) {
	if (NULL == p_handle) {
		return error;
	}
	
    if (true != p_handle->is_init) {
		return error;
	}

    set_sleep_();

    return ok;
}

status_t uart_waik(uart_handler *p_handle) {
	if (NULL == p_handle) {
		return error;
	}
	
    if (true != p_handle->is_init)
	{
		return error;
	}

    set_waik_();

    return ok;
}

//---------------------------------------------lower half---------------------------------------------//



//---------------------------------------------interrupts------------------------------------------//

ISR(USART_UDRE_vect) {
    if(UCSRB & (true << TXEN)) {
	    UDR = p_data_tx_[data_counter_tx_++];

		if ( data_counter_tx_ == size_tx_ ) {
		    M_set_interrupt(uart_tx, false);
		    set_write_(false);
		    *p_ready_tx_ = true;
	    }
	}
}

ISR(USART_RXC_vect) {
    p_data_rx_[data_counter_rx_++] = UDR;

    if (data_counter_rx_ == size_rx_) {
	    M_set_interrupt(uart_rx, false);
	    set_read_(false);
 	    *p_ready_rx_ = true;
    }
}

