//+
//todo: multimaster, block/unblock read/write slave, 9 bit, error statuses
#include "uart.h"

static uint8_t pre_sleep_state_ = 0; //

volatile bool *p_ready_ =  NULL;
static char *p_data_ = NULL;  
static size_t size_ = 0,     
              data_counter_ = 0; 


//---------------------------------------------upper half---------------------------------------------//

static status_t init_driver_(uart_handler *p_handle, bool do_init);
static status_t check_params_(uint8_t parity, uint8_t stop_bits, uint8_t data_bits, uint16_t baud_rate);
static void read_unblock_mode_data_(void);
static void read_block_mode_data_(char *p_data, size_t size);
static void write_unblock_mode_data_(void);
static void write_block_mode_data_(char *p_data, size_t size);

//---------------------------------------------lower half---------------------------------------------//

static inline void set_interrupt_(/*interrupts*/int interpt, bool do_setting);
static void set_double_speed_(bool do_double_speed,bool do_setting);
static void set_mul_proc_(bool do_mul_proc,bool do_setting);
static void set_synchronous_(bool do_synchronous, bool do_setting);
static void set_stop_bits_(uint8_t stop_bits, bool do_setting);
static void set_falling_edge_polarity_(bool do_falling_edge_polarity, bool do_setting);

static void set_data_size_(uint8_t data_bits, bool do_setting);
static void set_parity_(uint8_t parity, bool do_setting);
static void set_baud_rate_(bool do_double_speed, bool do_synchronous, uint16_t baud_rate, bool do_setting);

static void set_sleep_(void);
static void set_waik_(void);

static void set_write_(bool do_setting);
static void set_read_(bool do_setting);

static uint8_t get_data_word_(void);
static void set_data_word_(char data);

//---------------------------------------------upper half---------------------------------------------//

status_t uart_init(uart_handler *p_handle) {
   return init_driver_(p_handle, true);
}

static status_t init_driver_(uart_handler *p_handle, bool do_init) {
	p_handle->ready = false;
	p_handle->is_init = false;
	
	if (NULL == p_handle) {
		return error;
	}
	
	if (error == check_params_(p_handle->parity, p_handle->stop_bits, p_handle->data_bits, p_handle->baud_rate)) {
		return error;
	}

    set_double_speed_(p_handle->do_double_speed,do_init);
    set_mul_proc_(p_handle->do_mul_proc,do_init);
    set_synchronous_(p_handle->do_synchronous,do_init);
    set_stop_bits_(p_handle->stop_bits,do_init);
    set_falling_edge_polarity_(p_handle->do_falling_edge_polarity,do_init);
	set_parity_(p_handle->parity, do_init);
    set_data_size_(p_handle->data_bits, do_init);
    set_baud_rate_(p_handle->do_double_speed, p_handle->do_synchronous, p_handle->baud_rate, do_init);
	
	// for platform
	if ((true == p_handle->do_unblocking_mode) && (true == do_init)) {
        p_ready_ = &p_handle->ready;
	}
	else {
		p_ready_ = NULL;
	}

	if (true == do_init) {
		p_handle->is_init = true;
	}
	
	p_handle->ready = p_handle->is_init;
	
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

status_t uart_re_init(uart_handler *p_handle) {	
	if (false == p_handle->is_init) {
		return error;
	}
	
	return init_driver_(p_handle, true);
}

status_t uart_release(uart_handler *p_handle) {
	interrupts inter = uart_both;
	
    if (false == p_handle->is_init) {
	    return error;
    }
	
	if (error == init_driver_(p_handle, false)) {
		return error;
	}
	
	memset(p_handle, 0, sizeof(uart_handler));

	set_interrupt_(inter, false);

	return ok;
}

status_t uart_read_data(uart_handler *p_handle, void *p_data, size_t size) {
	if (false == p_handle->is_init) {
		return error;
	}

	if ((NULL == p_data) || (0 == size) || (NULL == p_handle)) {
		return error;
	}
	
	p_handle->ready = false;

	if (true == p_handle->do_unblocking_mode) {
		size_ = size;
		p_data_ = p_data;
		data_counter_ = 0;

		read_unblock_mode_data_();
	}
	else {
		read_block_mode_data_(p_data, size);
		p_handle->ready = true;
	}

	return 0;
}

static void read_unblock_mode_data_(void) {
   set_interrupt_(uart_rx, true);
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

status_t uart_get_ready(uart_handler *p_handle, bool *p_is_init) {
 	if ((true != p_handle->is_init) || (NULL == p_is_init) || (NULL == p_handle)) {
		return error;
 	}
	 
	*p_is_init = p_handle->is_init;

	return ok;
}

status_t uart_write_data(uart_handler *p_handle, char *p_data, size_t size) {
	if (true != p_handle->is_init) {
		return error;
	}

	if ((NULL == p_data) ||(0 == size) || (NULL == p_handle)) {
		return error;
	}
	
	p_handle->ready = false;
	
	if (true == p_handle->do_unblocking_mode) {
		size_ = size;
		p_data_ = p_data;
		data_counter_ = 0;
		write_unblock_mode_data_();
	}
	else {
		write_block_mode_data_(p_data, size);
		p_handle->ready = true;
	}

	return 0;
}

static void write_unblock_mode_data_(void) {
	set_interrupt_(uart_tx, true);
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


/*status_t uart_change_bool_param_(uart_bool_params bool_param, bool new_value)
{
    if (1 != init_)
	{
		return init_;
	}

    switch (bool_param)
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

status_t uart_change_data_size(uart_handler *p_handle, uart_data_bits new_data_size)
{
    if (true != p_handle->is_init)
	{
		return -1;
	}

    set_data_size_(new_data_size, true);

    return 0;
}

status_t uart_change_parity(uart_handler *p_handle, uart_parity_type new_parity)
{
    if (true != p_handle->is_init)
	{
		return -1;
	}

    set_parity_(new_parity, true);

    return 0;
}

status_t uart_change_baud_rate(uart_handler *p_handle)
{
    if (true != p_handle->is_init)
	{
		return -1;
	}

    set_baud_rate_(p_handle->do_double_speed, p_handle->do_synchronous, p_handle->baud_rate, true);

    return 0;
}

status_t uart_sleep(uart_handler *p_handle)
{
    if (true != p_handle->is_init)
	{
		return -1;
	}

    set_sleep_();

    return 0;
}

status_t uart_waik(uart_handler *p_handle)
{
    if (true != p_handle->is_init)
	{
		return -1;
	}

    set_waik_();

    return 0;
}

//---------------------------------------------lower half---------------------------------------------//

static inline void set_interrupt_(int interpt, bool do_setting)
{
    UCSRB &= (false << UDRIE) | (false << TXCIE) | (false << RXCIE);

	if (true == do_setting)
	{
	    switch (interpt)
	    {
	    case uart_rx:
	        UCSRB |= (true << RXCIE);
		break;
		case uart_tx:
		    UCSRB |= (true << UDRIE) | (true << TXCIE) ;
		break;
		case uart_both:
		    UCSRB |= (true << UDRIE) | (true << TXCIE) | (true << RXCIE);
		break;
		default:
		    UCSRB &= ~((true << UDRIE) | (true << TXCIE) | (true << RXCIE));
        break;
	    }
	}
}

static void set_double_speed_(bool do_double_speed, bool do_setting)
{
    UCSRA |= (false << U2X);

    if ((true == do_double_speed) && (true == do_setting))
	{
		UCSRA |= (true << U2X);
	}
}

static void set_mul_proc_(bool do_mul_proc, bool do_setting)
{
    UCSRA |= (false << MPCM);

    if((true == do_mul_proc) && (true == do_setting))
	{
		UCSRA |= (true << MPCM);
	}
}

static void set_synchronous_(bool do_synchronous, bool do_setting)
{
    UCSRC |= (true << URSEL) | (false << UMSEL);

    if ((true == do_setting) && (true == do_synchronous))
	{
		UCSRC |= (true << UMSEL);
	}
}

static void set_stop_bits_(uint8_t stop_bits, bool do_setting)
{
    UCSRC |= (true << URSEL) | (false << USBS);

    if ((true == do_setting) &&  (1 == stop_bits))
	{
		UCSRC |= (true << URSEL) | (true << USBS);
	}
}

static void set_falling_edge_polarity_(bool do_falling_edge_polarity, bool do_setting)
{
    UCSRC |= (true << URSEL) | (false << UCPOL);

    if ((true == do_setting) && (true == do_falling_edge_polarity))
	{
		UCSRC |= (true << URSEL) | (true << UCPOL);
	}
}

static void set_data_size_(uint8_t data_bits, bool do_setting)
{
    UCSRC |= (true << URSEL) | (false << UCSZ1) | (false << UCSZ0);
    UCSRB |= (false << UCSZ2);

    if (true == do_setting)
	{
		switch (data_bits)
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

//---------------------------------------------interrupts------------------------------------------//

ISR(USART_UDRE_vect) {
    if(UCSRB & (true << TXEN)) {
	    UDR = p_data_[data_counter_++];

		if ( data_counter_ == size_ ) {
		    set_interrupt_(uart_tx, false);
		    set_write_(false);
		    *p_ready_ = true;
	    }
	}
}

ISR(USART_RXC_vect) {
    p_data_[data_counter_++] = UDR;

    if (data_counter_ == size_) {
	    set_interrupt_(uart_rx, false);
	    set_read_(false);
 	    *p_ready_ = true;
    }
}

