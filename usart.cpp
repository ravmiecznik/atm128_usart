/*
 * usart.cpp
 *
 *  Created on: Jul 28, 2017
 *      Author: rafal
 *      This library supports atmega128
 *
 */

#include "usart.h"
#include <avr/interrupt.h>
#include "timers_r.h"
#include <avr/delay.h>
#include "prints.h"
#include <string.h>


//TODO: implement option to resize rx or tx buffer size on demand

Usart::Usart(usart_num usart, uint32_t baud, Timer1& timer, uint32_t rx_buff_siz, uint32_t tx_buff_siz):
				rx_buffer(rx_buff_siz), tx_buffer(tx_buff_siz), timer(timer){
#ifdef USART0_ENABLE
	if(usart==usart0){
		uart_status_register = &UCSR0A;
		uart_control_register_B = &UCSR0B;
		uart_control_register_C = &UCSR0C;
		uart_data_register = &UDR0;
		ubrrh_register = &UBRR0H;
		ubrrl_register = &UBRR0L;
		udrie = UDRIE0;
		u2x = U2X0;
		/* Enable USART receiver and transmitter and receive complete interrupt */
		uart_enable_flags = _BV(RXCIE0)|_BV(RXEN0)|_BV(TXEN0);
		uart_async_8bit_noparity_1stopbit_flags = _BV(UCSZ01) | _BV(UCSZ00);
		usart0_bind_buffers(&rx_buffer, &tx_buffer);
	}
#endif
	if(usart==usart1){
		uart_status_register = &UCSR1A;
		uart_control_register_B = &UCSR1B;
		uart_control_register_C = &UCSR1C;
		uart_data_register = &UDR1;
		ubrrh_register = &UBRR1H;
		ubrrl_register = &UBRR1L;
		udrie = UDRIE1;
		u2x = U2X1;
		/* Enable USART receiver and transmitter and receive complete interrupt */
		uart_enable_flags = _BV(RXCIE1)|_BV(RXEN1)|_BV(TXEN1);
		uart_async_8bit_noparity_1stopbit_flags = _BV(UCSZ11) | _BV(UCSZ10);
		usart1_bind_buffers(&rx_buffer, &tx_buffer);
	}
	uart_init(baud);

}

void Usart::Putchar(char c){
	while ( tx_buffer.available == tx_buffer.size ) {
		;/* wait for free space in buffer */
	}
	tx_buffer.put((uint8_t)c);
	/* enable UDRE interrupt */
	*uart_control_register_B |= _BV(udrie);
}

//void Usart::putchar(uint8_t c){
//	while ( tx_buffer.available == tx_buffer.size ) {
//		;/* wait for free space in buffer */
//	}
//	tx_buffer.put(c);
//	/* enable UDRE interrupt */
//	*uart_control_register_B |= _BV(udrie);
//}

void Usart::puts(uint32_t amount, uint8_t* buffer){
	while ( tx_buffer.available == tx_buffer.size ) {
		;/* wait for free space in buffer */
	}
	tx_buffer.put(amount, buffer);
	/* enable UDRE interrupt */
	*uart_control_register_B |= _BV(udrie);
}

void Usart::puts(uint8_t* buffer){
	while ( tx_buffer.available == tx_buffer.size ) {
		;/* wait for free space in buffer */
	}
	tx_buffer.puts(buffer);
	/* enable UDRE interrupt */
	*uart_control_register_B |= _BV(udrie);
	_delay_ms(1);
}

inline uint16_t calc_received_crc(uint8_t* buffer, uint16_t pos){
	return (buffer[pos] + (buffer[pos+1]<<8));
}


void Usart::puts_p(const char* data){
	char c;
	uint32_t i=0;
	while ( tx_buffer.available == tx_buffer.size ) {
		;/* wait for free space in tx buffer */
	}
	while(c=pgm_read_byte(data+(i++))){
		tx_buffer.put(c);
	}
	*uart_control_register_B |= _BV(udrie);
	_delay_ms(1);
}

void Usart::uart_init(uint32_t baudrate){
	baudrate = F_CPU/16/baudrate -1;
	if ( baudrate & 0x8000 ) {
		*uart_status_register = (1<<u2x);  //Enable 2x speed
		baudrate &= ~0x8000;
	}
	*ubrrh_register = (uint8_t)(baudrate>>8);
	*ubrrl_register = (uint8_t) baudrate;

	/* Enable USART receiver and transmitter and receive complete interrupt */
	*uart_control_register_B = uart_enable_flags;

	*uart_control_register_C = uart_async_8bit_noparity_1stopbit_flags;
}

bool Usart::wait_for_message(char* msg, uint16_t timeout){
	char tmp[100];
	uint32_t t0 = timer.tstamp_ms();
	rx_buffer.flush();
	while (true){
		if(rx_buffer.available and rx_buffer.is_in_buffer(msg)){
			return true;
			rx_buffer.flush();
		}
		else if(timer.tstamp_ms() - t0 > timeout){
			printf0("Timeout waiting for %s\n", msg);
			return false;
		}
		_delay_ms(10);
	}
	return true;
}

////////USART0//////////////////////////////////////////////////////////////////////////
#ifdef USART0_ENABLE
	#define UART0_STATUS   UCSR0A
	#define UART0_CONTROL  UCSR0B
	#define UART0_DATA     UDR0
	#define UART0_UDRIE    UDRIE0

CircBuffer *rx0_buffer;
CircBuffer *tx0_buffer;

void usart0_bind_buffers(CircBuffer* rx_buffer, CircBuffer* tx_buffer){
	rx0_buffer = rx_buffer;
	tx0_buffer = tx_buffer;
}

ISR(USART0_RX_vect)
/*************************************************************************
Function: UART Receive Complete interrupt
Purpose:  called when the UART has received a character
**************************************************************************/
{
    uint8_t data;
    uint8_t usr;
    //bool frame_error;

    /* read UART status register and UART data register */
    usr  = UART0_STATUS;
    //frame_error = usr & _BV(4);
    data = UART0_DATA;
    rx1_buffer->put(data);
}

ISR(USART0_UDRE_vect)
/*************************************************************************
Function: UART Data Register Empty interrupt
Purpose:  called when the UART is ready to transmit the next byte
**************************************************************************/
{
    if ( tx0_buffer->available){
    	UART0_DATA = tx0_buffer->get();
    } else {
        /* tx buffer empty, disable UDRE interrupt */
        UART0_CONTROL &= ~_BV(UART0_UDRIE);
    }
}

#endif

////////USART1//////////////////////////////////////////////////////////////////////////
#ifdef USART1_ENABLE
	#define UART1_STATUS   UCSR1A
	#define UART1_CONTROL  UCSR1B
	#define UART1_DATA     UDR1
	#define UART1_UDRIE    UDRIE1

CircBuffer *rx1_buffer;
CircBuffer *tx1_buffer;

void usart1_bind_buffers(CircBuffer* rx_buffer, CircBuffer* tx_buffer){
	rx1_buffer = rx_buffer;
	tx1_buffer = tx_buffer;
}

ISR(USART1_RX_vect)
/*************************************************************************
Function: UART Receive Complete interrupt
Purpose:  called when the UART has received a character
**************************************************************************/
{
    volatile uint8_t data;
    //uint8_t usr;
    //bool frame_error;

    /* read UART status register and UART data register */
    //usr  = UART1_STATUS;
    //frame_error = usr & _BV(4);
    data = UART1_DATA;
    rx1_buffer->put(data);
}

ISR(USART1_UDRE_vect)
/*************************************************************************
Function: UART Data Register Empty interrupt
Purpose:  called when the UART is ready to transmit the next byte
**************************************************************************/
{
    if ( tx1_buffer->available){
    	UART1_DATA = tx1_buffer->get();
    } else {
        /* tx buffer empty, disable UDRE interrupt */
        UART1_CONTROL &= ~_BV(UART1_UDRIE);
        tx1_buffer->flush();
    }
}

#endif

uint16_t calc_crc(uint8_t *buffer, uint16_t siz){
	/*
	 * Function calculates crc for provided array only for first "siz" bytes
	 */
	uint16_t crc = 0;
	for (uint16_t i = 0; i < siz; ++i) {
		crc = _crc(crc, buffer[i]);
	}
	return crc;
}




