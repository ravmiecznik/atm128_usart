/*
 * usart.h
 *
 *  Created on: Jul 28, 2017
 *      Author: rafal
 *
 *      This library supports atmega128
 */

#ifndef USART_H_
#define USART_H_

#include "cbuffer.h"
#include <avr/io.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "commands_pgm.h"
#include "timers_r.h"
#include <util/crc16.h>

#define _crc(crc, data) _crc_xmodem_update(crc, data)
#define USART1_ENABLE

#ifdef USART0_ENABLE
void usart0_bind_buffers(CircBuffer* rx_buffer, CircBuffer* tx_buffer);
#endif

#ifdef USART1_ENABLE
void usart1_bind_buffers(CircBuffer* rx_buffer, CircBuffer* tx_buffer);
#endif

uint16_t calc_crc(uint8_t *buffer, uint16_t siz);

enum usart_num{
	usart0,
	usart1,
};


//TODO: to use cbuffer methods test inheritance of cbuffer
class Usart{
private:
	volatile uint8_t* uart_status_register;
	volatile uint8_t* uart_control_register_B;
	volatile uint8_t* uart_control_register_C;
	volatile uint8_t* uart_data_register;
	volatile uint8_t* ubrrh_register;
	volatile uint8_t* ubrrl_register;
	uint8_t uart_enable_flags;
	uint8_t uart_async_8bit_noparity_1stopbit_flags;
	uint8_t udrie;
	uint8_t u2x;
public:
	CircBuffer rx_buffer;
	CircBuffer tx_buffer;
	//volatile uint32_t* available;
	Usart(usart_num u_num, uint32_t baud, Timer1& timer, uint32_t rx_buff_siz=100, uint32_t tx_buff_siz=100);
	Timer1& timer;
	void uart_init(uint32_t baud);
	void Putchar(char c);
	int fputc(int c, FILE* stream){
		char tmp = c;
		Putchar(tmp);
		return c;
	}
	void puts(uint32_t amount, uint8_t* data);
	void puts(uint8_t* data);
	void puts(uint8_t* data, char c){
		puts(data);
		Putchar(c);
	}
	void puts(char* data){
		puts((uint8_t*)data);
	};
	void puts(char* data, char c){
		puts((uint8_t*)data);
		Putchar(c);
	};
	void puts(const char* data){
		puts((uint8_t*)data);
	};
	void puts(const char* data, char endchar){
		puts((uint8_t*)data);
		Putchar(endchar);
	};
	void puts(int64_t value){
		char tmp[10];
		puts(ltoa(value, tmp, 10));
	}
	void puts(int64_t value, char endchar){
		char tmp[10];
		puts(ltoa(value, tmp, 10));
		Putchar(endchar);
	}
	void puts_p(const char*);
	void puts_p(const char* str, char c){
		puts_p(str);
		Putchar(c);
	}
	char get(){
		return rx_buffer.get();
	}
	void get(uint32_t amount, uint8_t* ext_buffer){
		rx_buffer.get(amount, ext_buffer);
	}
	uint16_t get_uint(){
		uint8_t int_in_array[2];
		rx_buffer.get(2, int_in_array);
		return *(uint16_t*)int_in_array;
	}
	char* gets(char* buffer, char str_end = '\x00'){
		//return rx_buffer.get_all(buffer);
		return rx_buffer.gets(buffer, str_end);
	}
	char* gets(char str_end = '\x00'){
		//return rx_buffer.get_all(buffer);
		return rx_buffer.gets(str_end);
	}
	void operator << (char* string){
		puts(string);
	}
	void operator << (const char* string){
		puts_p(string);
	}
	bool receive_data_amount(uint32_t amount, uint32_t timeout_ms, char* rdy_msg = (char*)"");
	void err(char* msg){
		puts_p(ERR);Putchar(':');puts(msg);Putchar('\n');
	}
	void err_p(const char* msg){
		puts_p(ERR);puts_p(msg);Putchar('\n');
	}
	void err(const char* msg){
		puts_p(ERR);puts(msg);Putchar('\n');
	}
	void dbg_p(const char* msg){
		puts_p(DBG);puts_p(msg);Putchar('\n');
	}
	void dbg(const char* msg){
		puts(DBG);puts(msg);Putchar('\n');
	}
	bool wait_for_message(char* msg, uint16_t t);
	uint32_t available(){
		return rx_buffer.available;
	}
};




#endif /* USART_H_ */
