
/*
 * Copyright (c) 2006-2009 by Roland Riegel <feedback@roland-riegel.de>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sfr_defs.h>
#include <avr/sleep.h>

#include "uart.h"

/* some mcus have multiple uarts */
#ifdef UDR0
#define UBRRH UBRR0H
#define UBRRL UBRR0L
#define UDR UDR0

#define UCSRA UCSR0A
#define UDRE UDRE0
#define RXC RXC0

#define UCSRB UCSR0B
#define RXEN RXEN0
#define TXEN TXEN0
#define RXCIE RXCIE0

#define UCSRC UCSR0C
#define URSEL 
#define UCSZ0 UCSZ00
#define UCSZ1 UCSZ01
#define UCSRC_SELECT 0
#else
#define UCSRC_SELECT (1 << URSEL)
#endif

#ifndef USART_RXC_vect
#if defined(UART0_RX_vect)
#define USART_RXC_vect UART0_RX_vect
#elif defined(UART_RX_vect)
#define USART_RXC_vect UART_RX_vect
#elif defined(USART0_RX_vect)
#define USART_RXC_vect USART0_RX_vect
#elif defined(USART_RX_vect)
#define USART_RXC_vect USART_RX_vect
#elif defined(USART0_RXC_vect)
#define USART_RXC_vect USART0_RXC_vect
#elif defined(USART_RXC_vect)
#define USART_RXC_vect USART_RXC_vect
#else
#error "Uart receive complete interrupt not defined!"
#endif
#endif

//#define BAUD 9600UL
#define BAUD 115200UL
//#define BAUD 19200UL
#define UBRRVAL (F_CPU/(BAUD*16)-1)
//#define USE_SLEEP 1
#define USE_SLEEP 0

/* This array holds the UBRR values to cooresponding baudrates in the
   UART_SPEED_T enum. */
const uint16_t PROGMEM uart_ubrr_table[UART_SPEED_MAX_ITEMS] = {832,416,207,104,52,34,16};

void uart_init(UART_SPEED_T uart_speed, uint8_t word_length, char *parity, uint8_t stop_bits)
{
	//Assume 16MHz
	uint16_t new_ubrr = pgm_read_word(&uart_ubrr_table[UART_SPEED_9600]);  //Default is 9600bps

    /********************************************
    * Calculating baud rate - OpenLog is using  *
    * the Asynchronous Double Speed Mode for    *
    * the USART. Therefore, the calculation for *
    * the UBRR (baud rate) is:                  *
    *      (Fosc/8*BAUD_RATE) - 1               *
    * Example 9600                              *
    *       (16MHz/8*9600) - 1 = 208.33333 - 1  *
    *                          = ~207           *
    * To add a new baud rate, calculate the     *
    * ubrr value. Then, add the baud rate to    *
    * the speed enum and the value to the table *
    * both located in uart.h                    *
    *********************************************/
    
    /* Bound check value before assigning.
       If value is valid, assign new ubrr value, if not leave as default */
    if( uart_speed >= UART_SPEED_2400 && uart_speed <= UART_SPEED_115200 )
    {
	    new_ubrr = pgm_read_word(&uart_ubrr_table[uart_speed]);
	}

    /* Doubling the UART transfer rate yields slightly more accurate UBRR 
       calculation */
    UCSR0A = (1<<U2X0);
    
    /* set baud rate register*/
    UBRR0L = new_ubrr;
    UBRR0H = new_ubrr >> 8;
    
	if(stop_bits == 2)
	{
		UCSR0C |= (1 << USBS0);
	}
	else
		// Default to 1 if invalid choice
		UCSR0C |= (0 << USBS0);
		
	/* set frame format: 8 bit, no parity, 1 bit */
    //UCSRC = UCSRC_SELECT | (1 << UCSZ1) | (1 << UCSZ0);
    /* enable serial receiver and transmitter */

	
#if !USE_SLEEP
    UCSRB = (1 << RXEN) | (1 << TXEN);
#else
    //UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);
    UCSRB = (1 << RXEN) | (1 << TXEN);
#endif

}

void uart_putc(uint8_t c)
{
    if(c == '\n')
        uart_putc('\r');

    /* wait until transmit buffer is empty */
    while(!(UCSRA & (1 << UDRE)));

    /* send next byte */
    UDR = c;
}

void uart_putc_hex(uint8_t b)
{
    /* upper nibble */
    if((b >> 4) < 0x0a)
        uart_putc((b >> 4) + '0');
    else
        uart_putc((b >> 4) - 0x0a + 'a');

    /* lower nibble */
    if((b & 0x0f) < 0x0a)
        uart_putc((b & 0x0f) + '0');
    else
        uart_putc((b & 0x0f) - 0x0a + 'a');
}

void uart_putw_hex(uint16_t w)
{
    uart_putc_hex((uint8_t) (w >> 8));
    uart_putc_hex((uint8_t) (w & 0xff));
}

void uart_putdw_hex(uint32_t dw)
{
    uart_putw_hex((uint16_t) (dw >> 16));
    uart_putw_hex((uint16_t) (dw & 0xffff));
}

void uart_putw_dec(uint16_t w)
{
    uint16_t num = 10000;
    uint8_t started = 0;

    while(num > 0)
    {
        uint8_t b = w / num;
        if(b > 0 || started || num == 1)
        {
            uart_putc('0' + b);
            started = 1;
        }
        w -= b * num;

        num /= 10;
    }
}

void uart_putdw_dec(uint32_t dw)
{
    uint32_t num = 1000000000;
    uint8_t started = 0;

    while(num > 0)
    {
        uint8_t b = dw / num;
        if(b > 0 || started || num == 1)
        {
            uart_putc('0' + b);
            started = 1;
        }
        dw -= b * num;

        num /= 10;
    }
}

void uart_puts(const char* str)
{
    while(*str)
        uart_putc(*str++);
}

void uart_puts_p(PGM_P str)
{
    while(1)
    {
        uint8_t b = pgm_read_byte_near(str++);
        if(!b)
            break;

        uart_putc(b);
    }
}

uint8_t uart_getc()
{
    /* wait until receive buffer is full */
#if USE_SLEEP
    uint8_t sreg = SREG;
    sei();

    while(!(UCSRA & (1 << RXC)))
        sleep_mode();

    SREG = sreg;
#else
    while(!(UCSRA & (1 << RXC)));
#endif

    uint8_t b = UDR;
    if(b == '\r')
        b = '\n';

    return b;
}

//EMPTY_INTERRUPT(USART_RXC_vect)
