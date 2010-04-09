
/*
 * Copyright (c) 2006-2009 by Roland Riegel <feedback@roland-riegel.de>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <avr/pgmspace.h>

#ifdef __cplusplus
extern "C"
{
#endif
typedef enum
{
    UART_SPEED_2400,
    UART_SPEED_4800,
    UART_SPEED_9600,
    UART_SPEED_19200,
    UART_SPEED_38400,
    UART_SPEED_57600,
    UART_SPEED_115200,
    UART_SPEED_MAX_ITEMS
} UART_SPEED_T;

void uart_init(UART_SPEED_T uart_speed, uint8_t word_length, char *parity, uint8_t stop_bits);

void uart_putc(uint8_t c);

void uart_putc_hex(uint8_t b);
void uart_putw_hex(uint16_t w);
void uart_putdw_hex(uint32_t dw);

void uart_putw_dec(uint16_t w);
void uart_putdw_dec(uint32_t dw);

void uart_puts(const char* str);
void uart_puts_p(PGM_P str);

uint8_t uart_getc(void);


#ifdef __cplusplus
}
#endif

#endif

