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

// Order to allow compatiability with old configs
#define BAUD_2400	0
#define BAUD_9600	1
#define BAUD_57600	2
#define BAUD_115200	3
#define BAUD_4800	4
#define BAUD_14400	5
#define BAUD_19200	6
#define BAUD_28800	7
#define BAUD_38400	8
#define BAUD_76800	9
#define BAUD_NONE	10

void uart_init(uint8_t uart_speed);

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

