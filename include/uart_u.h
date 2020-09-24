#ifndef _UART_H_
#define _UART_H_

#include "Arduino.h"
#define freq_CPU 48000000    // CPU clock frequency

void uart_init();

void uart_putc(char c);

void uart_write(uint16_t data);

void uart_puts(char *s);

void uart_put_hex(uint8_t x);

#endif