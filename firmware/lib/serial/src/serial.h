#pragma once
#include <stdint.h>
#include <avr/pgmspace.h>

#ifndef SERIAL_RX_BUFFER_SIZE
    #define SERIAL_RX_BUFFER_SIZE 64
#endif
#ifndef SERIAL_TX_BUFFER_SIZE
    #define SERIAL_TX_BUFFER_SIZE 64
#endif

#define SERIAL_NO_DATA 0xFF

#ifdef __cplusplus
extern "C" {
#endif

void serial_init9600();

void serial_putc(uint8_t data);
void serial_puts(const char *s);
void serial_putl(const char *s, size_t len);
void serial_puts_P(PGM_P string);
void serial_putint(int16_t number);
void serial_putuint(uint16_t number);

uint8_t serial_read();

uint8_t serial_available();

void serial_flush();

#ifdef __cplusplus
}
#endif
