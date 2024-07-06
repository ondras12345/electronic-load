#include "serial.h"
#include <avr/interrupt.h>
#include <stdlib.h>

#define RX_RING_BUFFER (SERIAL_RX_BUFFER_SIZE+1)
#define TX_RING_BUFFER (SERIAL_TX_BUFFER_SIZE+1)

static uint8_t rx_buffer[RX_RING_BUFFER];
static uint8_t rx_buffer_head = 0;
static volatile uint8_t rx_buffer_tail = 0;
// Why is rx_buffer_tail volatile? Shouldn't rx_buffer_head be voltaile instead?
// https://github.com/gnea/grbl/commit/a87f25773cfe551509f1f82c9bd997ae33c87aa5

static uint8_t tx_buffer[TX_RING_BUFFER];
static uint8_t tx_buffer_head = 0;
static volatile uint8_t tx_buffer_tail = 0;


void serial_init9600()
{
#define BAUD 9600
#include <util/setbaud.h>
    UBRRH = UBRRH_VALUE;
    UBRRL = UBRRL_VALUE;
#if USE_2X
    UCSRA |= (1 << U2X);
#else
    UCSRA &= ~(1 << U2X);
#endif

    // 8-bit, no parity, 1 stop bit (default)

    // enable rx, tx, and interrupt on complete reception of a byte
    UCSRB |= (1<<RXEN) | (1<<TXEN) | (1<<RXCIE);
}


/// Write a byte to the TX serial buffer.
void serial_putc(uint8_t data)
{
    // Calculate next head
    uint8_t next_head = tx_buffer_head + 1;
    if (next_head == TX_RING_BUFFER) next_head = 0;

    while (next_head == tx_buffer_tail);

    tx_buffer[tx_buffer_head] = data;
    tx_buffer_head = next_head;

    // Enable Data Register Empty Interrupt
    UCSRB |= (1<<UDRIE);
}

void serial_puts(const char *s)
{
    char c;
    while ((c = *s++)) serial_putc(c);
}


void serial_putl(const char *s, size_t len)
{
    while (len > 0)
    {
        serial_putc(*s++);
        len--;
    }
}


void serial_puts_P(PGM_P string)
{
    char c;
    while ((c = pgm_read_byte(string++))) serial_putc(c);
}

void serial_putint(int16_t number)
{
    char buff[sizeof("-32768")];
    itoa(number, buff, 10);
    serial_puts(buff);
}

void serial_putuint(uint16_t number)
{
    char buff[sizeof("65535")];
    utoa(number, buff, 10);
    serial_puts(buff);
}


// Data Register Empty Interrupt handler
ISR(USART_UDRE_vect)
{
    // tx_buffer_tail is volatile
    uint8_t tail = tx_buffer_tail;

    // Send a byte from the buffer
    UDR = tx_buffer[tail];

    // Update tail position
    tail++;
    if (tail == TX_RING_BUFFER) { tail = 0; }

    tx_buffer_tail = tail;

    // Turn of UDRE interrupt if this was the last character.
    if (tail == tx_buffer_head)
    {
        UCSRB &= ~(1 << UDRIE);
    }
}


/// Fetch the first byte in the serial read buffer.
uint8_t serial_read()
{
    uint8_t tail = rx_buffer_tail;
    if (rx_buffer_head == tail)
    {
        return SERIAL_NO_DATA;
    }
    else
    {
        uint8_t data = rx_buffer[tail];
        tail++;
        if (tail == RX_RING_BUFFER) tail = 0;
        rx_buffer_tail = tail;
        return data;
    }
}


ISR(USART_RXC_vect)
{
    uint8_t next_head = rx_buffer_head + 1;
    if (next_head == RX_RING_BUFFER) next_head = 0;

    if (next_head != rx_buffer_tail) {
        rx_buffer[rx_buffer_head] = UDR;
        rx_buffer_head = next_head;
    }
}


/// Return number of bytes available for read.
uint8_t serial_available()
{
    uint8_t rtail = rx_buffer_tail; // rx_buffer_tail is volatile
    if (rx_buffer_head >= rtail)
        return rx_buffer_head - rtail;
    return SERIAL_RX_BUFFER_SIZE - (rtail - rx_buffer_head);
}

void serial_flush()
{
    while (tx_buffer_head == tx_buffer_tail);
}
