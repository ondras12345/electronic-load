#include "millis.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

static volatile millis_t mstimer = 0;


millis_t millis()
{
    millis_t ms;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        ms = mstimer;
    }
    return ms;
}


void millis_init(void)
{
    // prescaler 64
    TCCR0 = (1<<CS00) | (1<<CS01);
    // enable timer 0 overflow interrupt
    TIMSK |= (1<<TOIE0);
    TCNT0 = 0;
}


ISR(TIMER0_OVF_vect)
{
    mstimer++;
    // compensate - get interrupt every ms instead of every 1.024 ms
    // TODO is this correct?
    TCNT0 += 6;
}
