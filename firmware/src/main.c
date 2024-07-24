#include <stdbool.h>
#include <ctype.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <math.h>

#include <lcd.h>
#include <gpio.h>
#include <serial.h>
#include "millis.h"
#include "serial_hash.h"

/*
PB1 .. ENC_SW
PB2 .. OC1B
PB3 .. FAN
PC0 .. ADC0 .. GNDPWR
PC1 .. ADC1 .. ISENSE
PC2 .. ADC2 .. VSENSE
PC3 .. ADC3 .. temperature
PC4 .. SDA
PC5 .. SCL

PD0 .. RX
PD1 .. TX
PD2 .. ENC_A
PD3 .. ENC_B
*/

#define PIN_ENCODER_SW  B, 1, 1 // PB1
#define PIN_PWM         B, 2, 2  // PB2
#define PIN_FAN         B, 3, 3  // PB3
#define PIN_ENCODER_AB  D, 3, 2  // PD3, PD2

// debounce times in ms
#define DEBOUNCE_ENCODER_SW 100U
// number of pulses per step
#define ENCODER_STEP 4

#define DISP_N_LINES 5

#define SETPOINT_MAX 8000U

// 12-bit PWM (frequency vs resolution tradeoff)
#define PWM_TOP 0x0FFF

#define ADC_GNDPWR 0
#define ADC_ISENSE 1
#define ADC_VSENSE 2
#define ADC_TEMPERATURE 3
#define ADC_NCHANNELS 4

// hypothetical ADC reading with 5 V on input
// AREF is 2.5V + diode drop (0.68V)
#define ADC_5V 1610U
#define ROOM_TEMPERATURE_KELVIN 298U  // 298.15
#define NTC_RDIV 22000U
#define NTC_BETA 3977U
#define NTC_RNOM 10000U

// fan temperature thresholds, 'C*10
#define TEMPERATURE_LOW 280
#define TEMPERATURE_HIGH 320

const int8_t ENCODER_MATRIX[16] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};

const uint16_t pow10_lut[] = {1, 10, 100, 1000, 10000};


volatile uint16_t ADC_values[ADC_NCHANNELS] = {0};
volatile bool ADC_complete = false;
volatile int8_t encoder_pulses = 0;
volatile bool encoder_pressed = false;
volatile bool encoder_fell = false;

uint16_t voltage_V100 = 0;  // in volts, stored * 100
uint16_t current_mA = 0;
uint16_t temperature_C10 = 0;  // 'C * 10
uint16_t power_W100 = 0;
uint8_t setpoint_digit = 2;  // which digit of setpoint is being edited
uint16_t setpoint_mA = 0;


typedef struct {
    // 2.5V ... 1023 ... 10A (roughly) ... 10000mA --> I_gain=9.77517106549365 -> 9775 ; seems to want 12775
    uint16_t I_gain1000;  // stored *1000
    // 2.5V ... 1023 ... 85V ... 8500 V100 --> V_gain=8.3088954056696 -> 8308 ; seems to want 10531
    uint16_t V_gain1000;  // stored *1000
    // full scale current in mA -> 10000 ; 10342
    uint16_t setpoint_gain;
    // offset added to OCR1B, ideally 0 ; 11
    uint8_t setpoint_offset;
} settings_t;

settings_t settings = { 0 };


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


void adc_init()
{
    // init ADC
    // use AREF
    ADMUX = 0x00;
    ADCSRA = (1<<ADEN)  // enable ADC
           | (1<<ADIE)  // ADC interrupt enable
           | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0)  // prescaler 128
           ;
    // first conversion will be started by TIMER0
}


void pwm_init()
{
    // set up timer1 for pwm on OC1B
    OCR1A = PWM_TOP;
    OCR1B = 0;  // duty
    // fast PWM mode, OC1B enabled inverting
    TCCR1A = (1<<COM1B1) | (1<<COM1B0) | (1<<WGM10) | (1<<WGM11);
    // clk/1
    TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS10);
    // PWM frequency is 16e6 / (PWM_TOP+1)
}


uint16_t NTC_temperature_C10(uint16_t ADC_reading)
{
    uint32_t R_thermistor = (uint32_t)(NTC_RDIV) * ADC_reading / (ADC_5V - ADC_reading);
    if (R_thermistor == 0) return 0;  // This shouldn't happen

    // Floating point division takes about 200B of FLASH space,
    // got rid of it by using log(a/b) = log(a) - log(b)
    uint16_t temperature_kelvin =
        ((uint32_t)(NTC_BETA) * ROOM_TEMPERATURE_KELVIN * 10) /
        (uint32_t)(NTC_BETA + ROOM_TEMPERATURE_KELVIN * (logf(R_thermistor) - logf(NTC_RNOM)));

    if (temperature_kelvin < 273*10) return 0;  // negative temperatures are not supported
    return temperature_kelvin - 273*10;
}


void format_current_mA(char * buf, size_t s)
{
    snprintf_P(buf, s,
        PSTR("%2u.%03u A"), current_mA / 1000, current_mA % 1000
    );
}


void format_voltage_V100(char * buf, size_t s)
{
    snprintf_P(buf, s,
            PSTR("%2u.%02u V"), voltage_V100 / 100, voltage_V100 % 100
    );
}


void format_power_W100(char * buf, size_t s)
{
    snprintf_P(buf, s,
            PSTR("%2u.%02u W"), power_W100 / 100, power_W100 % 100
    );
}


void serial_nl()
{
    serial_puts_P(PSTR("\r\n"));
}


void serial_parser()
{
    bool complete = false;
    static char buf[50 + sizeof '\0'];
    static uint8_t wi = 0;

    while (serial_available() && !complete)
    {
        char c = serial_read();
        buf[wi] = c;
        if (c == '\r' || c == '\n')
        {
            if (wi != 0)
            {
                buf[wi] = '\0';  // overwrite that newline
                complete = true;
                wi = 0;
            }
        }
        else
        {
            if (wi < sizeof(buf)-1)
            {
                wi++;
            }
            else
            {
                serial_puts_P(PSTR("E: too long\r\n"));
                wi = 0;
            }
        }
    }

    if (!complete) return;
    const char * number_str;
    for (number_str = buf; *number_str && !isdigit(*number_str); number_str++);

    // list of hashes corresponding to commands can be generated by make test
    long v = atol(number_str);  // temp variable for atol, used by some cases
    if (v < 0) v = 0;
    switch (serial_hash(buf))
    {
        case 45274:  // "ISET "
            // ISET 1000 : set current to 1A
            setpoint_mA = v;
            if (setpoint_mA > SETPOINT_MAX) setpoint_mA = SETPOINT_MAX;
            break;

        case 19405:  // "IGAIN "
            settings.I_gain1000 = v;
            break;

        case 23546:  // "VGAIN "
            settings.V_gain1000 = v;
            break;

        case 45927:  // "SPGAIN "
            settings.setpoint_gain = v;
            break;

        case 24099:  // "SPOFF "
            settings.setpoint_offset = v;
            break;

        case 45305:  // "ISET?"
            snprintf_P(buf, sizeof buf,
                    PSTR("%u.%03u A\r\n"), setpoint_mA / 1000, setpoint_mA % 1000
            );
            serial_puts(buf);
            break;

        case 36778:  // "DUTY?"
            serial_putuint(OCR1B);
            serial_nl();
            break;

        case 29677:  // "I?"
            format_current_mA(buf, sizeof buf);
            serial_puts(buf);
            serial_nl();
            break;

        case 30106:  // "V?"
            format_voltage_V100(buf, sizeof buf);
            serial_puts(buf);
            serial_nl();
            break;

        case 29908:  // "P?"
            format_power_W100(buf, sizeof buf);
            serial_puts(buf);
            serial_nl();
            break;

        case 12986:  // "TEMP?"
            snprintf_P(buf, sizeof buf,
                    PSTR("%u.%u degC\r\n"), temperature_C10 / 10, temperature_C10 % 10
            );
            serial_puts(buf);
            break;

        case 19436:  // "IGAIN?"
            serial_putuint(settings.I_gain1000);
            serial_nl();
            break;

        case 23577:  // "VGAIN?"
            serial_putuint(settings.V_gain1000);
            serial_nl();
            break;

        case 45958:  // "SPGAIN?"
            serial_putuint(settings.setpoint_gain);
            serial_nl();
            break;

        case 24130:  // "SPOFF?"
            serial_putuint(settings.setpoint_offset);
            serial_nl();
            break;

        case 56794:  // "*BOOTLOADER"
        {
            OCR1B = 0;  // duty
            gpio_set(PIN_FAN);

            _delay_ms(200);

            // https://arduino.stackexchange.com/questions/77226/jumping-to-bootloader-from-application-code-in-atmega328p
            // we are using Optiboot 8
            typedef void (*bootloader_jump_t)();
            const bootloader_jump_t bootloader_jump = (bootloader_jump_t)((FLASHEND-511)>>1);
            bootloader_jump();
        }
            break;

        case 33081:  // "*SAV"
            // save settings to EEPROM
            eeprom_update_block(&settings, 0, sizeof settings);
            break;

        case 32584:  // "*RST"
            OCR1B = 0;  // duty
            wdt_enable(WDTO_15MS);
            for (;;);
            break;

        default:
            serial_puts_P(PSTR("E: invalid cmd\r\n"));
            break;
    }
}


/// return true if refresh is done
bool disp_handler(void)
{
    static uint8_t line = 0;

    lcd_gotoxy(0, line);
    char buf[16];

    switch (line)
    {
        case 0:
            format_voltage_V100(buf, sizeof buf);
            break;
        case 1:
            format_current_mA(buf, sizeof buf);
            break;
        case 2:
            format_power_W100(buf, sizeof buf);
            break;
        case 3:
            snprintf_P(buf, sizeof buf,
                    PSTR("%2u.%u 'C"), temperature_C10 / 10, temperature_C10 % 10
            );
            break;
        case 4:
            snprintf_P(buf, sizeof buf,
                    PSTR("SET: %u.%03u A"), setpoint_mA / 1000, setpoint_mA % 1000
            );
            break;
        case 5:
        {
            // show cursor position
            //                 "SET: 1.235 A"
            strcpy_P(buf, PSTR("          "));
            uint8_t cpos = 9 - setpoint_digit;
            // handle decimal point
            if (cpos <= 6) cpos--;
            buf[cpos] = '^';
        }
            break;

        default:
            // this should never happen
            buf[0] = 0;
            break;
    }

    lcd_puts(buf);


    line = (line + 1) % (DISP_N_LINES+1);
    return line == 0;  // true if done
}


int main(void)
{
    gpio_conf(PIN_PWM, OUTPUT, 1);  // start with 0 current
    gpio_conf(PIN_FAN, OUTPUT, 1);  // start with fan on
    // inputs - nopullup is default
    //gpio_conf(PIN_ENCODER_AB, INPUT, NOPULLUP);
    //gpio_conf(PIN_ENCODER_SW, INPUT, NOPULLUP);

    // enable INT0, INT1 for encoder
    MCUCR |= (1<<ISC00) | (1<<ISC10);  // any logical change
    GICR |= (1<<INT0) | (1<<INT1);  // enable

    pwm_init();
    millis_init();
    adc_init();
    serial_init9600();
    sei();
    // read settings from EEPROM
    eeprom_read_block(&settings, 0, sizeof settings);
    wdt_enable(WDTO_500MS);
    wdt_reset();
    lcd_init(LCD_DISP_ON);
    lcd_puts_p(PSTR("boot"));

    for (;;)
    {
        serial_parser();

        // set duty
        uint16_t d = (uint32_t)(setpoint_mA) * PWM_TOP / settings.setpoint_gain + settings.setpoint_offset;
        OCR1B = (d < PWM_TOP) ? d : PWM_TOP;

        // control fan
        static bool fan = true;
        uint16_t fan_thres = fan ? TEMPERATURE_LOW : TEMPERATURE_HIGH;
        // if NTC is disconnected, it might report low temperature (or 0 on error)
        fan = (temperature_C10 > fan_thres || temperature_C10 < 10);
        if (fan) gpio_set(PIN_FAN);
        else gpio_clr(PIN_FAN);

        if (ADC_complete)
        {
            uint16_t vals[ADC_NCHANNELS];
            ATOMIC_BLOCK(ATOMIC_FORCEON)
            {
                for (uint8_t i = 0; i < ADC_NCHANNELS; i++)
                    vals[i] = ADC_values[i];
                ADC_complete = false;
            }
            // remove GNDPWR offset
            uint16_t off = vals[ADC_GNDPWR];
#define rmoff(val) val = (off > val) ? 0 : (val-off)
            rmoff(vals[ADC_VSENSE]);
            rmoff(vals[ADC_ISENSE]);
#undef rmoff

            const uint16_t V_max = (settings.V_gain1000 == 0) ? 0 : 65535000UL / settings.V_gain1000;
            if (vals[ADC_VSENSE] >= V_max) voltage_V100 = -1;
            else voltage_V100 = (uint32_t)(vals[ADC_VSENSE]) * settings.V_gain1000 / 1000U;

            const uint16_t I_max = (settings.I_gain1000 == 0) ? 0 : 65535000UL / settings.I_gain1000;
            if (vals[ADC_ISENSE] >= I_max) current_mA = -1;
            else current_mA = (uint32_t)(vals[ADC_ISENSE]) * settings.I_gain1000 / 1000U;

            power_W100 = ((uint32_t)(voltage_V100) * current_mA) / 1000U;

            temperature_C10 = NTC_temperature_C10(vals[ADC_TEMPERATURE]);
        }

        millis_t now = millis();
        bool force_refresh = false;
        // handle encoder
        if (encoder_fell)
        {
            encoder_fell = false;
            setpoint_digit++;
            if (setpoint_digit > 3) setpoint_digit = 0;
            force_refresh = true;
        }

        int8_t steps = 0;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            while (encoder_pulses >= ENCODER_STEP)
            {
                encoder_pulses -= ENCODER_STEP;
                steps++;
            }
            while (encoder_pulses <= -ENCODER_STEP)
            {
                encoder_pulses += ENCODER_STEP;
                steps--;
            }
        }
        static millis_t enc_prev_ms = 0;
        if (now - enc_prev_ms >= 1000U)
        {
            enc_prev_ms = now;
            // do not get stuck at number smaller than ENCODER_STEP
            encoder_pulses = 0;
        }
        if (setpoint_digit >= sizeof(pow10_lut)/sizeof(pow10_lut[0]))
            setpoint_digit = 0;
        if (steps != 0)
        {
            force_refresh = true;
            enc_prev_ms = now;
        }
        // quickly disable load by turning left while holding button
        if (encoder_pressed && steps < 0) setpoint_mA = 0;
        uint16_t prev = setpoint_mA;
        setpoint_mA += steps * pow10_lut[setpoint_digit];
        if (steps < 0 && setpoint_mA > prev) setpoint_mA = 0;
        if (setpoint_mA > SETPOINT_MAX) setpoint_mA = SETPOINT_MAX;

        static millis_t disp_prev_ms = 0;
        static bool disp_in_progress = false;
        if (now - disp_prev_ms >= 250U || force_refresh || disp_in_progress)
        {
            disp_prev_ms = now;
            disp_in_progress = !disp_handler();
        }

        wdt_reset();
    }

    return 0;
}


ISR(TIMER0_OVF_vect)
{
    mstimer++;
    // compensate - get interrupt every ms instead of every 1.024 ms
    // TODO is this correct?
    TCNT0 += 6;

    // encoder button debounce
    static uint8_t integrator = 0;
    if (gpio_tst(PIN_ENCODER_SW))
    {
        if (integrator < DEBOUNCE_ENCODER_SW) integrator++;
    }
    else
    {
        if (integrator > 0) integrator--;
    }
    bool prev = encoder_pressed;
    if (integrator == 0) encoder_pressed = true;
    if (integrator >= DEBOUNCE_ENCODER_SW) encoder_pressed = false;
    if (encoder_pressed && !prev) encoder_fell = true;

    // ignoring ADC_complete, we just hope loop() managed to read the values in
    // time

    // Start new conversion
    // at channel 0 (set either by ADC_init or ISR(ADC_vect)
    ADCSRA |= (1<<ADSC);
}


ISR(ADC_vect)
{
    uint8_t t = ADMUX;
    // warning: this does not use ADC_NCHANNELS
    uint8_t channel = t & 0x03;
    uint8_t next_channel = ((channel+1) & 0x03);

    // ADCL must be read first
    uint8_t L = ADCL;
    uint8_t H = ADCH;
    uint16_t sample = (H << 8) | L;

    // apply lowpass filter
    // https://docs.openenergymonitor.org/electricity-monitoring/ctac/digital-filters-for-offset-removal.html
    // this determines lowpass filter time constant, see test_DCfilter
    #define FILTERSHIFT 8
    #define FILTERROUNDING (1<<(FILTERSHIFT-1))
    static uint32_t foffset[ADC_NCHANNELS] = { 0 };
    int16_t offset = (foffset[channel] + FILTERROUNDING) >> FILTERSHIFT;
    foffset[channel] += (int16_t)sample - offset;
    ADC_values[channel] = offset;

    // advance to next channel
    ADMUX = (t & 0xF0) | next_channel;

    if (next_channel == 0) ADC_complete = true;
    // start new conversion
    else ADCSRA |= (1<<ADSC);
}


ISR(INT1_vect, ISR_ALIASOF(INT0_vect));

ISR(INT0_vect)
{
    static uint8_t state = 0x03;
    state = ((state << 2) | gpio_rd(PIN, PIN_ENCODER_AB)) & 0x0F;
    encoder_pulses += ENCODER_MATRIX[state];
}
