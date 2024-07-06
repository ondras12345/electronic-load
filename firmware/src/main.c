#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <avr/pgmspace.h>
#include <math.h>

#include <lcd.h>
#include <gpio.h>
#include <serial.h>
#include "millis.h"

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

// 12-bit PWM (frequency vs resolution tradeoff)
#define PWM_TOP 0x0FFF

#define ADC_GNDPWR 0
#define ADC_ISENSE 1
#define ADC_VSENSE 2
#define ADC_TEMPERATURE 3
#define ADC_NCHANNELS 4

#define ADC_MAX 1023U
// hypothetical ADC reading with 5 V on input
#define ADC_5V 2047U
#define ROOM_TEMPERATURE_KELVIN 298U  // 298.15
#define NTC_RDIV 22000U
#define NTC_BETA 3977U
#define NTC_RNOM 10000U

// fan temperature thresholds, 'C*10
#define TEMPERATURE_LOW 280
#define TEMPERATURE_HIGH 320

const int8_t ENCODER_MATRIX[16] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};


volatile uint16_t ADC_values[ADC_NCHANNELS] = {0};
volatile bool ADC_complete = false;
volatile int8_t encoder_pulses = 0;

uint16_t voltage_V100 = 0;  // in volts, stored * 100
uint16_t current_mA = 0;
uint16_t temperature_C10 = 0;  // 'C * 10


typedef struct {
    float I_gain;
    float V_gain;
} settings_t;

settings_t settings = {
    // 2.5V ... 1023 ... 10A (roughly) ... 10000mA --> I_gain=9.77517106549365
    9.7752,  // I_gain, TODO test
    // 2.5V ... 1023 ... 85V ... 8500 V100 --> V_gain=8.3088954056696
    8.3089,  // V_gain, TODO test
};


// TODO store settings in EEPROM




void adc_init()
{
    // init ADC
    // use AREF
    ADMUX = 0x00;
    ADCSRA = (1<<ADEN)  // enable ADC
           | (1<<ADIE)  // ADC interrupt enable
           | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0)  // prescaler 128
           ;
    ADCSRA |= (1<<ADSC);  // start conversion
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


// TODO test NTC
uint16_t NTC_temperature_C10(uint16_t ADC_reading)
{
    uint32_t R_thermistor = (uint32_t)(NTC_RDIV) * ADC_reading / (ADC_5V - ADC_reading);
    if (R_thermistor == 0) return 0;  // This shouldn't happen

    // Floating point division takes about 200B of FLASH space,
    // got rid of it by using log(a/b) = log(a) - log(b)
    uint16_t temperature_kelvin =
        ((uint32_t)(NTC_BETA) * ROOM_TEMPERATURE_KELVIN * 10) /
        (uint32_t)(NTC_BETA + ROOM_TEMPERATURE_KELVIN * (logf(R_thermistor) - log(NTC_RNOM)));

    if (temperature_kelvin < 273*10) return 0;  // negative temperatures are not supported
    return temperature_kelvin - 273*10;
}


int main(void)
{
    gpio_conf(PIN_PWM, OUTPUT, 1);  // start with 0 current
    gpio_conf(PIN_FAN, OUTPUT, 1);  // start with fan on
    // inputs - nopullup is default
    //gpio_conf(PIN_ENCODER_AB, INPUT, NOPULLUP);
    //gpio_conf(PIN_ENCODER_SW, INPUT, NOPULLUP);

    pwm_init();
    millis_init();
    adc_init();
    serial_init9600();
    sei();
    lcd_init(LCD_DISP_ON);
    lcd_puts_p(PSTR("boot"));

    for (;;)
    {
        // set duty
        OCR1B = 0;  // TODO

        // control fan
        static bool fan = true;
        uint16_t fan_thres = fan ? TEMPERATURE_LOW : TEMPERATURE_HIGH;
        fan = (temperature_C10 > fan_thres || temperature_C10 == 0);
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

            const uint16_t V_max = (settings.V_gain == 0.0 || !isfinite(settings.V_gain)) ? 0 : 65535U / settings.V_gain;
            if (vals[ADC_VSENSE] >= V_max) voltage_V100 = -1;
            else voltage_V100 = vals[ADC_VSENSE] * settings.V_gain;

            const uint16_t I_max = (settings.I_gain == 0 || !isfinite(settings.I_gain)) ? 0 : 65535U / settings.I_gain;
            if (vals[ADC_ISENSE] >= I_max) current_mA = -1;
            else current_mA = vals[ADC_ISENSE] * settings.I_gain;

            temperature_C10 = NTC_temperature_C10(vals[ADC_TEMPERATURE]);
        }

        static millis_t prev_ms = 0;
        millis_t now = millis();
        if (now - prev_ms >= 500UL)
        {
            prev_ms = now;
            // TODO use SCPI instead
            serial_puts_P(PSTR("V: "));
            serial_putuint(voltage_V100);
            serial_puts_P(PSTR(" I: "));
            serial_putuint(current_mA);
            serial_puts_P(PSTR(" T: "));
            serial_putuint(temperature_C10);
            serial_puts_P(PSTR(" d: "));
            serial_putuint(OCR1B);
            serial_puts_P(PSTR("\r\n"));

            lcd_home();
            char buf[16];
            snprintf_P(buf, sizeof buf,
                    PSTR("%2u.%02u V"), voltage_V100 / 100, voltage_V100 % 100
            );
            lcd_puts(buf);

            lcd_invert(1);  // TODO test
            lcd_gotoxy(0, 1);
            snprintf_P(buf, sizeof buf,
                    PSTR("%2u.%03u A"), current_mA / 1000, current_mA % 1000
            );
            lcd_puts(buf);
            lcd_invert(0);

            lcd_gotoxy(0, 2);
            snprintf_P(buf, sizeof buf,
                    PSTR("%2u.%u 'C"), temperature_C10 / 10, temperature_C10 % 10
            );
            lcd_puts(buf);
        }
        // TODO use encoder
    }

    return 0;
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
    ADC_values[channel] = (H << 8) | L;


    // advance to next channel
    ADMUX = (t & 0xF0) | next_channel;
    // start new conversion
    // (this works out to one conversion per 104 us)
    ADCSRA |= (1<<ADSC);

    if (next_channel == 0) ADC_complete = true;
}


ISR(INT1_vect, ISR_ALIASOF(INT0_vect));

ISR(INT0_vect)
{
    static uint8_t state = 0;
    state = ((state << 2) | gpio_rd(PIN, PIN_ENCODER_AB)) & 0x0F;
    encoder_pulses += ENCODER_MATRIX[state];
}
