#include <unity.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

// hypothetical ADC reading with 5 V on input
#define ADC_5V 1610U
#define ROOM_TEMPERATURE_KELVIN 298U  // 298.15
#define NTC_RDIV 22000U
#define NTC_BETA 3977U
#define NTC_RNOM 10000U


uint16_t NTC_temperature_C10(uint16_t ADC_reading)
{
    printf("ADC_reading = %u\n", ADC_reading);
    uint32_t R_thermistor = (uint32_t)(NTC_RDIV) * ADC_reading / (ADC_5V - ADC_reading);
    if (R_thermistor == 0) return 0;  // This shouldn't happen
    printf("R_thermistor = %u\n", R_thermistor);

    // Floating point division takes about 200B of FLASH space,
    // got rid of it by using log(a/b) = log(a) - log(b)
    uint16_t temperature_kelvin =
        ((uint32_t)(NTC_BETA) * ROOM_TEMPERATURE_KELVIN * 10) /
        (uint32_t)(NTC_BETA + ROOM_TEMPERATURE_KELVIN * (logf(R_thermistor) - logf(NTC_RNOM)));
    printf("kelvin = %u\n", temperature_kelvin);

    if (temperature_kelvin < 273*10) return 0;  // negative temperatures are not supported
    return temperature_kelvin - 273*10;
}




void test_NTC()
{
    // measured: 20kOhm 2.424V
    TEST_ASSERT_EQUAL_UINT16(96, NTC_temperature_C10(2.424 / 5 * ADC_5V));
    // measured: 8kOhm 1.359V
    TEST_ASSERT_EQUAL_UINT16(295, NTC_temperature_C10(1.359 / 5 * ADC_5V));
    TEST_ASSERT_EQUAL_UINT16(0, NTC_temperature_C10(0));
}


int runUnityTests(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_NTC);


    return UNITY_END();
}


int main(void)
{
    return runUnityTests();
}
