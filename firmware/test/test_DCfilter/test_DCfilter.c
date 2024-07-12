#include <unity.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#define FILTERSHIFT 8 // for low pass filters to determine ADC offsets
#define FILTERROUNDING (1<<(FILTERSHIFT-1))


uint16_t filter(uint16_t sample)
{
    static uint32_t foffset = 0;
    uint16_t offset = (foffset + FILTERROUNDING) >> FILTERSHIFT;
    foffset += (int16_t)sample - offset;
    return offset;
}


void test_DCfilter()
{
    FILE * fo = fopen("dcfilter.csv", "w");
    TEST_ASSERT_NOT_NULL(fo);
    fprintf(fo, "t\tu\ty\n");

    for (double t = 0.0; t < 10; t += 1e-3)
    {
        double v = 1 + sin(2*M_PI*50*t);
        int16_t off = (t < 5) ? 100 : -255;
        int16_t amplitude = (t < 5) ? 256 : 256;
        int16_t sample = v * amplitude + off;
        if (sample < 0) sample = 0;
        if (sample > 1023) sample = 1023;
        uint16_t filtered = filter(sample);

        fprintf(fo, "%f\t%u\t%u\n", t, sample, filtered);
    }
    fclose(fo);
}


int runUnityTests(void)
{
    if (chdir("test/test_DCfilter"))
    {
        fprintf(stderr, "failed to chdir");
        return 1;
    }

    UNITY_BEGIN();
    RUN_TEST(test_DCfilter);

    // gnuplot
    fprintf(stderr, "gnuplot result: %d\n", system("./dcfilter.gpi"));

    return UNITY_END();
}


int main(void)
{
    return runUnityTests();
}
