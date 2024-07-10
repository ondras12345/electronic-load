#include <unity.h>
#include <stdio.h>
#include "serial_hash.h"


void test_serial_hash()
{
    // print hashes of commands used in firmware
    const char * cmds[] = {
        "ISET ",
        "IGAIN ",
        "VGAIN ",
        "SPGAIN ",
        "ISET?",
        "DUTY?",
        "I?",
        "V?",
        "P?",
        "TEMP?",
        "IGAIN?",
        "VGAIN?",
        "SPGAIN?",
        "*BOOTLOADER",
        "*SAV",
        "*RST",
        NULL,
    };

    for (const char ** cmd = cmds; *cmd != NULL; cmd++)
    {
        printf("\"%s\"\t%u\r\n", *cmd, serial_hash(*cmd));
    }
}


int runUnityTests(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_serial_hash);
    return UNITY_END();
}


int main(void)
{
    return runUnityTests();
}
