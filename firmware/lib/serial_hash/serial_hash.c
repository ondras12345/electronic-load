#include "serial_hash.h"


// https://stackoverflow.com/questions/7666509/hash-function-for-string
// hash until end of string or first space (including said space)
uint16_t serial_hash(const char *str)
{
    uint16_t hash = 5381;
    uint8_t c;

    while ((c = *str++))
    {
        hash = ((hash << 5) + hash) + c; /* hash * 33 + c */
        if (c == ' ') break;
    }

    return hash;
}
