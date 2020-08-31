/*
 * helper_funcs.c
 *
 *  Created on: Jun 10, 2020
 *      Author: David Jonsson
 */

#include <stdio.h>
#include "helper_funcs.h"

void int_to_binstring(uint8_t integer, char *string)
{
    for(uint8_t
        stop = 8,
        i = 0,
        j = stop - 1;
        i < stop; ++i, --j)
//    for(uint8_t i = 0, j = 7; i < 8; ++i, --j)

    {
        string[j] = (uint8_t) pow(2,i) & integer ? '1' : '0';
    }
}

void int_to_hexstring(uint8_t integer, char *string)
{
    sprintf(string, "%02x", integer);
}

int strip_zeros(int mask, int number)
{
	int
		zeros = __builtin_ffs(mask) - 1;
	return number >> zeros;
}

int add_zeros(int mask, int number)
{
	int
		zeros = __builtin_ffs(mask) - 1;
	return number << zeros;
}
