/*
 * nametable.c
 *
 *  Created on: Aug 31, 2020
 *      Author: archie
 */

#include "mapper.h"
#include "nametable.h"
#include "bus.h"

uint8_t
    **ntable_mem = 0,
    *pal_mem = 0;

