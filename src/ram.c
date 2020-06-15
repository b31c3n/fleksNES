/*
 * ram.c
 *
 *  Created on: Jun 7, 2020
 *      Author: David Jonsson
 */

#include "peripheral.h"

uint8_t ram_mem[0x7FF + 1];
struct peripheral cpu_peripheral_ram =
{
    .bus_ = &cpu_bus,
    .address_min_ = 0,
    .address_max_ = 0x1FFF,
    .mirror_mask_ = 0x7FF,
    .memory_ = &ram_mem,
    .update = generic_update,
    .read = generic_read,
    .write = generic_write
};
