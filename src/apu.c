/*
 * apu.c
 *
 *  Created on: Jun 7, 2020
 *      Author: David Jonsson
 */

#include "peripheral.h"

uint8_t apu_mem[0x1F + 1];
struct peripheral cpu_peripheral_apu =
{
    .bus_ = &cpu_bus,
    .address_min_ = 0x4000,
    .address_max_ = 0x401F,
    .mirror_mask_ = 0x1F,
    .memory_ = &apu_mem,
    .update = generic_update,
    .read = generic_read,
    .write = generic_write
};
