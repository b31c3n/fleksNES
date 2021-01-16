/*
 * ram.c
 *
 *  Created on: Aug 21, 2020
 *      Author: David Jonsson
 */

#include "bus.h"
#include "ram.h"

uint8_t
    ram[0x7FF];


void ram_write()
{
    uint16_t
        address = cpu_bus.address_ & 0x07FF;
    ram[address] = cpu_bus.data_;
}

void ram_read()
{
    uint16_t
        address = cpu_bus.address_ & 0x07FF;
    cpu_bus.data_ = ram[address];
}

