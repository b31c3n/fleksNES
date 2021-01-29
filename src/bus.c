/*
 * bus.c
 *
 *  Created on: May 12, 2020
 *      Author: David Jonsson
 */

#include "bus.h"
#include "instruction.h"
#include "cpu.h"
#include "ram.h"
#include "ppu.h"
#include "apu.h"
#include "mapper.h"
#include "nametable.h"

void bus_read(
        struct bus *bus,
        uint16_t address)
{
    bus->address_ = address;
    bus->read[(address) / bus->interval_]();
    bus->ticker();
}

void bus_write(
        struct bus *bus,
        uint16_t address)
{
    bus->address_ = address;
    bus->write[(address) / bus->interval_]();
    bus->ticker();
}
