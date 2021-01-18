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

struct bus
    cpu_bus =
    {
            .interval_ = 0x2000,
            .write =
            {
                    ram_write,
                    ppu_write,
                    apu_write,
            },
            .read =
            {
                    ram_read,
                    ppu_read,
                    apu_read,
            },
            .ticker = cpu_tick
    };
struct bus
    ppu_bus =
    {
            .interval_ = 0x1000,
            .write =
            {
                    0,
                    0,
                    ntable_write,
                    ntable_write,
            },
            .read =
            {
                    0,
                    0,
                    ntable_read,
                    ntable_read,
            },
            .ticker = ppu_tick
    };

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
