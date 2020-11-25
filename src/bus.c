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
            .write =
            {
                    ram_write,
                    ppu_write,
                    apu_write,
                    mapper_write_cpu_side,
                    mapper_write_cpu_side,
                    mapper_write_cpu_side,
                    mapper_write_cpu_side,
                    mapper_write_cpu_side,
            },
            .read =
            {
                    ram_read,
                    ppu_read,
                    apu_read,
                    mapper_read_cpu_side,
                    mapper_read_cpu_side,
                    mapper_read_cpu_side,
                    mapper_read_cpu_side,
                    mapper_read_cpu_side,
            },
    };
struct bus
    ppu_bus =
    {
            .write =
            {
                    mapper_write_ppu_side,
                    ntable_write,
            },
            .read =
            {
                    mapper_read_ppu_side,
                    ntable_read,
            },
    };

void bus_read(
        struct bus *bus,
        uint16_t address)
{
    bus->address_ = address;
    bus->read[(address) / 0x2000]();
    if(bus == &cpu_bus)
        tick();
}

void bus_write(
        struct bus *bus,
        uint16_t address)
{
    bus->address_ = address;
    bus->write[(address) / 0x2000]();
    if(bus == &cpu_bus)
        tick();
}
