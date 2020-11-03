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
                    ram_write,
                    ntable_write,
            },
            .read =
            {
                    ram_read,
                    ntable_read,
            },
    };

void bus_read(
        struct bus *bus,
        uint16_t address)
{
    //bus->servered_ = false;
    //bus->write_ = false;
    bus->address_ = address;
    bus->read[(address) / 0x2000]();
    if(bus == &cpu_bus)
        tick();
}

void bus_write(
        struct bus *bus,
        uint16_t address)
{
    //bus->servered_ = false;
    //bus->write_ = true;
    bus->address_ = address;
    bus->write[(address) / 0x2000]();
    if(bus == &cpu_bus)
        tick();
}

void bus_listen(
        struct peripheral *peripheral,
        struct bus *bus)
{
//    if(peripheral->address_min_ <= bus->address_ &&
//       peripheral->address_max_ >= bus->address_)// &&
////       !bus->servered_)
//    {
//        if(bus->write_) peripheral->write(peripheral);
//        else            peripheral->read(peripheral);
////        bus->servered_ = true;
//    }
}
