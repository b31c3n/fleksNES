/*
 * bus.c
 *
 *  Created on: May 12, 2020
 *      Author: David Jonsson
 */

#include "bus.h"
#include "instruction.h"
#include "cpu.h"

struct bus
     cpu_bus =
     {
             .listeners_ = { &cpu_peripheral_ram, &cpu_peripheral_prgrom, &cpu_peripheral_prgram },
             .nr_listeners_ = 3
     };
struct bus
     ppu_bus;

void bus_read(
        struct bus *bus,
        uint16_t address)
{
    bus->write_ = false;
    bus->address_ = address;
    cpu_tick();
}

void bus_write(
        struct bus *bus,
        uint16_t address)
{
    bus->write_ = true;
    bus->address_ = address;
    cpu_tick();
}

void bus_listen(
        struct peripheral *peripheral,
        struct bus *bus)
{
    if(peripheral->address_min_ <= bus->address_ &&
       peripheral->address_max_ >= bus->address_)
    {
        if(bus->write_) peripheral->write(peripheral);
        else            peripheral->read(peripheral);
    }
}
