/*
 * ram.c
 *
 *  Created on: Aug 21, 2020
 *      Author: David Jonsson
 */

#include "peripherals.h"
#include "bus.h"
#include "ram.h"

uint8_t
    ram[0x7FF];


void ram_write()
{
    struct peripheral
        *this = &cpu_peripheral_ram;
    uint16_t
        address = this->bus_->address_ & this->mirror_mask_ - this->address_min_;
    this->memory_[address] = this->bus_->data_;
}

void ram_read()
{
    struct peripheral
        *this = &cpu_peripheral_ram;
    uint16_t
        address = this->bus_->address_ & this->mirror_mask_ - this->address_min_;
    this->bus_->data_ = this->memory_[address];
}

struct peripheral
    cpu_peripheral_ram =
    {
            .address_max_   = 0x1FFF,
            .address_min_   = 0x0,
            .mirror_mask_   = 0x07FF,
            .memory_        = &ram,
            .bus_           = &cpu_bus,
            .irq_line_      = 0,
            .nmi_line_      = 0
    };
