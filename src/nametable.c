/*
 * nametable.c
 *
 *  Created on: Aug 31, 2020
 *      Author: archie
 */

#include "peripherals.h"
#include "mapper.h"
#include "nametable.h"

uint8_t
	nametable_mem[2][1024];

void ntable_read()
{
    if(ppu_bus.address_ >= 0x3F00)
    {
        struct peripheral
            *this = &ppu_peripheral_palette;
        uint16_t
            address = this->bus_->address_ & this->mirror_mask_ - this->address_min_;
        address &= 0x1F;
        address -=  0x0010 * !((bool)(address & 0b00011)) * ((bool) (address & 0b10000));
        this->bus_->data_ = this->memory_[address];
    }
    else
    {
        struct peripheral
            *this = &ppu_peripheral_nametable;
        uint16_t
            address = this->bus_->address_;
        bool
            ntable_id;

        address &= 0x0FFF;
        ntable_id = 0;
        ntable_id += header.ver_mirror * ((bool) (address & 0x0400));
        ntable_id += (1 - header.ver_mirror) * ((bool) (address & 0x0800));

        address &= 0x03FF;
        uint8_t
            data = nametable_mem[ntable_id][address];
        this->bus_->data_ = data;
    }
}

void ntable_write()
{
    if(ppu_bus.address_ >= 0x3F00)
    {
        struct peripheral
            *this = &ppu_peripheral_palette;
        uint16_t
            address = this->bus_->address_ & this->mirror_mask_ - this->address_min_;
        address &= 0x1F;
        address -=  0x0010 * !((bool)(address & 0b00011)) * ((bool) (address & 0b10000));
        this->memory_[address] = this->bus_->data_;
    }
    else
    {
        struct peripheral
            *this = &ppu_peripheral_nametable;
        uint16_t
            address = this->bus_->address_;
        bool
            ntable_id;

        address &= 0x0FFF;
        ntable_id = 0;
        ntable_id += header.ver_mirror * ((bool) (address & 0x0400));
        ntable_id += (1 - header.ver_mirror) * ((bool) (address & 0x0800));

        address &= 0x03FF;
        uint8_t
            data = this->bus_->data_;
        nametable_mem[ntable_id][address] = data;
    }
}

struct peripheral
    ppu_peripheral_nametable =
{
    .bus_ = &ppu_bus,
    .address_min_ = 0x2000,
    .address_max_ = 0x3EFF,
    .mirror_mask_ = 0x23FF,
    .memory_ = &nametable_mem[0],
};


static uint8_t
    palette_mem[0x20];

struct peripheral
    ppu_peripheral_palette =
    {
            .address_max_   = 0x3FFF,
            .address_min_   = 0x3F00,
            .mirror_mask_   = 0x3F1F,
            .memory_        = &palette_mem,
            .bus_           = &ppu_bus,
            .irq_line_      = 0,
            .nmi_line_      = 0
    };
