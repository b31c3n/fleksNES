/*
 * nametable.c
 *
 *  Created on: Aug 31, 2020
 *      Author: archie
 */

#include "peripheral.h"
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
        this->bus_->data_ = this->memory_[address];
    }
    else
    {
        struct peripheral
            *this = &ppu_peripheral_nametable;
        uint16_t
            address = this->bus_->address_;
        uint8_t
            ntable_id;
        if(header.ver_mirror)
        {
            if( (address >= 0x2000 && address <= 0x23FF) ||
                (address >= 0x2800 && address <= 0x2BFF))
                ntable_id = 0;
            else
                ntable_id = 1;
        }
        else if(header.hor_mirror)
        {
            if( (address >= 0x2000 && address <= 0x23FF) ||
                (address >= 0x2400 && address <= 0x27FF))
                ntable_id = 0;
            else
                ntable_id = 1;
        }
        else
            ntable_id = 0;

        this->bus_->data_ =
                nametable_mem[ntable_id][address & this->mirror_mask_ - this->address_min_];
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
        this->memory_[address] = this->bus_->data_;
    }
    else
    {
        struct peripheral
            *this = &ppu_peripheral_nametable;
        uint16_t
            address = this->bus_->address_;
        uint8_t
            ntable_id;
        if(header.ver_mirror)
        {
            if( (address >= 0x2000 && address <= 0x23FF) ||
                (address >= 0x2800 && address <= 0x2BFF))
                ntable_id = 0;
            else
                ntable_id = 1;
        }
        else if(header.hor_mirror)
        {
            if( (address >= 0x2000 && address <= 0x23FF) ||
                (address >= 0x2400 && address <= 0x27FF))
                ntable_id = 0;
            else
                ntable_id = 1;
        }
        else
            ntable_id = 0;

        nametable_mem[ntable_id][address & this->mirror_mask_ - this->address_min_] =
                this->bus_->data_;
    }
}

struct peripheral
    ppu_peripheral_nametable =
{
    .bus_ = &ppu_bus,
    .address_min_ = 0x2000,
    .address_max_ = 0x3EFF,
    .mirror_mask_ = 0x23FF,
    .memory_ = &nametable_mem[0][0],
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
