/*
 * palette.c
 *
 *  Created on: Aug 27, 2020
 *      Author: archie
 */

#include "peripheral.h"

void palette_update(struct peripheral *this) {}
void palette_write(struct peripheral *this)
{
    uint16_t
        address = this->bus_->address_ & this->mirror_mask_ - this->address_min_;
    this->memory_[address] = this->bus_->data_;
}
void palette_read(struct peripheral *this)
{
    uint16_t
        address = this->bus_->address_ & this->mirror_mask_ - this->address_min_;
    this->bus_->data_ = this->memory_[address];
}

static uint8_t
	palette_mem[0x20];

struct peripheral
	ppu_peripheral_palette =
    {
            .read           = palette_read,
            .write          = palette_write,
            .update         = palette_update,
            .address_max_   = 0x3FFF,
            .address_min_   = 0x3F00,
            .mirror_mask_   = 0x3F1F,
            .memory_        = &palette_mem,
            .bus_           = &ppu_bus,
            .irq_line_      = 0,
            .nmi_line_      = 0
    };
