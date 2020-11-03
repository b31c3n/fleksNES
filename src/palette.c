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

