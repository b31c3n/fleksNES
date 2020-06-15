/*
 * peripheral.c
 *
 *  Created on: May 12, 2020
 *      Author: David Jonsson
 */

#include "peripheral.h"

void generic_update(struct peripheral *this) {}
void generic_write(struct peripheral *this)
{
    uint16_t
        address = this->bus_->address_ & this->mirror_mask_ - this->address_min_;
    this->memory_[address] = this->bus_->data_;
}
void generic_read(struct peripheral *this)
{
    uint16_t
        address = this->bus_->address_ & this->mirror_mask_ - this->address_min_;
    this->bus_->data_ = this->memory_[address];
}
