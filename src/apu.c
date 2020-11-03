/*
 * apu.c
 *
 *  Created on: Jun 7, 2020
 *      Author: David Jonsson
 */

#include "peripheral.h"
#include "apu.h"

uint8_t
    controller_buffer,
    controller_state;

void apu_write()
{
    struct peripheral
        *this = &cpu_peripheral_apu;
    uint16_t
        address = this->bus_->address_ & this->mirror_mask_ - this->address_min_;
//    this->memory_[address] = this->bus_->data_;
    if(address == 0x16)
        controller_state = controller_buffer;
}
void apu_read()
{
    struct peripheral
        *this = &cpu_peripheral_apu;
    uint16_t
        address = this->bus_->address_ & this->mirror_mask_ - this->address_min_;
    if(address == 0x16)
    {
        this->bus_->data_ = controller_state & 0b1;
        controller_state >>= 1;
    }
}

uint8_t apu_mem[0x1F + 1];
struct peripheral cpu_peripheral_apu =
{
    .bus_ = &cpu_bus,
    .address_min_ = 0x4000,
    .address_max_ = 0x401F,
    .mirror_mask_ = 0x401F,
    .memory_ = &apu_mem,
};
