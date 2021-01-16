/*
 * apu.c
 *
 *  Created on: Jun 7, 2020
 *      Author: David Jonsson
 */

#include "apu.h"
#include "cpu.h"

uint8_t
    controller_buffer,
    controller_state;

void apu_write()
{
    if(cpu_bus.address_ == 0x4014)
    {
        cpu.suspend_etc_ |= CPU_DMA;
    }
    else
    {
        uint16_t
            address = cpu_bus.address_ & 0x1F;
        if(address == 0x16)
            controller_state = controller_buffer;
    }
}
void apu_read()
{
    uint16_t
        address = cpu_bus.address_ & 0x1F;
    if(address == 0x16)
    {
        cpu_bus.data_ = controller_state & 0b1;
        controller_state >>= 1;
    }
}
