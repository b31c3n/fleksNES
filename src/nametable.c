/*
 * nametable.c
 *
 *  Created on: Aug 31, 2020
 *      Author: archie
 */

#include "mapper.h"
#include "nametable.h"
#include "bus.h"

uint8_t
	nametable_mem[2][1024],
    palette_mem[0x20];

void ntable_read()
{
    if(ppu_bus.address_ >= 0x3F00)
    {
        uint16_t
            address = ppu_bus.address_ & 0x1F;
        address -=  0x0010 * !((bool)(address & 0b00011)) * ((bool) (address & 0b10000));
        ppu_bus.data_ = palette_mem[address];
    }
    else
    {
        uint16_t
            address = ppu_bus.address_;
        bool
            ntable_id;

        address &= 0x0FFF;
        ntable_id = 0;
        ntable_id += header.ver_mirror_ * ((bool) (address & 0x0400));
        ntable_id += (1 - header.ver_mirror_) * ((bool) (address & 0x0800));

        address &= 0x03FF;
        uint8_t
            data = nametable_mem[ntable_id][address];
        ppu_bus.data_ = data;
    }
}

void ntable_write()
{
    if(ppu_bus.address_ >= 0x3F00)
    {
        uint16_t
            address = ppu_bus.address_ & 0x1F;
        address -=  0x0010 * !((bool)(address & 0b00011)) * ((bool) (address & 0b10000));
        palette_mem[address] = ppu_bus.data_;
    }
    else
    {
        uint16_t
            address = ppu_bus.address_;
        bool
            ntable_id;

        address &= 0x0FFF;
        ntable_id = 0;
        ntable_id += header.ver_mirror_ * ((bool) (address & 0x0400));
        ntable_id += (1 - header.ver_mirror_) * ((bool) (address & 0x0800));

        address &= 0x03FF;
        uint8_t
            data = ppu_bus.data_;
        nametable_mem[ntable_id][address] = data;
    }
}
