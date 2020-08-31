/*
 * nametable.c
 *
 *  Created on: Aug 31, 2020
 *      Author: archie
 */

#include "peripheral.h"
#include "mapper.h"

uint8_t
	nametable_mem[2][1024];

static void ntable_update(struct peripheral *this) {}
static void ntable_read(struct peripheral *this)
{
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
static void ntable_write(struct peripheral *this)
{
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

struct peripheral
    ppu_peripheral_nametable =
{
    .bus_ = &ppu_bus,
    .address_min_ = 0x2000,
    .address_max_ = 0x3EFF,
    .mirror_mask_ = 0x23FF,
    .memory_ = &nametable_mem[0][0],
    .update = ntable_update,
    .read = ntable_read,
    .write = ntable_write
};

