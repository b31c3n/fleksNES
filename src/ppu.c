/*
 * ppu.c
 *
 *  Created on: Jun 5, 2020
 *      Author: David Jonsson
 */

#include "ppu.h"
#include "clock.h"
#include "helper_funcs.h"

void ppu_run(void)
{
    while(1)
    {
        sem_wait(&nes_clock.ppu_sem_);
//        puts("ppu");
        for(int i = 0; i < ppu_bus.nr_listeners_; ++i)
        {
            bus_listen(ppu_bus.listeners_[i], &ppu_bus);
        }
        sem_post(&nes_clock.pclock_sem_);
    }
}

struct ppu_2C0X ppu =
{
        .read_flags_    = 0b10010100,
        .write_flags_   = 0b11111011,
        .latch_ 		= false,
};

static void ppu_update(struct peripheral *this) {}
static void ppu_write(struct peripheral *this)
{
    uint16_t
        address = this->bus_->address_ & this->mirror_mask_ - this->address_min_;
    uint8_t
        reg = 1 << address,
        *memory = this->memory_ + address;

    if(ppu.write_flags_ & reg)
    {
    	uint8_t
			data = this->bus_->data_;
    	if(address == PPU_CTRL)
    	{
        	ppu.tram_addr_ &= ~VRAM_TABLESEL;
        	ppu.tram_addr_ |= add_zeros(VRAM_TABLESEL, data ) & VRAM_TABLESEL;
    	}
    	else if(address == PPU_SCROLL || address == PPU_ADDR)
        {
            if(address == PPU_SCROLL)
            {
                if(ppu.latch_)
                {
                	ppu.tram_addr_ &= ~(VRAM_COARSE_Y | VRAM_FINE_Y);
                	uint16_t
                		result = add_zeros(VRAM_FINE_Y, data) & VRAM_FINE_Y;
                	ppu.tram_addr_ |= result;
                	ppu.tram_addr_ |= (data << 2) & VRAM_COARSE_Y;
                }
                else
				{
                	ppu.tram_addr_ &= ~VRAM_COARSE_X;
                	uint16_t
                		result = strip_zeros(VRAM_COARSE_X, (data & VRAM_COARSE_X));
                	ppu.tram_addr_ |= result;
                	ppu.x_finescroll_ = data & VRAM_FINE_X;
				}
            }
            else if(address == PPU_ADDR)
            {
                if(ppu.latch_)
                {
                	ppu.tram_addr_ &= ~VRAM_LOW;
                	ppu.tram_addr_ |= data & VRAM_LOW;
                }
                else
				{
                	ppu.tram_addr_ &= ~VRAM_HIGH;
                	uint16_t
                		result = add_zeros(VRAM_HIGH, data) & VRAM_HIGH;
                	ppu.tram_addr_ |= result;
				}
                if(ppu.latch_)
                	ppu.vram_addr_ = ppu.tram_addr_;
            }
            ppu.latch_ = ppu.latch_ ? 0 : 1;

        }
        else if(address == PPU_DATA)
        {
        	ppu.regs_[PPU_DATA] = this->bus_->data_;
            ppu_bus.data_ = this->bus_->data_;
//            ppu_bus.data_ = ppu.regs_[PPU_DATA];
            bus_write(&ppu_bus, ppu.vram_addr_);
            ppu.vram_addr_ += ppu.regs_[PPU_CTRL] & PPU_CTRL_VRAM_INCR ? 32 : 1;
        }
    }
}
static void ppu_read(struct peripheral *this)
{
    uint16_t
        address = this->bus_->address_ & this->mirror_mask_ - this->address_min_;
    uint8_t
        reg = 1 << address,
        *memory = this->memory_ + address;

    if(ppu.read_flags_ & reg)
    {
        if(address == PPU_STATUS)
        {
        	ppu.regs_[PPU_STATUS] |= PPU_STATUS_VBLANK;
        	this->bus_->data_ = ppu.regs_[PPU_STATUS];
            ppu.regs_[PPU_STATUS] &= ~PPU_STATUS_VBLANK;
            ppu.latch_ = 0;
        }
        if(address == PPU_DATA)
        {
            if(ppu.vram_addr_ <= 0x3EFF)
            {
                this->bus_->data_ = ppu.regs_[PPU_DATA];
                bus_read(&ppu_bus, ppu.vram_addr_);
                ppu.regs_[PPU_DATA] = ppu_bus.data_;
            }
            else
            {
                bus_read(&ppu_bus, ppu.vram_addr_);
                ppu.regs_[PPU_DATA] = ppu_bus.data_;
            	this->bus_->data_ = ppu.regs_[PPU_DATA];
            }
            ppu.vram_addr_ += ppu.regs_[PPU_CTRL] & PPU_CTRL_VRAM_INCR ? 32 : 1;
        }
    }
}

struct peripheral cpu_peripheral_ppu =
{
    .bus_ = &cpu_bus,
    .address_min_ = 0x2000,
    .address_max_ = 0x3FFF,
    .mirror_mask_ = 0x2007,
    .memory_ = &ppu.regs_,
    .update = ppu_update,
    .read = ppu_read,
    .write = ppu_write
};
