/*
 * ppu.c
 *
 *  Created on: Jun 5, 2020
 *      Author: David Jonsson
 */

#include "ppu.h"
#include "clock.h"
#include "helper_funcs.h"
#include "cpu.h"

void ppu_run(void)
{
    while(1)
    {
        for(;1; ppu.cycle_ 		++,
                ppu.cycle_		%= 341,
                ppu.scanline_ 	+= !ppu.cycle_ ? 1 : 0,
                ppu.scanline_ 	%= 262,
                ppu.frame_      += !ppu.scanline_ && !ppu.cycle_ ? 1 : 0)
        {

            /*
             * Graphics stuff
             */


            if(ppu.scanline_ == 261 && ppu.cycle_ == 339 && !(ppu.frame_ % 2))
            {
                ppu.cycle_ = 0;
                ppu.scanline_ = 0;
            }

            /*
             * Test and set for vblank
             */
            if(ppu.scanline_ == 241 && ppu.cycle_ == 1)
            {
                ppu.regs_[PPU_STATUS] = PPU_STATUS | PPU_STATUS_VBLANK;
                if(ppu.regs_[PPU_CTRL] &= PPU_CTRL_GENERATE_NMI)
                    cpu.nmi_ |= NMI_PPU;
            }
            else if(ppu.scanline_ == 261 && ppu.cycle_ == 1)
            {
                ppu.regs_[PPU_STATUS] = PPU_STATUS & ~PPU_STATUS_VBLANK;
            }

            /*
             * Other stuff
             */
            if(ppu.scanline_ >= 240 && ppu.scanline_ <= 260)
            {
                int test = 543534;;// Do nothing
            }
            else if((ppu.cycle_ >= 1 && ppu.cycle_ <= 256) ||
                    (ppu.cycle_ >= 321 && ppu.cycle_ <= 336))
            {
                switch((ppu.cycle_ - 1) % 8)
                {
                    case 0:
                    {
                        uint16_t
                            address = 0x2000 | (ppu.vram_addr_ & 0x0FFF);
                        bus_read(&ppu_bus, address);
                        for(int i = 0; i < ppu_bus.nr_listeners_; ++i)
                            bus_listen(ppu_bus.listeners_[i], &ppu_bus);
                        ppu.shiftreg_tile_id_ = ppu_bus.data_;
                        break;
                    }
                    case 2:
                    {
                        uint16_t
                            address =
                                0x23C0 |
                                (ppu.vram_addr_ & 0x0C00) |
                                ((ppu.vram_addr_ >> 4) & 0x38) |
                                ((ppu.vram_addr_ >> 2) & 0x07);
                        bus_read(&ppu_bus, address);
                        for(int i = 0; i < ppu_bus.nr_listeners_; ++i)
                            bus_listen(ppu_bus.listeners_[i], &ppu_bus);
                        ppu.shiftreg_attr_ = ppu_bus.data_;
                        break;
                    }
                    case 4:
                    {

                        break;
                    }
                    case 6:
                    {

                        break;
                    }
                    case 7:
                    {
                        /*
                         * X-scroll inc
                         */
                        if((ppu.vram_addr_ & 0x001F) == 31)
                        {
                            ppu.vram_addr_ &= ~0x001F;
                            ppu.vram_addr_ ^= 0x0400;
                        }
                        else
                            ppu.vram_addr_ += 1;
                    }
                }

                /*
                 * Y-scroll inc
                 */
                if(ppu.cycle_ == 256)
                {
                    if((ppu.vram_addr_ & 0x7000) != 0x7000)
                        ppu.vram_addr_ += 0x1000;
                    else
                    {
                        ppu.vram_addr_ &= ~0x7000;
                        int y = (ppu.vram_addr_ & 0x03E0) >> 5;
                        if (y == 29)
                        {
                            y = 0;
                            ppu.vram_addr_ ^= 0x0800;
                        }
                        else if (y == 31)
                            y = 0;
                        else
                            y += 1;
                        ppu.vram_addr_ = (ppu.vram_addr_ & ~0x03E0) | (y << 5);
                    }
                }

            }
            else if(ppu.cycle_ >= 257 && ppu.cycle_ <= 320)
            {
                if(ppu.cycle_ == 257)
                {
                    /*
                     * Reset X-scroll
                     */
                    ppu.vram_addr_ &= ~VRAM_COARSE_X;
                    ppu.vram_addr_ |= ppu.tram_addr_ & VRAM_COARSE_X;
                }
                if(ppu.cycle_ >= 280 && ppu.cycle_ <= 304)
                {
                    /*
                     * Reset Y-scroll
                     */
                    ppu.vram_addr_ &= ~VRAM_COARSE_Y;
                    ppu.vram_addr_ |= ppu.tram_addr_ & VRAM_COARSE_Y;
                    ppu.vram_addr_ &= ~VRAM_TABLESEL;
                    ppu.vram_addr_ |= ppu.tram_addr_ & VRAM_TABLESEL;
                }
                // Get sprites
            }
//            else if(ppu.cycle_ >= 337 && ppu.cycle_ <= 340)
//            {
//                // Double get 3rd next nameplate
//            }

            /*
             * Bus stuff
             */
            sem_wait(&nes_clock.ppu_sem_);
            //        puts("ppu");
            for(int i = 0; i < ppu_bus.nr_listeners_; ++i)
            {
                bus_listen(ppu_bus.listeners_[i], &ppu_bus);
            }
            sem_post(&nes_clock.pclock_sem_);
        }
    }
}

struct ppu_2C0X ppu =
{
        .read_flags_    = 0b10010100,
        .write_flags_   = 0b11111011,
        .latch_ 		= false,
        .scanline_      = 261,
        .cycle_         = 0
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
            ppu.regs_[PPU_CTRL] = data;
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
            this->bus_->data_ = ppu.regs_[PPU_STATUS] & 0xE0;
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
