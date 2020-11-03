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

static void inc_x()
{
    if(!(ppu.regs_[PPU_MASK] & PPU_MASK_SHOW_BACKGROUNND & PPU_MASK_SHOW_SPRITE)) return;

    if((ppu.vram_addr_ & 0x001F) == 31)
    {
        ppu.vram_addr_ &= ~0x001F;
        ppu.vram_addr_ ^= 0x0400;
    }
    else
        ppu.vram_addr_ += 1;
}

static void inc_y()
{
    if(!(ppu.regs_[PPU_MASK] & PPU_MASK_SHOW_BACKGROUNND & PPU_MASK_SHOW_SPRITE)) return;

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

static void reset_x()
{
    if(!(ppu.regs_[PPU_MASK] & PPU_MASK_SHOW_BACKGROUNND & PPU_MASK_SHOW_SPRITE)) return;

    uint16_t
        mask = VRAM_COARSE_X | 0x8400;
    ppu.vram_addr_ &= ~mask;
    ppu.vram_addr_ |= ppu.tram_addr_ & mask;
}

static void reset_y()
{
    if(!(ppu.regs_[PPU_MASK] & PPU_MASK_SHOW_BACKGROUNND & PPU_MASK_SHOW_SPRITE)) return;

    uint16_t
        mask = VRAM_COARSE_Y | VRAM_FINE_Y | 0x8800;
    ppu.vram_addr_ &= ~mask;
    ppu.vram_addr_ |= ppu.tram_addr_ & mask;
}

static void load_tile()
{
    switch((ppu.cycle_ - 1) % 8)
    {
        case 0:
        {
            uint16_t
                address = 0x2000 | (ppu.vram_addr_ & 0x0FFF);
            bus_read(&ppu_bus, address);
//            for(int i = 0; i < ppu_bus.nr_listeners_; ++i)
//                bus_listen(ppu_bus.listeners_[i], &ppu_bus);
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
//            for(int i = 0; i < ppu_bus.nr_listeners_; ++i)
//                bus_listen(ppu_bus.listeners_[i], &ppu_bus);
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
            inc_x();
        }
    }
}

void ppu_run(void)
{
    ppu.cycle_ 		++,
    ppu.cycle_		%= 341,
    ppu.scanline_ 	+= !ppu.cycle_ ? 1 : 0,
    ppu.scanline_ 	%= 262,
    ppu.frame_      += !ppu.scanline_ && !ppu.cycle_ ? 1 : 0;


    /*
     * Bus stuff
     */
//    for(int i = 0; i < ppu_bus.nr_listeners_; ++i)
//    {
//        bus_listen(ppu_bus.listeners_[i], &ppu_bus);
//    }

    /*
     * Pre-render scanline (-1 or 261)
     */
    if(ppu.scanline_ == 261)
    {
        /*
         * Set vblank at cycle 1
         */
        if(ppu.cycle_ == 1)
        {
            ppu.regs_[PPU_STATUS] = PPU_STATUS & ~PPU_STATUS_VBLANK;
            if(ppu.regs_[PPU_CTRL] & PPU_CTRL_GENERATE_NMI)
            {
                cpu.nmi_ = 1;
            }
        }

        /*
         * Check if end of scanline and on odd frame
         */
        if(ppu.cycle_ == 339 && !(ppu.frame_ % 2))
        {
            ppu.cycle_ = 0;
            ppu.scanline_ = 0;
        }

        /**
         * Load tiles
         */
        else if((ppu.cycle_ >= 1 && ppu.cycle_ <= 256) ||
                (ppu.cycle_ >= 321 && ppu.cycle_ <= 336))
        {
            load_tile();
            if(ppu.cycle_ == 256) inc_y();
        }

        /*
         * Reset x/y-scrolls
         */
        else if(ppu.cycle_ == 257)
            reset_x();
        else if(ppu.cycle_ >= 280 && ppu.cycle_ <= 304)
            reset_y();

    }

    /*
     * Visible scanlines (0-239)
     */
    if(ppu.scanline_ >= 0 || ppu.scanline_ <= 239)
    {
        /**
         * Load tiles
         */
        if((ppu.cycle_ >= 1 && ppu.cycle_ <= 256) ||
                (ppu.cycle_ >= 321 && ppu.cycle_ <= 336))
        {
            load_tile();
            if(ppu.cycle_ == 256) inc_y();
        }

        /*
         * Reset x-scroll
         */
        else if(ppu.cycle_ == 257)
            reset_x();
    }

    /*
     * Scanline 241
     */
    if(ppu.scanline_ == 241 && ppu.cycle_ == 1)
    {
        ppu.regs_[PPU_STATUS] = PPU_STATUS | PPU_STATUS_VBLANK;
    }
}


void ppu_write()
{
    struct peripheral
        *this = &cpu_peripheral_ppu;
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
            uint16_t
                test = ((uint16_t) 0b11 << 10) & VRAM_TABLESEL;
            ppu.regs_[PPU_CTRL] = data;
            ppu.tram_addr_ &= ~VRAM_TABLESEL;
            ppu.tram_addr_ |= ((uint16_t) data << 10) & VRAM_TABLESEL;
        }
        if(address == PPU_SCROLL)
        {
            if(ppu.latch_)
            {
                uint16_t
                    test = ((uint16_t) 0b11011101 << 2) & VRAM_COARSE_Y;
                    test |= ((uint16_t) 0b11011101 << 12) & VRAM_FINE_Y;
                ppu.tram_addr_ &= ~(VRAM_COARSE_Y | VRAM_FINE_Y);
                ppu.tram_addr_ |= ((uint16_t) data << 2) & VRAM_COARSE_Y;
                ppu.tram_addr_ |= ((uint16_t) data << 12) & VRAM_FINE_Y;
            }
            else
            {
                uint16_t
                    test1 = ((uint16_t) 0b11011101 >> 3) & VRAM_COARSE_X;
                uint16_t
                    test2 = ((uint16_t) 0b11011101) & VRAM_FINE_X;
                ppu.tram_addr_ &= ~VRAM_COARSE_X;
                ppu.tram_addr_ |= (data >> 3) & VRAM_COARSE_X;
                ppu.x_finescroll_ = data & VRAM_FINE_X;
            }
            ppu.latch_ = ppu.latch_ ? 0 : 1;
        }
        if(address == PPU_ADDR)
        {
            if(ppu.latch_)
            {
                ppu.tram_addr_ &= ~VRAM_LOW;
                ppu.tram_addr_ |= data;
                ppu.vram_addr_ = ppu.tram_addr_;
            }
            else
            {

                uint16_t
                    test = ((uint16_t) 0b11011101 << 8) & VRAM_HIGH;
                ppu.tram_addr_ &= ~0xFF00;
                ppu.tram_addr_ |= ((uint16_t) data << 8) & VRAM_HIGH;
            }
            ppu.latch_ = ppu.latch_ ? 0 : 1;
        }


        if(address == PPU_DATA)
        {
            //ppu.regs_[PPU_DATA] = this->bus_->data_;
            ppu_bus.data_ = this->bus_->data_;
            bus_write(&ppu_bus, ppu.vram_addr_);
            ppu.vram_addr_ += ppu.regs_[PPU_CTRL] & PPU_CTRL_VRAM_INCR ? 32 : 1;
        }
    }
}

void ppu_read()
{
    struct peripheral
        *this = &cpu_peripheral_ppu;
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

            this->bus_->data_ = ppu.ppu_data_buffer_;
            bus_read(&ppu_bus, ppu.vram_addr_);
            ppu.ppu_data_buffer_ = ppu_bus.data_;

            if(ppu.vram_addr_ <= 0x3EFF)
            {
                this->bus_->data_ = ppu.ppu_data_buffer_;
            }
            ppu.vram_addr_ += ppu.regs_[PPU_CTRL] & PPU_CTRL_VRAM_INCR ? 32 : 1;
        }
    }
}



struct ppu_2C0X ppu =
{
        .read_flags_    = 0b10010100,
        .write_flags_   = 0b11111011,
        .latch_         = false,
        .scanline_      = 0,
        .cycle_         = 0,
};

struct peripheral cpu_peripheral_ppu =
{
        .bus_ = &cpu_bus,
        .address_min_ = 0x2000,
        .address_max_ = 0x3FFF,
        .mirror_mask_ = 0x2007,
        .memory_ = &ppu.regs_,
};
