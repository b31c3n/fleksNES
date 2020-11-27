/*
 * ppu_comm.c
 *
 *  Created on: Nov 25, 2020
 *      Author: David Jonsson
 */

#include "ppu_comm.h"
#include "ppu_constants.h"
#include "peripherals.h"
#include "ppu.h"


static void write_ctrl()
{
    uint16_t
        test = ((uint16_t) 0b11 << 10) & VRAM_TABLESEL;
    ppu.regs_[PPU_CTRL] = ppu_comm.data_;
    ppu.tram_addr_ &= ~VRAM_TABLESEL;
    ppu.tram_addr_ |= ((uint16_t) ppu_comm.data_ << 10) & VRAM_TABLESEL;
}
static void write_mask()
{
    ppu.regs_[PPU_MASK] = ppu_comm.data_;
}

static void write_scroll()
{
    if(ppu.latch_)
    {
        uint16_t
            test = ((uint16_t) 0b11011101 << 2) & VRAM_COARSE_Y;
            test |= ((uint16_t) 0b11011101 << 12) & VRAM_FINE_Y;
        ppu.tram_addr_ &= ~(VRAM_COARSE_Y | VRAM_FINE_Y);
        ppu.tram_addr_ |= (((uint16_t) ppu_comm.data_) << 2) & VRAM_COARSE_Y;
        ppu.tram_addr_ |= (((uint16_t) ppu_comm.data_) << 12) & VRAM_FINE_Y;
    }
    else
    {
        uint16_t
            test1 = ((uint16_t) 0b11011101 >> 3) & VRAM_COARSE_X;
        uint16_t
            test2 = ((uint16_t) 0b11011101) & VRAM_FINE_X;
        ppu.tram_addr_ &= ~VRAM_COARSE_X;
        ppu.tram_addr_ |= (ppu_comm.data_ >> 3) & VRAM_COARSE_X;
        ppu.x_finescroll_ = ppu_comm.data_ & VRAM_FINE_X;
    }
    ppu.latch_ = ppu.latch_ ? 0 : 1;
}

static void write_addr()
{
    if(ppu.latch_)
    {
        ppu.tram_addr_ &= ~VRAM_LOW;
        ppu.tram_addr_ |= ppu_comm.data_;
        ppu.vram_addr_ = ppu.tram_addr_;
    }
    else
    {

        uint16_t
            test = (((uint16_t) 0b11011101) << 8) & VRAM_HIGH;
        ppu.tram_addr_ &= ~0xFF00;
        ppu.tram_addr_ |= (((uint16_t) ppu_comm.data_) << 8) & VRAM_HIGH;
    }
    ppu.latch_ = ppu.latch_ ? 0 : 1;
}


static void write_data()
{
    ppu_bus.data_ = ppu_comm.ppu_->bus_->data_;
    bus_write(&ppu_bus, ppu.vram_addr_ & 0x3FFF);
    ppu.vram_addr_ += ppu.regs_[PPU_CTRL] & PPU_CTRL_VRAM_INCR ? 32 : 1;
}

static void read_status()
{
    ppu_comm.ppu_->bus_->data_ = ppu.regs_[PPU_STATUS] & 0xE0;
    ppu.regs_[PPU_STATUS] &= ~PPU_STATUS_VBLANK;
    ppu.latch_ = 0;
}

static void read_data()
{
    ppu_comm.ppu_->bus_->data_ = ppu.ppu_data_buffer_;
    bus_read(&ppu_bus, ppu.vram_addr_ & 0x3FFF);
    ppu.ppu_data_buffer_ = ppu_bus.data_;

    if(ppu.vram_addr_ >= 0x3EFF)
    {
        ppu_comm.ppu_->bus_->data_ = ppu.ppu_data_buffer_;
    }
    ppu.vram_addr_ += ppu.regs_[PPU_CTRL] & PPU_CTRL_VRAM_INCR ? 32 : 1;
}

static void do_nothing()
{

}

struct ppu_comm_
    ppu_comm =
    {
            .ppu_ = &cpu_peripheral_ppu,
            .write_funcs =
            {
                    write_ctrl,
                    write_mask,
                    do_nothing,
                    do_nothing,
                    do_nothing,
                    write_scroll,
                    write_addr,
                    write_data
            },
            .read_funcs =
            {
                    do_nothing,
                    do_nothing,
                    read_status,
                    do_nothing,
                    do_nothing,
                    do_nothing,
                    do_nothing,
                    read_data
            }
    };

