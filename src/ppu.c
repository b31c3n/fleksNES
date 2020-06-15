/*
 * ppu.c
 *
 *  Created on: Jun 5, 2020
 *      Author: David Jonsson
 */

#include "ppu.h"


struct ppu_2C0X ppu =
{
        .read_flags_    = 0b10010100,
        .write_flags_   = 0b11111011
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
        *memory = this->bus_->data_;
        if(reg == PPU_SCROLL || reg == PPU_ADDR)
        {
            if(reg == PPU_SCROLL)   ppu.scroll_ |= ppu.latch_ ? *memory : (uint16_t) ((*memory) << 8);
            if(reg == PPU_ADDR)     ppu.ppu_addr_ |= ppu.latch_ ? *memory : (uint16_t) ((*memory) << 8);
            ppu.latch_ = 1;
        }
        else if(reg == PPU_DATA)
        {
            ppu_bus.data_ = ppu.regs_[PPU_DATA];
            bus_write(&ppu_bus, ppu.regs_[PPU_ADDR]);
            ppu.ppu_addr_ += ppu.regs_[PPU_CTRL] & PPU_CTRL_VRAM_INCR ? 32 : 1;
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
        if(reg == PPU_STATUS)
        {
            this->bus_ = ppu.regs_[reg];
            ppu.regs_[PPU_STATUS] &= ~PPU_STATUS_VBLANK;
            ppu.latch_ = 0;
        }
        if(reg == PPU_OAM_DATA)
        {
            if(ppu.regs_[PPU_ADDR] <= 0x3EFF)
            {
                this->bus_ = ppu.regs_[PPU_ADDR];
                ppu.regs_[PPU_ADDR] = ppu.oam_prm_[ppu.regs_[PPU_OAM_ADDR]];
            }
            else
            {
                ppu.regs_[PPU_ADDR] = ppu.oam_prm_[ppu.regs_[PPU_OAM_ADDR]];
                this->bus_ = ppu.regs_[PPU_ADDR];
            }
            ppu.ppu_addr_ += ppu.regs_[PPU_CTRL] & PPU_CTRL_VRAM_INCR ? 32 : 1;
        }
    }
}

struct peripheral cpu_peripheral_ppu =
{
    .bus_ = &cpu_bus,
    .address_min_ = 0x2000,
    .address_max_ = 0x3FFF,
    .mirror_mask_ = 0x7,
    .memory_ = &ppu.regs_,
    .update = ppu_update,
    .read = ppu_read,
    .write = ppu_write
};

struct peripheral ppu_peripheral_palette =
{

};
