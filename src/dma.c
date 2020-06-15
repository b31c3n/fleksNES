/*
 * dma.c
 *
 *  Created on: Jun 6, 2020
 *      Author: David Jonsson
 */

#include "peripheral.h"
#include "ppu.h"
#include "cpu.h"

uint8_t dma_mem;
static void dma_update(struct peripheral *this) {}
static void dma_read(struct peripheral *this) {}
static void dma_write(struct peripheral *this)
{
    dma_mem = this->bus_->data_;
    uint16_t
        msb = dma_mem << 8;

    /**
     * Suspend cpu, copy data to oam
     */
    cpu.suspend_etc_ |= CPU_SUSPEND;
    for(int lsb = 0; lsb <= 0xFF; ++lsb)
    {
        bus_read(&cpu_bus, msb + lsb);
        ppu.oam_prm_[lsb] = cpu_bus.data_;
    }
    cpu.suspend_etc_ &= ~CPU_SUSPEND;
}

struct peripheral cpu_peripheral_dma =
{
    .bus_ = &cpu_bus,
    .address_min_ = 0x4014,
    .address_max_ = 0x4014,
    .mirror_mask_ = 0x4014,
    .memory_ = &dma_mem,
    .update = dma_update,
    .read = dma_read,
    .write = dma_write
};
