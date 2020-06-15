/*
 * peripheral.h
 *
 *  Created on: May 12, 2020
 *      Author: David Jonsson
 */

#ifndef SRC_PERIPHERAL_H_
#define SRC_PERIPHERAL_H_

#include "bus.h"

/**
 * IRQ lines
 */
#define IRQ_PPU         0b0000000000000001
#define IRQ_APU         0b0000000000000010


struct peripheral
{
    struct bus
        *bus_;
    uint16_t
        address_min_,
        address_max_,
        mirror_mask_,
        *irq_line_,
        *nmi_line_;
    uint8_t
        *memory_;
    void
        (*update)(struct peripheral *this),
        (*read)(struct peripheral *this),
        (*write)(struct peripheral *this);
};

void generic_update(struct peripheral *this);
void generic_write(struct peripheral *this);
void generic_read(struct peripheral *this);

extern struct peripheral
    cpu_peripheral_ram,
    cpu_peripheral_apu,
    cpu_peripheral_ppu,
    cpu_peripheral_dma,
    cpu_peripheral_prgrom,
    cpu_peripheral_prgram,
    ppu_peripheral_chrrom,
    ppu_peripheral_palette,
    ppu_peripheral_vram;

#endif /* SRC_PERIPHERAL_H_ */
