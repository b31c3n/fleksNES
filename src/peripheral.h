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


#endif /* SRC_PERIPHERAL_H_ */
