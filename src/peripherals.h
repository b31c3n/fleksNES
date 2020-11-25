/*
 * peripherals.h
 *
 *  Created on: Nov 25, 2020
 *      Author: David Jonsson
 */

#ifndef SRC_PERIPHERALS_H_
#define SRC_PERIPHERALS_H_

#include "peripheral.h"

extern struct peripheral
    cpu_peripheral_ram,
    cpu_peripheral_apu,
    cpu_peripheral_ppu,
    cpu_peripheral_dma,
    cpu_peripheral_prgrom,
    cpu_peripheral_prgram,
    ppu_peripheral_chrrom,
    ppu_peripheral_palette,
    ppu_peripheral_nametable;


#endif /* SRC_PERIPHERALS_H_ */
