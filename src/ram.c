/*
 * ram.c
 *
 *  Created on: Aug 21, 2020
 *      Author: David Jonsson
 */

#include "peripheral.h"
#include "bus.h"

static uint8_t
    ram[0x7FF];


struct peripheral
    cpu_peripheral_ram =
    {
            .read           = generic_read,
            .write          = generic_write,
            .update         = generic_update,
            .address_max_   = 0x1FFF,
            .address_min_   = 0x0,
            .mirror_mask_   = 0x0800,
            .memory_        = &ram,
            .bus_           = &cpu_bus,
            .irq_line_      = 0,
            .nmi_line_      = 0
    };
