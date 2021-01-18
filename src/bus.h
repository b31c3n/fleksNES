/*
 * bus.h
 *
 *  Created on: May 12, 2020
 *      Author: David Jonsson
 */

#ifndef SRC_BUS_H_
#define SRC_BUS_H_

#include <stdbool.h>
#include <stdint.h>

#include "ppu_constants.h"

struct bus
{
    uint8_t
        data_;
    uint16_t
        address_,
        interval_;
    void
        (*write[8]) (),
        (*read[8]) (),
        (*ticker) ();
};

extern struct bus
     cpu_bus,
     ppu_bus;

/**
 * @brief makes cpu tick 1 cycle
 */
void bus_read(
        struct bus *bus,
        uint16_t address);

/**
 * @brief makes cpu tick 1 cycle
 */
void bus_write(
        struct bus *bus,
        uint16_t address);


#endif /* SRC_BUS_H_ */
