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
#include "peripheral.h"

struct bus
{
    uint8_t
        data_,
        nr_listeners_;
    uint16_t
        address_;
    bool
        write_;
    struct peripheral
        *listeners_[10];
};
extern struct bus
     cpu_bus;

extern struct bus
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

/**
 * @brief
 */
void bus_listen(
        struct peripheral *peripheral,
        struct bus *bus);

#endif /* SRC_BUS_H_ */
