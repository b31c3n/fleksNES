/*
 * ppu_comm.h
 *
 *  Created on: Nov 25, 2020
 *      Author: David Jonsson
 */

#ifndef SRC_PPU_COMM_H_
#define SRC_PPU_COMM_H_

#include <stdint.h>

struct ppu_comm_
{
    struct peripheral
        *ppu_;
    uint16_t
        address_;
    uint8_t
        reg_,
        data_;
    void
        (*write_funcs[8]) (),
        (*read_funcs[8]) ();
} extern ppu_comm;

#endif /* SRC_PPU_COMM_H_ */
