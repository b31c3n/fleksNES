/*
 * state.h
 *
 *  Created on: Jan 23, 2021
 *      Author: David Jonsson
 */

#ifndef SRC_STATE_H_
#define SRC_STATE_H_

#include "cpu.h"
#include "ppu.h"
#include "mapper.h"
#include "bus.h"

void save_state(void);
void load_state(void);

struct nes_state
{
    /**
     * Cpu stuff
     */
    struct c6502
        cpu_;

    struct ppu_2C0X
        ppu_;

    struct bus
        cpu_bus_,
        ppu_bus_;

    size_t
        prg_size_,
        chr_size_;

    uint8_t
        controller_state_,
        controller_buffer_;

    /**
     * Nametable pointers
     */
    uint8_t
        **ntable_mem_,
        *pal_mem_;

    uint8_t
        ram_[0x7FF],
        nametable_mem_[2][1024],
        palette_mem_[0x20];

    uint8_t
        *prg_rom_,
        *prg_ram_,
        *chr_rom_;

    /**
     * Memory for mapping logic
     */
    uint8_t
        *mlogic_8bit_;
    uint32_t
        *mlogic_32bit_;
    uint16_t
        *mlogic_16bit_;

    struct ines_header
        header_;

};

extern struct nes_state
    *active_state,
    internal_state;

extern uint8_t
    mapper_internal_mem[0x20000000];

#endif /* SRC_STATE_H_ */
