/*
 * refactor_marcos.h
 *
 *  Created on: Jan 29, 2021
 *      Author: David Jonsson
 */

#ifndef SRC_REFACTORING_H_
#define SRC_REFACTORING_H_

#include "bus.h"
#include "ram.h"
#include "state.h"

/**
 * These macros became necessary after a refactorization,
 * where I implemented a struct containing the state of the NES
 */
#define cpu                 active_state->cpu_
#define ppu                 active_state->ppu_
#define cpu_bus             active_state->cpu_bus_
#define ppu_bus             active_state->ppu_bus_
#define ram                 active_state->ram_
#define controller_state    active_state->controller_state_
#define controller_buffer   active_state->controller_buffer_
#define header              active_state->header_
#define prg_rom             active_state->prg_rom_
#define prg_ram             active_state->prg_ram_
#define chr_rom             active_state->chr_rom_
#define nametable_mem       active_state->nametable_mem_
#define palette_mem         active_state->palette_mem_
#define ntable_mem          active_state->ntable_mem_
#define pal_mem             active_state->pal_mem_
#define prg_size            active_state->prg_size_
#define chr_size            active_state->chr_size_

#endif /* SRC_REFACTORING_H_ */
