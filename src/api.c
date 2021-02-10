/*
 * api.c
 *
 *  Created on: Feb 10, 2021
 *      Author: David Jonsson
 */

#include "api.h"
#include "state.h"
#include "refactoring.h"
#include "apu.h"

void fleks_init
(
    char    *game,
    uint8_t *mem,
    uint8_t *pixels,
    uint8_t *ram_mem
)
{
    struct nes_state
        *state = mem,

        temp_state =
        {
            .cpu_bus_ =
            {
                    .interval_ = 0x2000,
                    .write =
                    {
                            ram_write,
                            ppu_write,
                            apu_write,
                    },
                    .read =
                    {
                            ram_read,
                            ppu_read,
                            apu_read,
                    },
                    .ticker = cpu_tick
            },
            .ppu_bus_ =
            {
                    .interval_ = 0x1000,
                    .ticker = ppu_tick
            },
            .cpu_ =
            {
                    .status_ = 0x24,
                    .opcode_ = 0xEA,
                    .stack_pointer_ =
                    {
                        .word_ = 0x01FF,
                        .lsb_ = &cpu.stack_pointer_.word_,
                        .msb_ = 0x0
                    },
                    .adh_adl_ =
                    {
                        .word_ = 0x0,
                        .lsb_ = &cpu.adh_adl_.word_,
                        .msb_ = ((uint8_t *) (&cpu.adh_adl_.word_)) + 1
                    },
                    .opcode_args_ =
                    {
                        .word_ = 0x0,
                        .lsb_ = &cpu.opcode_args_.word_,
                        .msb_ = ((uint8_t *) (&cpu.opcode_args_.word_)) + 1
                    },
                    .program_counter_ =
                    {
                        .word_ = 0x0,
                        .lsb_ = &cpu.program_counter_.word_,
                        .msb_ = ((uint8_t *) (&cpu.program_counter_.word_)) + 1
                    },
            },
            .ppu_ =
            {
                    .latch_             = false,
                    .scanline_          = 261,
                    .cycle_             = 0,
            },
            .header_ = { .mapper_ctr_mirror_ = 0 },
            .ntable_mem_ = 0,
            .pal_mem_    = 0,
        };
    *state = temp_state;
    active_state = state;

    mapper_init(game, mapper_internal_mem);
}

void fleks_step(uint8_t *mem)
{

}

void fleks_destroy(uint8_t *mem)
{

}
