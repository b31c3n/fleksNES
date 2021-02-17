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
    char *game,
    char *nes_mem,
    char *mapper_mem,
    char **pixels,
    char **ram_mem,
    char **ctrl_buffer
)
{
    struct nes_state
        *state = nes_mem,

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
//                    .stack_pointer_ =
//                    {
//                        .word_ = 0x01FF,
//                        .lsb_ = &cpu.stack_pointer_.word_,
//                        .msb_ = 0x0
//                    },
//                    .adh_adl_ =
//                    {
//                        .word_ = 0x0,
//                        .lsb_ = &cpu.adh_adl_.word_,
//                        .msb_ = ((uint8_t *) (&cpu.adh_adl_.word_)) + 1
//                    },
//                    .opcode_args_ =
//                    {
//                        .word_ = 0x0,
//                        .lsb_ = &cpu.opcode_args_.word_,
//                        .msb_ = ((uint8_t *) (&cpu.opcode_args_.word_)) + 1
//                    },
//                    .program_counter_ =
//                    {
//                        .word_ = 0x0,
//                        .lsb_ = &cpu.program_counter_.word_,
//                        .msb_ = ((uint8_t *) (&cpu.program_counter_.word_)) + 1
//                    },
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

    _16_bit_init(&active_state->cpu_.stack_pointer_);
    _16_bit_init(&active_state->cpu_.adh_adl_);
    _16_bit_init(&active_state->cpu_.opcode_args_);
    _16_bit_init(&active_state->cpu_.program_counter_);

    active_state->cpu_.stack_pointer_.word_ = 0x01FF;
    active_state->cpu_.stack_pointer_.msb_ = 0x0;

    *ctrl_buffer = &active_state->controller_buffer_;
    *pixels      = &active_state->ppu_.pixels_;

    mapper_init(game, mapper_mem);
}

void fleks_step()
{
    cpu_execute_nextinstr();
}

void fleks_destroy()
{

}

void fleks_savestate()
{
    save_state();
}
void fleks_loadstate()
{
    load_state();
}
