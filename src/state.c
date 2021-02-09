/*
 * state.c
 *
 *  Created on: Jan 23, 2021
 *      Author: David Jonsson
 */

#include <stdio.h>

#include "state.h"
#include "cpu.h"
#include "ppu.h"
#include "mapper.h"
#include "ram.h"
#include "apu.h"

#define cpu     internal_state.cpu_

struct nes_state
    internal_state =
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
    },
    *active_state = &internal_state;


void save_state(void)
{
    FILE *fp = fopen("state.000", "w");

    cpu_save_state(fp);
    ppu_save_state(fp);
    mapper_save_state(fp);

    fclose(fp);
}

void load_state(void)
{
    FILE *fp = fopen("state.000", "r");

    cpu_load_state(fp);
    ppu_load_state(fp);
    mapper_load_state(fp);

    fclose(fp);
}


