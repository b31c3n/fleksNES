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

uint8_t
    mapper_internal_mem[0x20000000];

struct nes_state
    *active_state;



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


