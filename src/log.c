/*
 * log.c
 *
 *  Created on: Jun 10, 2020
 *      Author: David Jonsson
 */

#include <stdio.h>
#include "log.h"
#include "cpu.h"
#include "helper_funcs.h"
#include "instruction_tbl.h"

static FILE
    *log_file;
static uint16_t
    log_rows = 0;

void log_clear()
{
    log_file = fopen("./log", "w");
    fclose(log_file);
    log_rows = 0;
}

void log_write(char *string)
{
    log_file = fopen("./log", "a");
    fputs(string, log_file);
    fclose(log_file);
    ++log_rows;
//    if(log_rows > 1000)
//        log_clear();
}

void log_state()
{
    struct
        instruction *instruction = &instruction_tbl[cpu.opcode_];
    char
        buffer[256],
        instr_flags[9];

    int_to_binstring(cpu.status_, &instr_flags);

    sprintf(buffer, "%04x  %02x %02x %02x      A:%02x X:%02x Y:%02x     flags:%s\0",
            cpu.program_counter_.word_,
            cpu.opcode_,
            *cpu.opcode_args_.lsb_,
            *cpu.opcode_args_.msb_,
            cpu.accumulator_,
            cpu.x_,
            cpu.y_,
            instr_flags);
    log_write(buffer);
}
