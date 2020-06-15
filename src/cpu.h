/*
 * cpu.h
 *
 *  Created on: May 5, 2020
 *      Author: David Jonsson
 */

#ifndef SRC_CPU_H_
#define SRC_CPU_H_

/**
 * STATUS-bits
 */
#define CPU_STATUS_CARRY        0b00000001
#define CPU_STATUS_ZERO         0b00000010
#define CPU_STATUS_INTERUPT     0b00000100
#define CPU_STATUS_DECIMAL      0b00001000
#define CPU_STATUS_BREAK        0b00010000
#define CPU_STATUS_NOT_USED     0b00100000
#define CPU_STATUS_OVERFLOW     0b01000000
#define CPU_STATUS_NEGATIVE     0b10000000

/**
 * CPU-other stuff
 */
#define CPU_RESET               0b00000001
#define CPU_SUSPEND             0b00000010

#include <stdbool.h>
#include <stdint.h>
#include "instruction.h"
#include "bus.h"
#include "16_bit.h"


struct c6502
{
    uint8_t
        accumulator_,
        x_,
        y_,
        status_,
        opcode_,
        suspend_etc_;
    uint16_t
        irq_,
        nmi_;
    uint64_t
        ticks_;
    struct _16_bit
        program_counter_,
        stack_pointer_,
        opcode_args_,
        adh_adl_;

} extern cpu;

void cpu_fetch_instruction();
void cpu_execute_instruction();
void cpu_tick();

/**
 * Synchronized
 */
void cpu_set_instruction();
void cpu_load_instruction(int instruction);

#endif /* SRC_CPU_H_ */
