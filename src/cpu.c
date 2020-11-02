/*
 * cpu.c
 *
 *  Created on: May 5, 2020
 *      Author: David Jonsson
 */

#include <stdint.h>
#include <omp.h>
#include "cpu.h"
#include "instruction_tbl.h"
#include "clock.h"

bool shutdown = 0;

struct c6502 cpu =
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

};

void cpu_fetch_instruction()
{
    bus_read(&cpu_bus, cpu.program_counter_.word_);
    cpu.opcode_ = cpu_bus.data_;
    struct instruction *instruction = &instruction_tbl[cpu.opcode_];
    ++cpu.program_counter_.word_;

    /**
     * Bus read for the
     * extra tick, simulating
     * instruction decoding for
     * 1 byte instructions.
     */
    bus_read(&cpu_bus, cpu.program_counter_.word_);
    if(instruction->nr_bytes_ > 1)
    {
        *cpu.opcode_args_.lsb_ = cpu_bus.data_;
        ++cpu.program_counter_.word_;
    }
    if(instruction->nr_bytes_ > 2)
    {
        bus_read(&cpu_bus, cpu.program_counter_.word_);
        *cpu.opcode_args_.msb_ = cpu_bus.data_;
        ++cpu.program_counter_.word_;
    }
}

void cpu_execute_instruction()
{
    struct instruction *instruction = &instruction_tbl[cpu.opcode_];
    instruction->set_operand(instruction);
    if(instruction->flags_ & READ_DATA)
    {
        bus_read(&cpu_bus, cpu.adh_adl_.word_);
    }

    instruction->execute(instruction);

    if(instruction->flags_ & WRITE_DATA)
    {
        bus_write(&cpu_bus, cpu.adh_adl_.word_);
    }
}


void tick()
{
//    puts("cpu");
    // Peripheral reads on buses
    for(int i = 0; i < cpu_bus.nr_listeners_; ++i)
    {
        bus_listen(cpu_bus.listeners_[i], &cpu_bus);
    }
    ++cpu.nr_ticks_;

    ppu_run(), ppu_run(), ppu_run();
    //nanosleep(&nanosecs, NULL);
}

void cpu_run()
{
    while(!shutdown)
    {
        cpu.status_ |= CPU_STATUS_NOT_USED;
        if(!(cpu.suspend_etc_ & CPU_SUSPEND))
        {

            if(cpu.nmi_ )
            {
                cpu.opcode_ = 0x00;
            }
            else if((cpu.irq_ && !(cpu.status_ & CPU_STATUS_INTERUPT)))
            {
                cpu.opcode_ = 0x00;
            }
            else
            {
                ++cpu.nr_instructions;
                struct _16_bit
                    old_counter = cpu.program_counter_;
                cpu_fetch_instruction();
            }
            struct instruction *instruction = &instruction_tbl[cpu.opcode_];

            /*
             * logging
             */

//            cpu.program_counter_.word_ -= instruction->nr_bytes_ ? instruction->nr_bytes_ : 0;
//            log_state();
//            log_write("\n\0");
//            cpu.program_counter_.word_ += instruction->nr_bytes_ ? instruction->nr_bytes_ : 0;

            /*
             *  end logging
             */

            cpu_execute_instruction();

        }
    }}


omp_lock_t writelock;
static int temp_instruction;

void cpu_set_instruction()
{
    omp_set_lock(&writelock);
    if(temp_instruction)
    {
        uint8_t
            *byte = &temp_instruction,
            *address = &cpu.opcode_args_;


        cpu.opcode_ = temp_instruction;
        ++byte;
        *address = *byte;
        ++byte;
        ++address;
        *address = *byte;

    }
    temp_instruction = 0;
    omp_unset_lock(&writelock);
}


void cpu_load_instruction(int instruction)
{
    omp_set_lock(&writelock);
    temp_instruction = instruction;
    omp_unset_lock(&writelock);
}

