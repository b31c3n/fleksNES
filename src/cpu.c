/*
 * cpu.c
 *
 *  Created on: May 5, 2020
 *      Author: David Jonsson
 */

#include <stdint.h>

#include "cpu.h"
#include "instruction_tbl.h"
#include "clock.h"
#include "ppu.h"
#include "config.h"
#include "ram.h"
#include "refactoring.h"

bool
    cpu_shutdown   = 0,
    cpu_pause      = 0,
    cpu_save       = 0,
    cpu_load       = 0;



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


void cpu_tick()
{
    ppu_run(), ppu_run(), ppu_run();
    //clock_nanosleep(&nanosecs, NULL);
}

void cpu_run()
{
    while(!cpu_shutdown)
    {
        cpu.status_ |= CPU_STATUS_NOT_USED;

        while(cpu_pause);
        if(cpu_save)
        {
            save_state();
            cpu_save = 0;
        }
        if(cpu_load)
        {
            load_state();
            cpu_load = 0;
        }

        if(cpu.suspend_etc_ & CPU_DMA)
        {
            uint16_t
                msb = cpu_bus.data_ << 8;

            /**
             * Suspend cpu, copy data to oam
             */
            cpu.suspend_etc_ |= CPU_SUSPEND;

            /*
             * Extra tick stuff?
             */
            if(cpu.suspend_etc_ & CPU_ODD_CYCLE)
            {
                cpu.suspend_etc_ ^=  CPU_ODD_CYCLE;
            }
            for(int lsb = 0; lsb <= 0xFF; ++lsb)
            {
                bus_read(&cpu_bus, msb + lsb);
                ppu.oam_prm_[lsb] = cpu_bus.data_;

                ppu_run(), ppu_run(), ppu_run();
                cpu.suspend_etc_ ^=  CPU_ODD_CYCLE;
            }

            cpu.suspend_etc_ ^= CPU_DMA;
            cpu.suspend_etc_ ^= CPU_SUSPEND;
        }

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
                struct _16_bit
                    old_counter = cpu.program_counter_;
                cpu_fetch_instruction();
            }
            struct instruction *instruction = &instruction_tbl[cpu.opcode_];

            #ifdef LOG
                cpu.program_counter_.word_ -= instruction->nr_bytes_ ? instruction->nr_bytes_ : 0;
                log_state();
                log_write("\n\0");
                cpu.program_counter_.word_ += instruction->nr_bytes_ ? instruction->nr_bytes_ : 0;
            #endif

            cpu_execute_instruction();
        }
        cpu.suspend_etc_ ^=  CPU_ODD_CYCLE;
    }
}

<<<<<<< HEAD
=======
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

>>>>>>> 3efba92c3b3e947a65011cae703266e9293b8943
void cpu_load_state(FILE *fp)
{
    fread(&cpu, 8, 1, fp);
    for(struct _16_bit *p = &cpu.program_counter_.word_;
        p < &cpu.adh_adl_ + 1; ++p)
    {
        fread(&p->word_, 2, 1, fp);
    }
    fread(ram, 0x7FF, 1, fp);

}

void cpu_save_state(FILE *fp)
{
    fwrite(&cpu, 8, 1, fp);
    for(struct _16_bit *p = &cpu.program_counter_.word_;
        p < &cpu.adh_adl_ + 1; ++p)
    {
        fwrite(&p->word_, 2, 1, fp);
    }
    fwrite(ram, 0x7FF, 1, fp);
}

