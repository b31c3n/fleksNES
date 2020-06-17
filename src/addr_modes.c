/*
 * addr_modes.c
 *
 *  Created on: Jun 8, 2020
 *      Author: David Jonsson
 */


#include "addr_modes.h"

/**
 *
 */
void accumulator(struct instruction *this)
{
    this->operand_ = &cpu.accumulator_;
}

/**
 *
 */
void absolute(struct instruction *this)
{
    cpu.adh_adl_.word_ = cpu.opcode_args_.word_;
    this->operand_ = &cpu_bus.data_;
}

/**
 *
 */
void absolute_x(struct instruction *this)
{
    uint16_t
        address = cpu.opcode_args_.word_ + cpu.x_;
    bool
        bit8_1 = cpu.opcode_args_.word_ & 0x100,
        bit8_2 = address & 0x100,
        page_crossed = bit8_1 != bit8_2;

    if(!(this->flags_ & CHECK_PAGECROSS) || page_crossed)
    {
        cpu_tick();
    }

    cpu.adh_adl_.word_ = address;
    this->operand_ = &cpu_bus.data_;
}

/**
 *
 */
void absolute_y(struct instruction *this)
{
    uint16_t
        address = cpu.opcode_args_.word_ + cpu.y_;
    bool
        bit8_1 = cpu.opcode_args_.word_ & 0x100,
        bit8_2 = address & 0x100,
        page_crossed = bit8_1 != bit8_2;


    if(!(this->flags_ & CHECK_PAGECROSS) || page_crossed)
    {
        cpu_tick();
    }

    cpu.adh_adl_.word_ = address;
    this->operand_ = &cpu_bus.data_;
}

/**
 * 3-byte instruction (JMP)
 * byte 2 - 3 contains pointer to a pointer to
 * the address containing next instruction
 */
void absolute_indirect(struct instruction *this)
{

    bus_read(&cpu_bus, cpu.opcode_args_.word_);
    *cpu.adh_adl_.lsb_ = cpu_bus.data_;
    ++*cpu.opcode_args_.lsb_;
    bus_read(&cpu_bus, cpu.opcode_args_.word_);
    *cpu.adh_adl_.msb_ = cpu_bus.data_;

    this->operand_ = &cpu_bus.data_;
}


/**
 * 2-byte instruction
 */
void immediate(struct instruction *this)
{
    this->operand_ = cpu.opcode_args_.lsb_;
}

/**
 *
 */
void implied(struct instruction *this)
{

}

/**
 *
 */
void indirect(struct instruction *this)
{
    struct _16_bit temp_addr;
    *cpu.adh_adl_.msb_ = 0;
    *cpu.adh_adl_.lsb_ = *cpu.opcode_args_.lsb_;

    cpu_tick();
    bus_read(&cpu_bus, cpu.adh_adl_.word_);
    *temp_addr.lsb_ = cpu_bus.data_;

    ++*cpu.adh_adl_.lsb_;
    bus_read(&cpu_bus, cpu.adh_adl_.word_);
    *temp_addr.msb_ = cpu_bus.data_;
    cpu.adh_adl_.word_ = temp_addr.word_;
    this->operand_ = &cpu_bus.data_;
}

/**
 *
 */
void indexed_indirect(struct instruction *this)
{
    struct _16_bit temp_addr;
    _16_bit_init(&temp_addr);
    *cpu.adh_adl_.msb_ = 0;
    *cpu.adh_adl_.lsb_ = *cpu.opcode_args_.lsb_ + cpu.x_;

    cpu_tick();
    bus_read(&cpu_bus, cpu.adh_adl_.word_);
    *temp_addr.lsb_ = cpu_bus.data_;

    ++*cpu.adh_adl_.lsb_;
    bus_read(&cpu_bus, cpu.adh_adl_.word_);
    *temp_addr.msb_ = cpu_bus.data_;
    cpu.adh_adl_.word_ = temp_addr.word_;
    this->operand_ = &cpu_bus.data_;
}

/**
 *
 */
void indirect_indexed(struct instruction *this)
{
    struct _16_bit temp_addr;
    _16_bit_init(&temp_addr);
    *cpu.adh_adl_.msb_ = 0;
    *cpu.adh_adl_.lsb_ = *cpu.opcode_args_.lsb_;

    cpu_tick();
    bus_read(&cpu_bus, cpu.adh_adl_.word_);
    *temp_addr.lsb_ = cpu_bus.data_;

    ++*cpu.adh_adl_.lsb_;
    bus_read(&cpu_bus, cpu.adh_adl_.word_);
    *temp_addr.msb_ = cpu_bus.data_;
    cpu.adh_adl_.word_ = temp_addr.word_;

    cpu.adh_adl_.word_ += cpu.y_;

    bool
        bit8_1 = temp_addr.word_ & 0x100 ,
        bit8_2 = cpu.adh_adl_.word_ & 0x100,
        page_crossed = bit8_1 != bit8_2;

    if(!(this->flags_ & CHECK_PAGECROSS) || page_crossed)
    {
        cpu_tick();
    }
    this->operand_ = &cpu_bus.data_;
}

/**
 * 2 byte instruction, 2nd byte is the offset from PC
 */
void relative(struct instruction *this)
{
    if(cpu.opcode_args_.word_ & CPU_STATUS_NEGATIVE)
        cpu.opcode_args_.word_ |= 0xFF00;
    this->operand_ = &cpu.opcode_args_.word_;
}

/**
 *
 */
void zero_page(struct instruction *this)
{
    *cpu.adh_adl_.msb_ = 0;
    *cpu.adh_adl_.lsb_ = *cpu.opcode_args_.lsb_;
    this->operand_ = &cpu_bus.data_;
}

/**
 *
 */
void zero_page_x(struct instruction *this)
{
    *cpu.adh_adl_.msb_ = 0;
    *cpu.adh_adl_.lsb_ = *cpu.opcode_args_.lsb_ + cpu.x_;
    cpu_tick();
    this->operand_ = &cpu_bus.data_;
}

/**
 *
 */
void zero_page_y(struct instruction *this)
{
    *cpu.adh_adl_.msb_ = 0;
    *cpu.adh_adl_.lsb_ = *cpu.opcode_args_.lsb_ + cpu.y_;
    cpu_tick();
    this->operand_ = &cpu_bus.data_;
}
