/*
 * instructions.c
 *
 *  Created on: May 4, 2020
 *      Author: David Jonsson
 */
#include "instruction.h"
#include "16_bit.h"
#include "refactoring.h"

void branching_stuff(struct instruction *this)
{
    uint16_t
        result = (uint16_t) *cpu.program_counter_.lsb_ + *cpu.opcode_args_.lsb_;
    bool
        carry = result & 0x100,
        borrow = result & 0x80,
        negative = *cpu.opcode_args_.lsb_ & 0x80;

    *cpu.program_counter_.lsb_ = result;
    cpu_tick();

    if(negative && !carry)
    {
        *cpu.program_counter_.msb_ += 0xFF;
        cpu_tick();
    }
    else if(!negative && carry)
    {
        *cpu.program_counter_.msb_ += carry;
        cpu_tick();
    }
}


/**
 * ADC Add memory to accumulator with carry
 * Operation: A + M + C -> A, C
 */
void adc(struct instruction *this)
{
    uint8_t
        operand = *this->operand_,
        acc     = cpu.accumulator_;
    uint16_t
        result  = (uint16_t) cpu.accumulator_ + operand + (cpu.status_ & CPU_STATUS_CARRY);

    uint8_t
        carry       = (bool) (result & 0x100),
        negative    = result & 0x80,
        zero        = 2 - ((bool) (result & 0xFF) << 1),
        overflow    = (~(operand ^ acc) & (result ^ acc) & 0x80) >> 1;

    cpu.status_ &= 0b00111100;
    cpu.status_ |= carry | negative | overflow | zero;
    cpu.accumulator_ = result & 0xFF;
}

/**
 * AND Memory with Accumulator
 * Operation: A ^ M -> A
 */
void and(struct instruction *this)
{
    cpu.accumulator_ &= *this->operand_;

    uint8_t
        acc         = cpu.accumulator_,
        negative    = acc & 0x80,
        zero        = 2 - ((bool) (acc & 0xFF) << 1);

    cpu.status_ &= 0b01111101;
    cpu.status_ |= negative | zero;
}

/**
 * ASL Shift Left One Bit (Memory or Accumulator)
 * Operation: C <- 7 6 5 4 3 2 1 0 <- 0
 */
void asl(struct instruction *this)
{
    uint8_t
        *address = this->operand_;
    uint16_t
        result = (*address) << 1;

    uint8_t
        carry       = (bool) (result & 0x100),
        negative    = result & 0x80,
        zero        = 2 - ((bool) (result & 0xFF) << 1);

    cpu.status_ &= 0b01111100;
    cpu.status_ |= negative | zero | carry;

    *address = result;
}

/**
 * BCC Branch on Carry Clear
 * Operation: Branch on C = 0
 */
void bcc(struct instruction *this)
{
    if(!(cpu.status_ & CPU_STATUS_CARRY))
    {
        branching_stuff(this);
    }
}

/**
 * BCS Branch on Carry Set
 * Operation: Branch on C = 1
 */
void bcs(struct instruction *this)
{
    if((cpu.status_ & CPU_STATUS_CARRY))
    {
        branching_stuff(this);
    }
}

/**
 * BEQ Branch on Result Zero
 * Operation: Branch on Z = 1
 */
void beq(struct instruction *this)
{
    if((cpu.status_ & CPU_STATUS_ZERO))
    {
        branching_stuff(this);
    }
}

/**
 * BIT Test bits in memory with accumulator
 * Operation: a ^ M, M7 -> N, M6 ->V
 *
 * Bit 6 and bit 7 are transferred to the status register.
 * If the result of A ^ M is zero then Z = 1, otherwise Z = 0
 */
void bit(struct instruction *this)
{
    uint8_t
        operand     = *this->operand_,
        negative    = operand & 0x80,
        overflow    = operand & 0x60,
        zero        = (1 - (bool) (cpu.accumulator_ & operand)) << 1;

    cpu.status_ &= 0b00111101;
    cpu.status_ |= overflow | negative | zero;
}

/**
 * BMI Branch on result minus
 * Operation: Branch on N = 1
 */
void bmi(struct instruction *this)
{
    if((cpu.status_ & CPU_STATUS_NEGATIVE))
    {
        branching_stuff(this);
    }
}

/**
 * BNE Branch on result not zero
 * Operation: Branch on Z = 0
 */
void bne(struct instruction *this)
{
    if(!(cpu.status_ & CPU_STATUS_ZERO))
    {
        branching_stuff(this);
    }
}

/**
 * BPL Branch on result plus
 * Operation: Branch on N = 0
 */
void bpl(struct instruction *this)
{
    if(!(cpu.status_ & CPU_STATUS_NEGATIVE))
    {
        branching_stuff(this);
    }
}

/**
 * BRK Force Break
 * Operation: Forced Interrupt PC + 2 v P v
 */
void brk(struct instruction *this)
{
    uint16_t
            irq_addr = 0xFFFE,
            nmi_addr = 0xFFFA;
    struct _16_bit
            effective_addr;

    _16_bit_init(&effective_addr);
    cpu.status_ |= CPU_STATUS_INTERUPT;
    uint8_t
        status_bits = (cpu.irq_ || cpu.nmi_) ? 0b100000 : 0b110000 ;
    status_bits |= cpu.status_;

    cpu_bus.data_ = *cpu.program_counter_.msb_;
    bus_write(&cpu_bus, cpu.stack_pointer_.word_);
    --*cpu.stack_pointer_.lsb_;

    cpu_bus.data_ = *cpu.program_counter_.lsb_;
    bus_write(&cpu_bus, cpu.stack_pointer_.word_);
    --*cpu.stack_pointer_.lsb_;


    cpu_bus.data_ = status_bits;
    bus_write(&cpu_bus, cpu.stack_pointer_.word_);
    --*cpu.stack_pointer_.lsb_;

    /*
     * Non maskable interrupt?
     */
    bool
        nmi = (bool) cpu.nmi_;

    effective_addr.word_ += nmi_addr * nmi;
    effective_addr.word_ += irq_addr * (1 - nmi);
    cpu.nmi_ -= cpu.nmi_ * nmi;
    cpu.irq_ -= cpu.irq_ * (1 - nmi);

    bus_read(&cpu_bus, effective_addr.word_);
    *cpu.program_counter_.lsb_ = cpu_bus.data_;
    ++*effective_addr.lsb_;

    bus_read(&cpu_bus, effective_addr.word_);
    *cpu.program_counter_.msb_ = cpu_bus.data_;
}

/**
 * BVC Branch on overflow clear
 * Operation: Branch on V = 0
 */
void bvc(struct instruction *this)
{
    if(!(cpu.status_ & CPU_STATUS_OVERFLOW))
    {
        branching_stuff(this);
    }
}

/**
 * BVS Branch on overflow set
 * Operation: Branch on V = 1
 */
void bvs(struct instruction *this)
{
    if((cpu.status_ & CPU_STATUS_OVERFLOW))
    {
        branching_stuff(this);
    }
}

/**
 * CLC Clear carry flag
 * Operation: 0 -> C
 */
void clc(struct instruction *this)
{
    cpu_tick(); // Need 1 cycly for instruction to be interpreted
    cpu.status_ &= ~CPU_STATUS_CARRY;
}

/**
 * CLD Clear decimal mode
 * Operation: 0 -> D
 */
void cld(struct instruction *this)
{
    cpu_tick(); // Need 1 cycly for instruction to be interpreted
    cpu.status_ &= ~CPU_STATUS_DECIMAL;
}

/**
 * CLI Clear interrupt disable bit
 * Operation: 0 -> I
 */
void cli(struct instruction *this)
{
    cpu_tick(); // Need 1 cycly for instruction to be interpreted
    cpu.status_ &= ~CPU_STATUS_INTERUPT;
}

/**
 * CLV Clear overflow flag
 * Operation: 0 -> V
 */
void clv(struct instruction *this)
{
    cpu_tick(); // Need 1 cycly for instruction to be interpreted
    cpu.status_ &= ~CPU_STATUS_OVERFLOW;
}

/**
 * CMP Compare memory and accumulator
 * Operation: A - M
 */
void cmp(struct instruction *this)
{
    uint8_t
        result      = cpu.accumulator_ - *this->operand_,
        negative    = result & 0x80,
        zero        = 2 - ((bool) (result & 0xFF) << 1),
        carry       = (bool) (*this->operand_ <= cpu.accumulator_);

    cpu.status_ &= 0b01111100;
    cpu.status_ |= negative | zero | carry;
}

/**
 * CPX Compare Memory and Index X
 * Operation: X - M
 */
void cpx(struct instruction *this)
{
    uint8_t
        *operand = this->operand_,
        result = cpu.x_ - *operand,
        negative    = result & 0x80,
        zero        = 2 - ((bool) (result & 0xFF) << 1),
        carry       = (bool) (*this->operand_ <= cpu.x_);

    cpu.status_ &= 0b01111100;
    cpu.status_ |= negative | zero | carry;
}

/**
 * CPY Compare Memory and Index Y
 * Operation: Y - M
 */
void cpy(struct instruction *this)
{
    uint8_t
        *operand = this->operand_,
        result = cpu.y_ - *operand,
        negative    = result & 0x80,
        zero        = 2 - ((bool) (result & 0xFF) << 1),
        carry       = (bool) (*this->operand_ <= cpu.y_);

    cpu.status_ &= 0b01111100;
    cpu.status_ |= negative | zero | carry;
}

/**
 * DEC Decrement memory by one
 * Operation: M - 1 -> M
 */
void dec(struct instruction *this)
{
    uint8_t
        *operand = this->operand_,
        result = --(*operand),
        negative    = result & 0x80,
        zero        = 2 - ((bool) (result & 0xFF) << 1);

    cpu.status_ &= 0b01111101;
    cpu.status_ |= negative | zero;

    *operand = result;
}

/**
 * DEX Decrement index X by one
 * Operation: X - 1 -> X
 */
void dex(struct instruction *this)
{
    uint8_t
        *operand = &cpu.x_,
        result = --(*operand),
        negative    = result & 0x80,
        zero        = 2 - ((bool) (result & 0xFF) << 1);

    cpu.status_ &= 0b01111101;
    cpu.status_ |= negative | zero;

    *operand = result;
}

/**
 * DEY Decrement index Y by one
 * Operation: Y - 1 -> Y
 */
void dey(struct instruction *this)
{
    uint8_t
        *operand = &cpu.y_,
        result = --(*operand),
        negative    = result & 0x80,
        zero        = 2 - ((bool) (result & 0xFF) << 1);

    cpu.status_ &= 0b01111101;
    cpu.status_ |= negative | zero;

    *operand = result;
}

/**
 * EOR "Exclusive-Or" memory with accumulator
 * Operation: A v M -> A
 */
void eor(struct instruction *this)
{
    uint8_t
        *operand = this->operand_,
        result = cpu.accumulator_ ^ (*operand),
        negative    = result & 0x80,
        zero        = 2 - ((bool) (result & 0xFF) << 1);

    cpu.status_ &= 0b01111101;
    cpu.status_ |= negative | zero;

    cpu.accumulator_ = result;
}

/**
 * INC Increment memory by one
 * Operation: M + -> M
 */
void inc(struct instruction *this)
{
    uint8_t
        *operand = this->operand_,
        result = 1 + (*operand),
        negative    = result & 0x80,
        zero        = 2 - ((bool) (result & 0xFF) << 1);

    cpu.status_ &= 0b01111101;
    cpu.status_ |= negative | zero;

    *operand = result;
}

/**
 * INX Increment Index X by one
 * Operation: X + -> X
 */
void inx(struct instruction *this)
{
    uint8_t
        *operand = &cpu.x_,
        result = 1 + (*operand),
        negative    = result & 0x80,
        zero        = 2 - ((bool) (result & 0xFF) << 1);

    cpu.status_ &= 0b01111101;
    cpu.status_ |= negative | zero;

    *operand = result;
}

/**
 * INY Increment Index Y by one
 * Operation: Y + -> Y
 */
void iny(struct instruction *this)
{
    uint8_t
        *operand = &cpu.y_,
        result = 1 + (*operand),
        negative    = result & 0x80,
        zero        = 2 - ((bool) (result & 0xFF) << 1);

    cpu.status_ &= 0b01111101;
    cpu.status_ |= negative | zero;

    *operand = result;
}

/**
 * JMP Jump to new location
 * Operation: (PC + 1) -> PCL
 *            (PC + 1) -> PCH
 */
void jmp(struct instruction *this)
{
    cpu.program_counter_.word_ = cpu.adh_adl_.word_;
}

/**
 * JSR Jump to new location saving return address
 * Operation: PC + 2 v , (PC + 1) -> PCL
 *            (PC + 1) -> PCH
 */
void jsr(struct instruction *this)
{

    uint8_t
        *address_p = (uint8_t *) (&cpu.program_counter_) + 1;

    /**
     * Some foul play here, 6502 stores last address of the JRS instr,
     * RTS increments it when its popped of the stack
     */
    --cpu.program_counter_.word_;

    cpu_bus.data_ = *address_p;
    bus_write(&cpu_bus, cpu.stack_pointer_.word_);
    --*cpu.stack_pointer_.lsb_;

    --address_p;
    cpu_bus.data_ = *address_p;
    bus_write(&cpu_bus, cpu.stack_pointer_.word_);
    --*cpu.stack_pointer_.lsb_;

    cpu.program_counter_.word_ = cpu.opcode_args_.word_;
    cpu_tick();
}

/**
 * LDA Load accumulator with memory
 * Operation: M -> A
 */
void lda(struct instruction *this)
{
    uint8_t
        *operand = this->operand_,
        result = (*operand),
        negative    = result & 0x80,
        zero        = 2 - ((bool) (result & 0xFF) << 1);

    cpu.status_ &= 0b01111101;
    cpu.status_ |= negative | zero;

    cpu.accumulator_ = result;
}

/**
 * LDX Load index X with memory
 * Operation: M -> X
 */
void ldx(struct instruction *this)
{
    uint8_t
        *operand = this->operand_,
        result = (*operand),
        negative    = result & 0x80,
        zero        = 2 - ((bool) (result & 0xFF) << 1);

    cpu.status_ &= 0b01111101;
    cpu.status_ |= negative | zero;

    cpu.x_ = result;
}

/**
 * LDY Load index Y with memory
 * Operation: M -> Y
 */
void ldy(struct instruction *this)
{
    uint8_t
        *operand = this->operand_,
        result = (*operand),
        negative    = result & 0x80,
        zero        = 2 - ((bool) (result & 0xFF) << 1);

    cpu.status_ &= 0b01111101;
    cpu.status_ |= negative | zero;

    cpu.y_ = result;
}

/**
 * LSR Shift right one bit (memory or accumulator)
 * Operation: 0 -> byte ->C
 */
void lsr(struct instruction *this)
{
    uint8_t
        *address = this->operand_;
    uint16_t
        result = (*address) >> 1;

    uint8_t
        carry       = (bool) (*address & 0x01),
        zero        = 2 - ((bool) (result & 0xFF) << 1);

    cpu.status_ &= 0b01111100;
    cpu.status_ |= zero | carry;

    *address = result;
}

/**
 * NOP No operation
 * Operation: No Operation (2 cycles)
 */
void nop(struct instruction *this)
{

}

/**
 * ORA "OR" memory with accumulator
 * Operation: A v M -> A
 */
void ora(struct instruction *this)
{
    uint8_t
        *operand = this->operand_,
        result = cpu.accumulator_ | (*operand),
        negative    = result & 0x80,
        zero        = 2 - ((bool) (result & 0xFF) << 1);

    cpu.status_ &= 0b01111101;
    cpu.status_ |= negative | zero;

    cpu.accumulator_ = result;
}

/**
 * PHA Push accumulator on stack
 * Operation: A v
 */
void pha(struct instruction *this)
{
    cpu_bus.data_ = cpu.accumulator_;
    bus_write(&cpu_bus, cpu.stack_pointer_.word_);
    --*cpu.stack_pointer_.lsb_;
}

/**
 * PHP Push processor status on stack
 * Operation: P v
 */
void php(struct instruction *this)
{
    cpu_bus.data_ = cpu.status_ | 0b110000;
    bus_write(&cpu_bus, cpu.stack_pointer_.word_);
    --*cpu.stack_pointer_.lsb_;
}

/**
 * PLA Pull accumulator from stack
 * Operation: A ^
 */
void pla(struct instruction *this)
{
    ++*cpu.stack_pointer_.lsb_;
    bus_read(&cpu_bus, cpu.stack_pointer_.word_);
    cpu.accumulator_ = cpu_bus.data_;

    uint8_t
        acc         = cpu.accumulator_,
        negative    = acc & 0x80,
        zero        = 2 - ((bool) (acc & 0xFF) << 1);

    cpu.status_ &= 0b01111101;
    cpu.status_ |= negative | zero;
}

/**
 * PLP Pull processor status from stack
 * Operation: P ^
 */
void plp(struct instruction *this)
{
    ++*cpu.stack_pointer_.lsb_;
    bus_read(&cpu_bus, cpu.stack_pointer_.word_);
    cpu.status_ = cpu_bus.data_ & ~0b110000;
}

/**
 * ROL Rotate one bit left (memory or accumulator)
 * Operation: |-<- byte <- C <-|
 */
void rol(struct instruction *this)
{
    uint8_t
        *address = this->operand_;
    uint16_t
        result = *address;

    result <<=  1;
    result |= cpu.status_ & CPU_STATUS_CARRY;

    uint8_t
        carry       = (bool) (result & 0x100),
        negative    = result & 0x80,
        zero        = 2 - ((bool) (result & 0xFF) << 1);

    cpu.status_ &= 0b01111100;
    cpu.status_ |= negative | zero | carry;

    *address = result;
}

/**
 * ROR Rotate one bit right (memory or accumulator)
 * Operation: |-> C -> byte ->-|
 */
void ror(struct instruction *this)
{
    uint8_t
        *address = this->operand_;
    uint16_t
        result = *address;

    result >>=  1;
    result |= (cpu.status_ & CPU_STATUS_CARRY) << 7;

    uint8_t
        carry       = (bool) (*address & 0x01),
        negative    = result & 0x80,
        zero        = 2 - ((bool) (result & 0xFF) << 1);

    cpu.status_ &= 0b01111100;
    cpu.status_ |= negative | zero | carry;

    *address = result;
}

/**
 * RTI Return from interrupt
 * Operation: P ^ PC^
 */
void rti(struct instruction *this)
{
    /**
     * Discarded read
     */
    bus_read(&cpu_bus, cpu.stack_pointer_.word_);

    ++*cpu.stack_pointer_.lsb_;
    bus_read(&cpu_bus, cpu.stack_pointer_.word_);
    cpu.status_ = cpu_bus.data_ & ~0b110000;

    ++*cpu.stack_pointer_.lsb_;
    bus_read(&cpu_bus, cpu.stack_pointer_.word_);
    *cpu.program_counter_.lsb_ = cpu_bus.data_;

    ++*cpu.stack_pointer_.lsb_;
    bus_read(&cpu_bus, cpu.stack_pointer_.word_);
    *cpu.program_counter_.msb_ = cpu_bus.data_;

}

/**
 * RTS Return from subroutine
 * Operation: PC^, PC + 1 -> PC
 */
void rts(struct instruction *this)
{
    /**
     * Discarded read
     */
    bus_read(&cpu_bus, cpu.stack_pointer_.word_);

    ++*cpu.stack_pointer_.lsb_;
    bus_read(&cpu_bus, cpu.stack_pointer_.word_);
    *cpu.program_counter_.lsb_ = cpu_bus.data_;

    ++*cpu.stack_pointer_.lsb_;
    bus_read(&cpu_bus, cpu.stack_pointer_.word_);
    *cpu.program_counter_.msb_ = cpu_bus.data_;
    ++cpu.program_counter_.word_;
}

/**
 * SBC Subtract memory from accumulator with borrow
 * Operation: A - M - -C -> A
 * Note: -C = Borrow
 */
void sbc(struct instruction *this)
{
    uint8_t
        operand = ~(*this->operand_),
        acc = cpu.accumulator_;
    uint16_t
        result = (uint16_t) cpu.accumulator_ + operand + (cpu.status_ & CPU_STATUS_CARRY);

    uint8_t
        carry       = (bool) (result & 0x100),
        negative    = result & 0x80,
        zero        = 2 - ((bool) (result & 0xFF) << 1),
        overflow    = (~(operand ^ acc) & (result ^ acc) & 0x80) >> 1;

    cpu.status_ &= 0b00111100;
    cpu.status_ |= carry | negative | overflow | zero;
    cpu.accumulator_ = result & 0xFF;
}

/**
 * SEC Set carry flag
 * Operation: 1 -> C
 */
void sec(struct instruction *this)
{
    cpu.status_ |= CPU_STATUS_CARRY;
}

/**
 * SED Set decimal mode
 * Operation: 1 -> D
 */
void sed(struct instruction *this)
{
    cpu.status_ |= CPU_STATUS_DECIMAL;
}

/**
 * SEI Set interrupt disable status
 * Operation: 1 -> I
 */
void sei(struct instruction *this)
{
    cpu.status_ |= CPU_STATUS_INTERUPT;
}

/**
 * STA Store accumulator in memory
 * Operation: A -> M
 */
void sta(struct instruction *this)
{
    *this->operand_ = cpu.accumulator_;
}

/**
 * STX Store index X in memory
 * Operation: X -> M
 */
void stx(struct instruction *this)
{
    *this->operand_ = cpu.x_;
}

/**
 * STY Store index Y in memory
 * Operation: Y -> M
 */
void sty(struct instruction *this)
{
    *this->operand_ = cpu.y_;
}

/**
 * TAX Transfer accumulator to index X
 * Operation: A -> X
 */
void tax(struct instruction *this)
{
    cpu.x_ = cpu.accumulator_;
}

/**
 * TAY Transfer accumulator to index Y
 * Operation: A -> Y
 */
void tay(struct instruction *this)
{
    cpu.y_ = cpu.accumulator_;

}

/**
 * TYA Transfer index Y to accumulator
 * Operation: Y -> A
 */
void tya(struct instruction *this)
{
    cpu.accumulator_ = cpu.y_;

    uint8_t
        acc         = cpu.accumulator_,
        negative    = acc & 0x80,
        zero        = 2 - ((bool) (acc & 0xFF) << 1);

    cpu.status_ &= 0b01111101;
    cpu.status_ |= negative | zero;
}

/**
 * TSX Transfer stack pointer to index X
 * Operation: S -> X
 */
void tsx(struct instruction *this)
{
    cpu.x_ = *cpu.stack_pointer_.lsb_;

    uint8_t
        x           = cpu.x_,
        negative    = x & 0x80,
        zero        = 2 - ((bool) (x & 0xFF) << 1);

    cpu.status_ &= 0b01111101;
    cpu.status_ |= negative | zero;
}

/**
 * TXA Transfer index X to accumulator
 * Operation: X -> A
 */
void txa(struct instruction *this)
{
    cpu.accumulator_ = cpu.x_;

    uint8_t
        acc         = cpu.accumulator_,
        negative    = acc & 0x80,
        zero        = 2 - ((bool) (acc & 0xFF) << 1);

    cpu.status_ &= 0b01111101;
    cpu.status_ |= negative | zero;
}

/**
 * TXS Transfer index X to stack pointer
 * Operation: X -> S
 */
void txs(struct instruction *this)
{
    *cpu.stack_pointer_.lsb_ = cpu.x_;
}

void unofficial_opcode(struct instruction *this)
{

}


