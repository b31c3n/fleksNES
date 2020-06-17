/*
 * instruction.h
 *
 *  Created on: May 5, 2020
 *      Author: David Jonsson
 */

#ifndef SRC_INSTRUCTION_H_
#define SRC_INSTRUCTION_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * Details possible memory operations etc
 * in an instruction
 */
#define READ_DATA       0b0000000000000001
#define WRITE_DATA      0b0000000000000010
#define PUSH_RETURN     0b0000000000000100
#define PULL_RETURN     Ob0000000000001000
#define PUSH_REG        0b0000000000010000
#define PULL_PULL       Ob0000000000100000
#define CHECK_PAGECROSS 0b0000000001000000
#define BRANCH_FLAG     0b0000000010000000

struct instruction
{
    uint8_t
        nr_bytes_,
        nr_cycles_,
        *operand_;
    uint16_t
        flags_;
    void
        (*set_operand)(struct instruction *this);   // Only to be called once, in execution face
    void
        (*execute)(struct instruction *this);

};

void adc(struct instruction *this);     // ADC Add memory to accumulator with carry
void and(struct instruction *this);     // AND Memory with Accumulator
void asl(struct instruction *this);     // ASL Shift Left One Bit (Memory or Accumulator)
void bcc(struct instruction *this);     // BCC Branch on Carry Clear
void bcs(struct instruction *this);     // BCS Branch on Carry Set
void beq(struct instruction *this);     // BEQ Branch on Result Zero
void bit(struct instruction *this);     // BIT Test bits in memory with accumulator
void bmi(struct instruction *this);     // BMI Branch on result minus
void bne(struct instruction *this);     // BNE Branch on result not zero
void bpl(struct instruction *this);     // BPL Branch on result plus
void brk(struct instruction *this);     // BRK Force Break
void bvc(struct instruction *this);     // BVC Branch on overflow clear
void bvs(struct instruction *this);     // BVS Branch on overflow set
void clc(struct instruction *this);     // CLC Clear carry flag
void cld(struct instruction *this);     // CLD Clear decimal mode
void cli(struct instruction *this);     // CLI Clear interrupt disable bit
void clv(struct instruction *this);     // CLV Clear overflow flag
void cmp(struct instruction *this);     // CMP Compare memory and accumulator
void cpx(struct instruction *this);     // CPX Compare Memory and Index X
void cpy(struct instruction *this);     // CPY Compare Memory and Index Y
void dec(struct instruction *this);     // DEC Decrement memory by one
void dex(struct instruction *this);     // DEX Decrement index X by one
void dey(struct instruction *this);     // DEY Decrement index Y by one
void eor(struct instruction *this);     // EOR "Exclusive-Or" memory with accumulator
void inc(struct instruction *this);     // INC Increment memory by one
void inx(struct instruction *this);     // INX Increment Index X by one
void iny(struct instruction *this);     // INY Increment Index Y by one
void jmp(struct instruction *this);     // JMP Jump to new location
void jsr(struct instruction *this);     // JSR Jump to new location saving return address
void lda(struct instruction *this);     // LDA Load accumulator with memory
void ldx(struct instruction *this);     // LDX Load index X with memory
void ldy(struct instruction *this);     // LDY Load index Y with memory
void lsr(struct instruction *this);     // LSR Shift right one bit (memory or accumulator)
void nop(struct instruction *this);     // NOP No operation
void ora(struct instruction *this);     // ORA "OR" memory with accumulator
void pha(struct instruction *this);     // PHA Push accumulator on stack
void php(struct instruction *this);     // PHP Push processor status on stack
void pla(struct instruction *this);     // PLA Pull accumulator from stack
void plp(struct instruction *this);     // PLP Pull processor status from stack
void rol(struct instruction *this);     // ROL Rotate one bit left (memory or accumulator)
void ror(struct instruction *this);     // ROR Rotate one bit right (memory or accumulator)
void rti(struct instruction *this);     // RTI Return from interrupt
void rts(struct instruction *this);     // RTS Return from subroutine
void sbc(struct instruction *this);     // SBC Subtract memory from accumulator with borrow
void sec(struct instruction *this);     // SEC Set carry flag
void sed(struct instruction *this);     // SED Set decimal mode
void sei(struct instruction *this);     // SEI Set interrupt disable status
void sta(struct instruction *this);     // STA Store accumulator in memory
void stx(struct instruction *this);     // STX Store index X in memory
void sty(struct instruction *this);     // STY Store index Y in memory
void tax(struct instruction *this);     // TAX Transfer accumulator to index X
void tay(struct instruction *this);     // TAY Transfer accumulator to index Y
void tya(struct instruction *this);     // TYA Transfer index Y to accumulator
void tsx(struct instruction *this);     // TSX Transfer stack pointer to index X
void txa(struct instruction *this);     // TXA Transfer index X to accumulator
void txs(struct instruction *this);     // TXS Transfer index X to stack pointer

void unofficial_opcode(struct instruction *this);


#endif /* SRC_INSTRUCTION_H_ */
