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
        (*set_operand)(struct instruction *_this);   // Only to be called once, in execution face
    void
        (*execute)(struct instruction *_this);

};

void adc(struct instruction *_this);     // ADC Add memory to accumulator with carry
void and(struct instruction *_this);     // AND Memory with Accumulator
void asl(struct instruction *_this);     // ASL Shift Left One Bit (Memory or Accumulator)
void bcc(struct instruction *_this);     // BCC Branch on Carry Clear
void bcs(struct instruction *_this);     // BCS Branch on Carry Set
void beq(struct instruction *_this);     // BEQ Branch on Result Zero
void bit(struct instruction *_this);     // BIT Test bits in memory with accumulator
void bmi(struct instruction *_this);     // BMI Branch on result minus
void bne(struct instruction *_this);     // BNE Branch on result not zero
void bpl(struct instruction *_this);     // BPL Branch on result plus
void brk(struct instruction *_this);     // BRK Force Break
void bvc(struct instruction *_this);     // BVC Branch on overflow clear
void bvs(struct instruction *_this);     // BVS Branch on overflow set
void clc(struct instruction *_this);     // CLC Clear carry flag
void cld(struct instruction *_this);     // CLD Clear decimal mode
void cli(struct instruction *_this);     // CLI Clear interrupt disable bit
void clv(struct instruction *_this);     // CLV Clear overflow flag
void cmp(struct instruction *_this);     // CMP Compare memory and accumulator
void cpx(struct instruction *_this);     // CPX Compare Memory and Index X
void cpy(struct instruction *_this);     // CPY Compare Memory and Index Y
void dec(struct instruction *_this);     // DEC Decrement memory by one
void dex(struct instruction *_this);     // DEX Decrement index X by one
void dey(struct instruction *_this);     // DEY Decrement index Y by one
void eor(struct instruction *_this);     // EOR "Exclusive-Or" memory with accumulator
void inc(struct instruction *_this);     // INC Increment memory by one
void inx(struct instruction *_this);     // INX Increment Index X by one
void iny(struct instruction *_this);     // INY Increment Index Y by one
void jmp(struct instruction *_this);     // JMP Jump to new location
void jsr(struct instruction *_this);     // JSR Jump to new location saving return address
void lda(struct instruction *_this);     // LDA Load accumulator with memory
void ldx(struct instruction *_this);     // LDX Load index X with memory
void ldy(struct instruction *_this);     // LDY Load index Y with memory
void lsr(struct instruction *_this);     // LSR Shift right one bit (memory or accumulator)
void nop(struct instruction *_this);     // NOP No operation
void ora(struct instruction *_this);     // ORA "OR" memory with accumulator
void pha(struct instruction *_this);     // PHA Push accumulator on stack
void php(struct instruction *_this);     // PHP Push processor status on stack
void pla(struct instruction *_this);     // PLA Pull accumulator from stack
void plp(struct instruction *_this);     // PLP Pull processor status from stack
void rol(struct instruction *_this);     // ROL Rotate one bit left (memory or accumulator)
void ror(struct instruction *_this);     // ROR Rotate one bit right (memory or accumulator)
void rti(struct instruction *_this);     // RTI Return from interrupt
void rts(struct instruction *_this);     // RTS Return from subroutine
void sbc(struct instruction *_this);     // SBC Subtract memory from accumulator with borrow
void sec(struct instruction *_this);     // SEC Set carry flag
void sed(struct instruction *_this);     // SED Set decimal mode
void sei(struct instruction *_this);     // SEI Set interrupt disable status
void sta(struct instruction *_this);     // STA Store accumulator in memory
void stx(struct instruction *_this);     // STX Store index X in memory
void sty(struct instruction *_this);     // STY Store index Y in memory
void tax(struct instruction *_this);     // TAX Transfer accumulator to index X
void tay(struct instruction *_this);     // TAY Transfer accumulator to index Y
void tya(struct instruction *_this);     // TYA Transfer index Y to accumulator
void tsx(struct instruction *_this);     // TSX Transfer stack pointer to index X
void txa(struct instruction *_this);     // TXA Transfer index X to accumulator
void txs(struct instruction *_this);     // TXS Transfer index X to stack pointer

void unofficial_opcode(struct instruction *_this);


#endif /* SRC_INSTRUCTION_H_ */
