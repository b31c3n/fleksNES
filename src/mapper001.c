/*
 * mapper000.c
 *
 *  Created on: Jan 16, 2021
 *      Author: David Jonsson
 */

#include <stdint.h>
#include <stdbool.h>

#include "mapper000.h"
#include "mapper.h"
#include "bus.h"
#include "cpu.h"

static size_t
    prg_size,
    chr_size;

static uint8_t
    shift_reg   = 0,
    shift_bits  = 0,
    prg_bank    = 0,
    chr_bank0   = 0,
    chr_bank1   = 0,
    prg_mode    = 3,
    chr_mode    = 1,
    internal_regs[4] = {0};


#define MAPPER001_CTRL  0
#define MAPPER001_CHR0  1
#define MAPPER001_CHR1  2
#define MAPPER001_PRG   3
#define MAPPER001_PRG_MODE0 0
#define MAPPER001_PRG_MODE1 1
#define MAPPER001_PRG_MODE2 2
#define MAPPER001_PRG_MODE3 3
#define MAPPER001_CHR_MODE0 0
#define MAPPER001_CHR_MODE1 1
#define MAPPER001_CHR_MODE2 2
#define MAPPER001_CHR_MODE3 3

static uint16_t
    reg_select  = 0,
    prg0_offset = 0,
    prg1_offset = 0x4000,
    chr0_offset = 0,
    chr1_offset = 0x1000;

static void write_prgram()
{
    uint16_t
        address = cpu_bus.address_ & 0x1FFF;
    prg_ram[address] = cpu_bus.data_;
}

static void read_prgram()
{
    uint16_t
        address = cpu_bus.address_  & 0x1FFF;
    cpu_bus.data_ = prg_ram[address];
}

static void write_prgrom_8000()
{
    shift_reg |= (cpu_bus.data_ & 0x1) << shift_bits;
    ++shift_bits;
    bool
        reset = 1 -  (bool) shift_bits & 5;

    if(reset)
    {
        header.mapper_ctr_mirror_ = shift_reg & 0b11;
        prg_mode = shift_reg & 0b01100,
        chr_mode = shift_reg & 0b10000;

        if(!(prg_mode & 0b10))
        {
            prg0_offset = 0x0000,
            prg1_offset = 0x4000;
        }
        else if(prg_mode & 0b01)
        {
            prg0_offset = 0x0000,
            prg1_offset = 0x4000;
        }
        else
        {
            prg0_offset = 0x0000,
            prg1_offset = 0x4000;
        }
        if(chr_mode)
        {

        }
        shift_reg  = 0;
        shift_bits = 0;
    }
}

static void write_prgrom_A000()
{
    shift_reg |= (cpu_bus.data_ & 0x1) << shift_bits;
    ++shift_bits;
    bool
        reset = 1 -  (bool) shift_bits & 5;

    if(reset)
    {
        chr_bank0 = internal_regs[MAPPER001_CHR0];
        if(chr_mode)
            chr0_offset = 0x0000 + 0x1000 * chr_bank0;
        else
        {
            chr0_offset = 0x0000 + 0x1000 * chr_bank0;
            chr1_offset = 0x1000 + 0x1000 * chr_bank0;
        }

        shift_reg  = 0;
        shift_bits = 0;
    }
}

static void write_prgrom_C000()
{
    shift_reg |= (cpu_bus.data_ & 0x1) << shift_bits;
    ++shift_bits;
    bool
        reset = 1 -  (bool) shift_bits & 5;

    if(reset)
    {
        chr_bank1 = internal_regs[MAPPER001_CHR0];
        if(chr_mode)
            chr1_offset = 0x1000 + 0x1000 * chr_bank1;

        shift_reg  = 0;
        shift_bits = 0;

    }
}

static void write_prgrom_E000()
{
    shift_reg |= (cpu_bus.data_ & 0x1) << shift_bits;
    ++shift_bits;
    bool
        reset = 1 -  (bool) shift_bits & 5;

    if(reset)
    {
        prg_bank = internal_regs[MAPPER001_PRG]  & 0b01110;
        if(!(prg_mode & 0b10))
        {
            prg0_offset = 0x0000 + 0x8000 * prg_bank,
            prg1_offset = 0x4000 + 0x8000 * prg_bank;
        }
        else if(prg_mode & 0b01)
        {
            prg0_offset = 0x0000 + 0x4000 * prg_bank;
            prg1_offset = 0x4000;
        }
        else
        {
            prg0_offset = 0x0000,
            prg1_offset = 0x4000 + 0x4000 * prg_bank;
        }
        shift_reg  = 0;
        shift_bits = 0;

    }
}

static void read_prgrom_8000()
{
    uint16_t
        address = (cpu_bus.address_ & 0x3FFF) + prg0_offset;
    cpu_bus.data_ = prg_rom[address];
}

static void read_prgrom_C000()
{
    uint16_t
        address = (cpu_bus.address_ & 0x3FFF) + prg1_offset;
    cpu_bus.data_ = prg_rom[address];
}

static void write_chrrom(){}

static void read_chrrom0000()
{
    uint16_t
        address = ppu_bus.address_ + chr0_offset;
    ppu_bus.data_ = chr_rom[address];
}

static void read_chrrom1000()
{
    uint16_t
        address = ppu_bus.address_ & 0x0fff + chr1_offset;
    ppu_bus.data_ = chr_rom[address];
}

void mapper001_init()
{
    cpu_bus.read[3] = read_prgram,
    cpu_bus.read[4] = read_prgrom_8000,
    cpu_bus.read[5] = read_prgrom_8000,
    cpu_bus.read[6] = read_prgrom_C000,
    cpu_bus.read[7] = read_prgrom_C000,

    cpu_bus.write[3] = write_prgram,
    cpu_bus.write[4] = write_prgrom_8000,
    cpu_bus.write[5] = write_prgrom_A000,
    cpu_bus.write[6] = write_prgrom_C000,
    cpu_bus.write[7] = write_prgrom_E000;

    ppu_bus.read[0] = read_chrrom0000,
    ppu_bus.read[1] = read_chrrom1000,
    ppu_bus.write[0] = write_chrrom;
    ppu_bus.write[1] = write_chrrom;

    prg_size = 0x4000 * header.prgrom_size_,
    chr_size = 0x2000 * header.chrrom_size_;

    prg_rom = malloc(prg_size);
    chr_rom = malloc(chr_size);

    /**
     * Need to fix program ram according to mapper, 8k for now to get nestest working
     */
    prg_ram = malloc(0x1FFF);

    fread(prg_rom, prg_size, 1, header.file_);
    fread(chr_rom, chr_size, 1, header.file_);

    *cpu.program_counter_.lsb_ = prg_rom[prg_size - 4];
    *cpu.program_counter_.msb_ = prg_rom[prg_size - 3];
}

