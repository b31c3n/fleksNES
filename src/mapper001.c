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
#include "nametable.h"
#include "bus.h"
#include "cpu.h"

static uint8_t
    nametable_mem[2][1024],
    palette_mem[0x20];

static size_t
    prg_size,
    chr_size;

static uint8_t
    shift_reg   = 0,
    shift_bits  = 0,
    ctrl_reg    = 0,
    prg_bank    = 0,
    chr_bank0   = 0,
    chr_bank1   = 1;
//    prg_mode    = 3,
//    chr_mode    = 0;


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
    reg_select  = 0;
    prg0_offset = 0,
    prg1_offset = 0x4000,
    chr0_offset = 0x1000 * 0,
    chr1_offset = 0x1000 * 1;

static bool
    reset = 0,
    write = 1;
static void shift()
{
    shift_reg |= (cpu_bus.data_ & 0x1) << shift_bits;
    ++shift_bits;

    reset = shift_bits == 5;
    if(cpu_bus.data_ & 0x80)
    {
        shift_reg       = 0,
        shift_bits      = 0,
        reset           = 0,
        ctrl_reg        |= 0x0C,
        prg1_offset     = 0x4000 * (header.prgrom_size_ - 1);
    }
}

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
    shift();
    if(reset)
    {
        ctrl_reg = shift_reg & 0b11111;
        uint8_t prg_mode = shift_reg & 0b01100,
        chr_mode = shift_reg & 0b10000;

        prg_bank = shift_reg & 0b01111;
        if(!(ctrl_reg & 0b10))
        {
            prg0_offset = 0x8000 * (prg_bank & 0b01110),
            prg1_offset = 0x4000 + 0x8000 * (prg_bank & 0b01110);
        }
        else if(ctrl_reg & 0b01)
        {
            prg0_offset = 0x4000 * prg_bank;
            prg1_offset = 0x4000 * (header.prgrom_size_ - 1);
        }
        else
        {
            prg0_offset = 0x0000,
            prg1_offset = 0x4000 * prg_bank;
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
    shift();
    if(reset)
    {
        chr_bank0 = shift_reg;
        if(ctrl_reg & 0b10000)
            chr0_offset = 0x0000 + 0x1000 * chr_bank0;
        else
        {
            chr0_offset = 0x2000 * (chr_bank0 & 0b11110);
            chr1_offset = 0x1000 + 0x2000 * (chr_bank0 & 0b11110);
        }

        shift_reg  = 0;
        shift_bits = 0;
    }
}

static void write_prgrom_C000()
{
    shift();
    if(reset)
    {
        chr_bank1 = shift_reg;
        if(ctrl_reg & 0b10000)
            chr1_offset = 0x1000 * chr_bank1;

        shift_reg  = 0;
        shift_bits = 0;
    }
}

static void write_prgrom_E000()
{
    shift();
    if(reset)
    {
        prg_bank = shift_reg & 0b01111;
        if(!(ctrl_reg & 0b10))
        {
            prg0_offset = 0x8000 * (prg_bank & 0b01110),
            prg1_offset = 0x4000 + 0x8000 * (prg_bank & 0b01110);
        }
        else if(ctrl_reg & 0b01)
        {
            prg0_offset = 0x4000 * prg_bank;
            prg1_offset = 0x4000 * (header.prgrom_size_ - 1);
        }
        else
        {
            prg0_offset = 0x0000,
            prg1_offset = 0x4000 * prg_bank;
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
        address = (ppu_bus.address_ & 0x0fff) + chr1_offset;
    ppu_bus.data_ = chr_rom[address];
}

static void ntable_read()
{
    if(ppu_bus.address_ >= 0x3F00)
    {
        uint16_t
            address = ppu_bus.address_ & 0x1F;
        address -=  0x0010 * !((bool)(address & 0b00011)) * ((bool) (address & 0b10000));
        ppu_bus.data_ = palette_mem[address];
    }
    else
    {
        uint16_t
            address = ppu_bus.address_;
        bool
            ntable_id;

        address &= 0x0FFF;
        ntable_id = 0;
        ntable_id += header.ver_mirror_ * ((bool) (address & 0x0400));
        ntable_id += (1 - header.ver_mirror_) * ((bool) (address & 0x0800));
        ntable_id = ctrl_reg & 0b1;

        address &= 0x03FF;
        uint8_t
            data = nametable_mem[ntable_id][address];
        ppu_bus.data_ = data;
    }
}

static void ntable_write()
{
    if(ppu_bus.address_ >= 0x3F00)
    {
        uint16_t
            address = ppu_bus.address_ & 0x1F;
        address -=  0x0010 * !((bool)(address & 0b00011)) * ((bool) (address & 0b10000));
        palette_mem[address] = ppu_bus.data_;
    }
    else
    {
        uint16_t
            address = ppu_bus.address_;
        bool
            ntable_id;

        address &= 0x0FFF;
        ntable_id = 0;
        ntable_id += header.ver_mirror_ * ((bool) (address & 0x0400));
        ntable_id += (1 - header.ver_mirror_) * ((bool) (address & 0x0800));
        ntable_id = ctrl_reg & 0b1;

        address &= 0x03FF;
        uint8_t
            data = ppu_bus.data_;
        nametable_mem[ntable_id][address] = data;
    }
}

void mapper001_init()
{
    ntable_mem = nametable_mem;
    pal_mem = palette_mem;

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
    ppu_bus.read[2] = ntable_read,
    ppu_bus.read[3] = ntable_read,
    ppu_bus.write[0] = write_chrrom,
    ppu_bus.write[1] = write_chrrom,
    ppu_bus.write[2] = ntable_write,
    ppu_bus.write[3] = ntable_write;

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

