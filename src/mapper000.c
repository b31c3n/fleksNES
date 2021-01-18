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

static void write_prgrom(){}

static void read_prgrom()
{
    uint16_t
        address = cpu_bus.address_ & (0x4000 * header.prgrom_size_ - 1);
    cpu_bus.data_ = prg_rom[address];
}

static void write_chrrom(){}

static void read_chrrom()
{
    uint16_t
        address = ppu_bus.address_;
    ppu_bus.data_ = chr_rom[address];
}

void mapper000_init()
{
    cpu_bus.read[3] = read_prgram,
    cpu_bus.read[4] = read_prgrom,
    cpu_bus.read[5] = read_prgrom,
    cpu_bus.read[6] = read_prgrom,
    cpu_bus.read[7] = read_prgrom,

    cpu_bus.write[3] = write_prgram,
    cpu_bus.write[4] = write_prgrom,
    cpu_bus.write[5] = write_prgrom,
    cpu_bus.write[6] = write_prgrom,
    cpu_bus.write[7] = write_prgrom;

    ppu_bus.read[0] = read_chrrom,
    ppu_bus.read[1] = read_chrrom,
    ppu_bus.write[0] = write_chrrom,
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

