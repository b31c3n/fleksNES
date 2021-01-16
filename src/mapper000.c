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

uint8_t
    *prg_rom,
    *prg_ram,
    *chr_rom;

void mapper000_write_prgram()
{
    uint16_t
        address = cpu_bus.address_ & 0x1FFF;
    prg_ram[address] = cpu_bus.data_;
}

void mapper000_read_prgram()
{
    uint16_t
        address = cpu_bus.address_  & 0x1FFF;
    cpu_bus.data_ = prg_ram[address];
}

void mapper000_write_prgrom(){}

void mapper000_read_prgrom()
{
    uint16_t
        address = cpu_bus.address_ & (0x4000 * header.prgrom_size_ - 1);
    cpu_bus.data_ = prg_rom[address];
}

void mapper000_write_chrrom(){}

void mapper000_read_chrrom()
{
    uint16_t
        address = ppu_bus.address_;
    ppu_bus.data_ = chr_rom[address];
}

void mapper000_init()
{
    cpu_bus.read[3] = mapper000_read_prgram,
    cpu_bus.read[4] = mapper000_read_prgrom,
    cpu_bus.read[5] = mapper000_read_prgrom,
    cpu_bus.read[6] = mapper000_read_prgrom,
    cpu_bus.read[7] = mapper000_read_prgrom,

    cpu_bus.write[3] = mapper000_write_prgram,
    cpu_bus.write[4] = mapper000_write_prgrom,
    cpu_bus.write[5] = mapper000_write_prgrom,
    cpu_bus.write[6] = mapper000_write_prgrom,
    cpu_bus.write[7] = mapper000_write_prgrom;

    ppu_bus.read[0] = mapper000_read_chrrom,
    ppu_bus.write[0] = mapper000_write_chrrom;

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

