/*
 * mapper.c
 *
 *  Created on: Jun 4, 2020
 *      Author: David Jonsson
 */

#include <stdio.h>

#include "mapper.h"
#include "bus.h"
#include "peripheral.h"
#include "cpu.h"

/**
 * 16 byte header
 */
struct ines_header
	header;


struct peripheral
    cpu_peripheral_prgrom,
    cpu_peripheral_prgram,
    ppu_peripheral_chrrom;


void mapper_write_cpu_side()
{
    uint16_t
        t_address = cpu_bus.address_;
    struct peripheral
        *this = ((uint64_t) &cpu_peripheral_prgram) * !((bool) (t_address & 0x8000)) +
                ((uint64_t) &cpu_peripheral_prgrom) * ((bool) (t_address & 0x8000));
    uint16_t
        address = this->bus_->address_ & this->mirror_mask_ - this->address_min_;
    this->memory_[address] = this->bus_->data_;
}
void mapper_read_cpu_side()
{
    uint16_t
        t_address = cpu_bus.address_;
    struct peripheral
        *this = ((uint64_t) &cpu_peripheral_prgram) * !((bool) (t_address & 0x8000)) +
                ((uint64_t) &cpu_peripheral_prgrom) * ((bool) (t_address & 0x8000));
    uint16_t
        address = this->bus_->address_ & this->mirror_mask_ - this->address_min_;
    this->bus_->data_ = this->memory_[address];
}

void mapper_write_ppu_side()
{

}
void mapper_read_ppu_side()
{
    struct peripheral
        *this = &ppu_peripheral_chrrom;
    uint16_t
        address = this->bus_->address_ & this->mirror_mask_ - this->address_min_;
    this->bus_->data_ = this->memory_[address];
}

static uint8_t
    *prg_rom,
    *prg_ram,
    *chr_rom;

void mapper_init(char *file_name)
{
    FILE *file_p = fopen(file_name, "r");
    if(!file_p) exit(1);

    for(uint8_t *p = header.constant_, i = 0; i < 4; ++i)
        p[i] = fgetc(file_p);

    header.prgrom_size_ = fgetc(file_p);
    header.chrrom_size_ = fgetc(file_p);

    for(uint8_t *p = &header.flags_, i = 0; i < 5; ++i)
        p[i] = fgetc(file_p);

    header.hor_mirror = header.flags_[0] & 0b01;
    header.ver_mirror = header.flags_[0] & 0b10;
    /**
     * Check if bit 2-3 in byte 7 is equal to 2, then do INes 2.0 stuff, otherwise do the stuff below
     */

    for(uint8_t *p = &header.padding_, i = 0; i < 5; ++i)
        p[i] = fgetc(file_p);

    header.mapper_nr_ = ((header.flags_[0] & 0xF0) >> 4) | (header.flags_[1] & 0xF0);

    size_t
        prg_size = 0x4000 * header.prgrom_size_,
        chr_size = 0x2000 * header.chrrom_size_;

    prg_rom = malloc(prg_size);
    chr_rom = malloc(chr_size);


    /**
     * Need to fix program ram according to mapper, 8k for now to get nestest working
     */
    prg_ram = malloc(0x1FFF);

    if(header.mapper_nr_ == 0)
    {
        fread(prg_rom, prg_size, 1, file_p);
        fread(chr_rom, chr_size, 1, file_p);

        cpu_peripheral_prgram.address_min_ = 0x6000;
        cpu_peripheral_prgram.address_max_ = 0x7FFF;
        cpu_peripheral_prgram.mirror_mask_ = 0x7FFF;
        cpu_peripheral_prgram.bus_ = &cpu_bus;
        cpu_peripheral_prgram.irq_line_ = 0;
        cpu_peripheral_prgram.memory_ = prg_ram;

        cpu_peripheral_prgrom.address_min_ = 0x8000;
        cpu_peripheral_prgrom.address_max_ = 0xFFFF;
        cpu_peripheral_prgrom.mirror_mask_ = 0x8000 + prg_size - 1;
        cpu_peripheral_prgrom.bus_ = &cpu_bus;
        cpu_peripheral_prgrom.irq_line_ = 0;
        cpu_peripheral_prgrom.memory_ = prg_rom;

        ppu_peripheral_chrrom.address_min_ = 0x0000;
        ppu_peripheral_chrrom.address_max_ = 0x1FFF;
        ppu_peripheral_chrrom.mirror_mask_ = 0x1FFF;
        ppu_peripheral_chrrom.bus_ = &ppu_bus;
        ppu_peripheral_chrrom.irq_line_ = 0;
        ppu_peripheral_chrrom.memory_ = chr_rom;

        *cpu.program_counter_.lsb_ = prg_rom[0x3FFC];
        *cpu.program_counter_.msb_ = prg_rom[0x3FFD];
    }
    else exit(1);
}

void mapper_destroy()
{
    free(prg_rom);
    free(chr_rom);
}


