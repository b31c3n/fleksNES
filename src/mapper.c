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

/**
 * 16 byte header
 */
struct ines_header
{
    char
        constant_[4];
    uint8_t
        prgrom_size_,
        chrrom_size_,
        flags_[5],
        padding_[5];
    uint16_t
        mapper_nr_;
} header;


struct peripheral
    cpu_peripheral_prgrom,
    cpu_peripheral_prgram,
    ppu_peripheral_chrrom;

void mapper000_prgram_update(struct peripheral *this)
{

}
void mapper000_prgram_write(struct peripheral *this)
{
    uint16_t
        address = this->bus_->address_ & this->mirror_mask_ - this->address_min_;
    this->memory_[address] = this->bus_->data_;
}
void mapper000_prgram_read(struct peripheral *this)
{
    uint16_t
        address = this->bus_->address_ & this->mirror_mask_ - this->address_min_;
    this->bus_->data_ = this->memory_[address];
}

void mapper000_prgrom_update(struct peripheral *this)
{

}
void mapper000_prgrom_write(struct peripheral *this)
{
//    uint16_t
//        address = this->bus_->address_ & this->mirror_mask_ - this->address_min_;
//    this->memory_[address] = this->bus_->data_;
}
void mapper000_prgrom_read(struct peripheral *this)
{
    uint16_t
        address = this->bus_->address_ & this->mirror_mask_ - this->address_min_;
    this->bus_->data_ = this->memory_[address];
}


void mapper000_chrrom_update(struct peripheral *this)
{

}
void mapper000_chrrom_write(struct peripheral *this)
{

}
void mapper000_chrrom_read(struct peripheral *this)
{

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
        cpu_peripheral_prgram.update = mapper000_prgram_update;
        cpu_peripheral_prgram.write = mapper000_prgram_write;
        cpu_peripheral_prgram.read = mapper000_prgram_read;
        cpu_peripheral_prgram.bus_ = &cpu_bus;
        cpu_peripheral_prgram.irq_line_ = 0;
        cpu_peripheral_prgram.memory_ = prg_ram;

        cpu_peripheral_prgrom.address_min_ = 0x8000;
        cpu_peripheral_prgrom.address_max_ = 0xFFFF;
        cpu_peripheral_prgrom.mirror_mask_ = 0x8000 + prg_size - 1;
        cpu_peripheral_prgrom.update = mapper000_prgrom_update;
        cpu_peripheral_prgrom.write = mapper000_prgrom_write;
        cpu_peripheral_prgrom.read = mapper000_prgrom_read;
        cpu_peripheral_prgrom.bus_ = &cpu_bus;
        cpu_peripheral_prgrom.irq_line_ = 0;
        cpu_peripheral_prgrom.memory_ = prg_rom;

        ppu_peripheral_chrrom.address_min_ = 0x0000;
        ppu_peripheral_chrrom.address_max_ = 0x1FFF;
        ppu_peripheral_chrrom.mirror_mask_ = 0x1FFF;
        ppu_peripheral_chrrom.update = mapper000_chrrom_update;
        ppu_peripheral_chrrom.write = mapper000_chrrom_write;
        ppu_peripheral_chrrom.read = mapper000_chrrom_read;
        ppu_peripheral_chrrom.bus_ = &ppu_bus;
        ppu_peripheral_chrrom.irq_line_ = 0;
        ppu_peripheral_chrrom.memory_ = chr_rom;
    }
    else exit(1);
}

void mapper_destroy()
{
    free(prg_rom);
    free(chr_rom);
}


