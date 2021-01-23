/*
 * mapper.c
 *
 *  Created on: Jun 4, 2020
 *      Author: David Jonsson
 */

#include "mapper.h"
#include "bus.h"
#include "cpu.h"
#include "mapper000.h"

/**
 * 16 byte header
 */
struct ines_header
	header =
	        {
	                .mapper_ctr_mirror_ = 0,
	        };


uint8_t
    *prg_rom,
    *prg_ram,
    *chr_rom;


void mapper_init(char *file_name)
{
    header.file_ = fopen(file_name, "r");
    if(!header.file_) exit(1);

    for(uint8_t *p = header.constant_, i = 0; i < 4; ++i)
        p[i] = fgetc(header.file_);

    header.prgrom_size_ = fgetc(header.file_);
    header.chrrom_size_ = fgetc(header.file_);

    for(uint8_t *p = &header.flags_, i = 0; i < 5; ++i)
        p[i] = fgetc(header.file_);

    header.ver_mirror_ = header.flags_[0] & 0b01;

    /**
     * Check if bit 2-3 in byte 7 is equal to 2, then do INes 2.0 stuff, otherwise do the stuff below
     */

    for(uint8_t *p = &header.padding_, i = 0; i < 5; ++i)
        p[i] = fgetc(header.file_);

    header.mapper_nr_ = ((header.flags_[0] & 0xF0) >> 4) | (header.flags_[1] & 0xF0);

    if(header.mapper_nr_ == 0) mapper000_init();
    else if(header.mapper_nr_ == 1) mapper001_init();
    else exit(1);

    fclose(header.file_);
}

void mapper_destroy()
{
    free(prg_rom);
    free(chr_rom);
}

static void temp(FILE *fp) {}

void
   (*mapper_load_state)(FILE *) = temp,
   (*mapper_save_state)(FILE *) = temp;


