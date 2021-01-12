/*
 * mapper.h
 *
 *  Created on: Jun 4, 2020
 *      Author: David Jonsson
 */

#ifndef SRC_MAPPER_H_
#define SRC_MAPPER_H_

#include <stdint.h>
#include <stdbool.h>

extern uint8_t
    *prg_rom,
    *prg_ram,
    *chr_rom;

void mapper_init(char *file_name);
void mapper_destroy();

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
    bool
    	ver_mirror;

} extern header;

void mapper_read_cpu_side();
void mapper_write_cpu_side();
void mapper_read_ppu_side();
void mapper_write_ppu_side();

#endif /* SRC_MAPPER_H_ */
