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
#include <stdio.h>

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
    	ver_mirror_;
    FILE
        *file_;

} extern header;


#endif /* SRC_MAPPER_H_ */
