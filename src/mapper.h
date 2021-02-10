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

#define MIRROR_1SCREEN_LO 0
#define MIRROR_1SRCEEN_HI 1
#define MIRROR_VERTICAL   2
#define MIRROR_HORIZONTAL 3

void mapper_init(char *file_name, uint8_t *mlogic_mem);
void mapper_destroy();

struct ines_header
{
    char
        constant_[4];
    uint8_t
        prgrom_size_,
        chrrom_size_,
        flags_[5],
        padding_[5],
        mapper_ctr_mirror_;
    uint16_t
        mapper_nr_;
    uint8_t
    	ver_mirror_;
    FILE
        *file_;

};

extern void
    (*mapper_load_state)(FILE *),
    (*mapper_save_state)(FILE *);

#endif /* SRC_MAPPER_H_ */
