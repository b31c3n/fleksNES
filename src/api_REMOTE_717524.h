/*
 * api.h
 *
 *  Created on: Feb 9, 2021
 *      Author: David Jonsson
 */

#ifndef SRC_API_H_
#define SRC_API_H_

#include <stdint.h>

void fleks_init
(
    char    *game,
    uint8_t *mem,
    uint8_t *pixels,
    uint8_t *ram_mem
);
void fleks_step(uint8_t *mem);
void fleks_destroy(uint8_t *mem);

#endif /* SRC_API_H_ */
