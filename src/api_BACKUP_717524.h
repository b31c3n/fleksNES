/*
 * api.h
 *
 *  Created on: Feb 9, 2021
 *      Author: David Jonsson
 */

#ifndef SRC_API_H_
#define SRC_API_H_

<<<<<<< HEAD
=======
#include <stdint.h>
>>>>>>> 3efba92c3b3e947a65011cae703266e9293b8943

void fleks_init
(
    char    *game,
<<<<<<< HEAD
    char *mem,
    char *pixels,
    char *ram_mem
);
void fleks_step(char *mem);
void fleks_destroy(char *mem);
=======
    uint8_t *mem,
    uint8_t *pixels,
    uint8_t *ram_mem
);
void fleks_step(uint8_t *mem);
void fleks_destroy(uint8_t *mem);
>>>>>>> 3efba92c3b3e947a65011cae703266e9293b8943

#endif /* SRC_API_H_ */
