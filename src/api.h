/*
 * api.h
 *
 *  Created on: Feb 9, 2021
 *      Author: David Jonsson
 */

#ifndef SRC_API_H_
#define SRC_API_H_

void fleks_init
(
    char    *game,
    char *mem,
    char *pixels,
    char *ram_mem
);
void fleks_step(char *mem);
void fleks_destroy(char *mem);


#endif /* SRC_API_H_ */
