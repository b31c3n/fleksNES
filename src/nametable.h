/*
 * nametable.h
 *
 *  Created on: Nov 3, 2020
 *      Author: David Jonsson
 */

#ifndef SRC_NAMETABLE_H_
#define SRC_NAMETABLE_H_

extern uint8_t
    nametable_mem[2][1024];

void ntable_read();
void ntable_write();

#endif /* SRC_NAMETABLE_H_ */
