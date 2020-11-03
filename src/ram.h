/*
 * ram.h
 *
 *  Created on: Nov 3, 2020
 *      Author: David Jonsson
 */

#ifndef SRC_RAM_H_
#define SRC_RAM_H_

extern uint8_t
    ram[0x7FF];

void ram_write();
void ram_read();

#endif /* SRC_RAM_H_ */
