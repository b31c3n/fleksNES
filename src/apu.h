/*
 * apu.h
 *
 *  Created on: Nov 2, 2020
 *      Author: David Jonsson
 */

#ifndef SRC_APU_H_
#define SRC_APU_H_

#include <stdint.h>

extern uint8_t
    controller_buffer;

void apu_write();
void apu_read();


#endif /* SRC_APU_H_ */
