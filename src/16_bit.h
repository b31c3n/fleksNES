/*
 * stack.h
 *
 *  Created on: Jun 11, 2020
 *      Author: David Jonsson
 */

#ifndef SRC_16_BIT_H_
#define SRC_16_BIT_H_

#include <stdint.h>

/**
 * Use pointer to access the stack on the buss
 * Important! Only modify lsb, msb is "hardcoded" to 0x01 on 650x
 */
struct _16_bit
{
    uint16_t
        word_;
    uint8_t
        *msb_,
        *lsb_;
};

void _16_bit_init(struct _16_bit *_this);

#endif /* SRC_16_BIT_H_ */
