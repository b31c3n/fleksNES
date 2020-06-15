/*
 * stack.c
 *
 *  Created on: Jun 11, 2020
 *      Author: David Jonsson
 */

#include "16_bit.h"

void _16_bit_init(struct _16_bit *this)
{
    this->lsb_ = &this->word_;
    this->msb_ = ((uint8_t *) (&this->word_)) + 1;
}
