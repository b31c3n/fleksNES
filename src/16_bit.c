/*
 * stack.c
 *
 *  Created on: Jun 11, 2020
 *      Author: David Jonsson
 */

#include "16_bit.h"

void _16_bit_init(struct _16_bit *_this)
{
    _this->word_ = 0;
    _this->lsb_ = &_this->word_;
    _this->msb_ = ((uint8_t *) (&_this->word_)) + 1;
}
