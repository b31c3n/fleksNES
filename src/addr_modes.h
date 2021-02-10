/*
 * addr_modes.h
 *
 *  Created on: Jun 8, 2020
 *      Author: David Jonsson
 */

#ifndef SRC_ADDR_MODES_H_
#define SRC_ADDR_MODES_H_

#include "instruction.h"
#include "cpu.h"

void accumulator(struct instruction *_this);
void absolute(struct instruction *_this);
void absolute_x(struct instruction *_this);
void absolute_y(struct instruction *_this);
void absolute_indirect(struct instruction *_this);
void immediate(struct instruction *_this);
void implied(struct instruction *_this);
void indirect(struct instruction *_this);
void indexed_indirect(struct instruction *_this);
void indirect_indexed(struct instruction *_this);
void relative(struct instruction *_this);
void zero_page(struct instruction *_this);
void zero_page_x(struct instruction *_this);
void zero_page_y(struct instruction *_this);

#endif /* SRC_ADDR_MODES_H_ */
