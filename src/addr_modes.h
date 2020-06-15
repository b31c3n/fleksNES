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

void accumulator(struct instruction *this);
void absolute(struct instruction *this);
void absolute_x(struct instruction *this);
void absolute_y(struct instruction *this);
void absolute_indirect(struct instruction *this);
void immediate(struct instruction *this);
void implied(struct instruction *this);
void indirect(struct instruction *this);
void indexed_indirect(struct instruction *this);
void indirect_indexed(struct instruction *this);
void relative(struct instruction *this);
void zero_page(struct instruction *this);
void zero_page_x(struct instruction *this);
void zero_page_y(struct instruction *this);

#endif /* SRC_ADDR_MODES_H_ */
