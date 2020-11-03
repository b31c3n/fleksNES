/*
 * clock.h
 *
 *  Created on: Aug 21, 2020
 *      Author: David Jonsson
 */

#ifndef SRC_CLOCK_H_
#define SRC_CLOCK_H_

#include <semaphore.h>
#include <stdint.h>
#include <time.h>

extern struct timespec
    nanosecs;

extern clock_t
    start_t,
    end_t;

struct clock_struct
{
    sem_t
        cpu_sem_,
        ppu_sem_,
        cclock_sem_,
        pclock_sem_;
    uint8_t
        cycle_;
} extern nes_clock;

void clock_run(void);
void clock_init(void);
void clock_destroy(void);

#endif /* SRC_CLOCK_H_ */
