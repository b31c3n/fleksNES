/*
 * clock.c
 *
 *  Created on: Aug 21, 2020
 *      Author: David Jonsson
 */

#include "clock.h"

struct clock_struct
    nes_clock;

void clock_run(void)
{
    while(1)
    {
        sem_post(&nes_clock.ppu_sem_);
        sem_wait(&nes_clock.pclock_sem_);

        if(!nes_clock.cycle_)
        {
            nes_clock.cycle_ = 2;
            sem_post(&nes_clock.cpu_sem_);
            sem_wait(&nes_clock.cclock_sem_);
        }
        else
            --nes_clock.cycle_;
    }
}

void clock_init(void)
{
    nes_clock.cycle_ = 2;
    sem_init(&nes_clock.cclock_sem_, 0, 0);
    sem_init(&nes_clock.cclock_sem_, 0, 0);
    sem_init(&nes_clock.cpu_sem_, 0, 0);
    sem_init(&nes_clock.ppu_sem_, 0, 0);
}

void clock_destroy(void)
{
    sem_destroy(&nes_clock.cclock_sem_);
    sem_destroy(&nes_clock.cpu_sem_);
    sem_destroy(&nes_clock.ppu_sem_);
}
