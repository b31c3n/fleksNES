/*
 * main.c
 *
 *  Created on: May 4, 2020
 *      Author: David Jonsson
 */
#include <omp.h>
#include <time.h>
#include <signal.h>

#include "cpu.h"
#include "tui.h";
#include "mapper.h"
#include "display.h"
#include "instruction_tbl.h"
#include "clock.h"

void ctrl_c(int signal)
{
    shutdown = 1;
}

int main(int argc, char **argv)
{
    signal(SIGINT, ctrl_c);

    #pragma omp parallel
    {
        #pragma omp single nowait
        {
            log_clear();
            clock_init();
            mapper_init(argv[1]);
            cpu_run();
            mapper_destroy();
        }
        #pragma omp single nowait
        {
            struct timespec
                nanosecs = { .tv_nsec = 11, .tv_sec = 0 };
            tui_init();
            while(!shutdown)
            {
                tui_draw();
                nanosleep(&nanosecs, NULL);
            }
            tui_destroy();
        }
        #pragma omp single nowait
        {
            display_init();
            while(!shutdown) display_draw();
            display_destroy();
        }
    }
    return 0;
}
