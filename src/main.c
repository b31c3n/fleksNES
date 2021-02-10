/*
 * main.c
 *
 *  Created on: May 4, 2020
 *      Author: David Jonsson
 */
#include <omp.h>
#include <signal.h>

#include "cpu.h"
#include "state.h"
//#include "tui.h";
//#include "mapper.h"
//#include "display.h"
//#include "instruction_tbl.h"
//#include "clock.h"
//#include "ppu.h"
//#include "config.h"
//#include "state.h"
#include "api.h"

void sigint(int signal)
{
    cpu_shutdown = 1;
    tui_destroy();
    puts("Interrupted!");
}

void sigsegv(int signal)
{
    cpu_shutdown = 1;
    tui_destroy();
    puts("Segfault!");
}

int main(int argc, char **argv)
{
//    printf("%i\n", sizeof(struct nes_state));
    signal(SIGINT, sigint);
    signal(SIGSEGV, sigsegv);
    #pragma omp parallel sections
    {
        #pragma omp section
        {
            log_clear();
            uint8_t
                *ph1 = &internal_state,
                *ph2,
                *ph3;
            fleks_init( argv[1],
                        ph1,
                        ph2,
                        ph3);
            cpu_run();
            mapper_destroy();
        }
        #ifdef USE_TUI
        #pragma omp section
        {
            tui_init();
            while(!cpu_shutdown)
            {
                tui_draw();
            }
        }
        #endif
        #pragma omp section
        {
            display_init();
            while(!cpu_shutdown)
            {
                display_draw();
                capture_events();
            }
            display_destroy();
        }
    }
    return 0;
}
