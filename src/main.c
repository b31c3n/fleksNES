/*
 * main.c
 *
 *  Created on: May 4, 2020
 *      Author: David Jonsson
 */
#include <omp.h>
#include <time.h>
#include "cpu.h"
#include "tui.h";
#include "mapper.h"
#include "display.h"
#include "instruction_tbl.h"
#include "clock.h"

int main(int argc, char **argv)
{

    #pragma omp parallel
    {
        #pragma omp single nowait
        {
            log_clear();
            clock_init();
            mapper_init(argv[1]);
//            cpu.program_counter_.word_ = 0xC000;
            #pragma omp parallel
            {
                #pragma omp single nowait
                {
                    clock_run();
                }
                #pragma omp single nowait
                {
                    cpu_run();
                }
                #pragma omp single nowait
                {
                    ppu_run();
                }
            }

            /*
             *   omp parrallel
             *   while(1)
             *      run cpu 1/3 cycle
             *      run ppu 1 cycle
             *      clock 1 cycle
             *
             */

            mapper_destroy();
        }
        #pragma omp single nowait
        {
            struct timespec
                nanosecs = { .tv_nsec = 11, .tv_sec = 0 };
            tui_init();
            while(1)
            {
                tui_draw();
//                tui_get_command();
                nanosleep(&nanosecs, NULL);
            }
            tui_destroy();
        }
//        #pragma omp single nowait
//        {
//            display_init();
//            while(display_draw());
//            display_destroy();
//
//        }
    }

    return 0;
}
