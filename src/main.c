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

int main(int argc, char **argv)
{
//#pragma omp parallel
    {
//#pragma omp single nowait7
        {
            log_clear();
            mapper_init(argv[1]);
            struct timespec
                nanosecs = { .tv_nsec = 10, .tv_sec = 0 };
            cpu.program_counter_.word_ = 0xC000;
//            cpu.program_counter_ = 0x0fc0;

//            cpu.current_instruction_ = 0xea;

            while(1)
            {
                if(!(cpu.suspend_etc_ & CPU_SUSPEND))
                {
//                    cpu_set_instruction();

                    cpu_fetch_instruction();
                    cpu_execute_instruction();
//                    cpu.current_instruction_ = 0xEA;
                    if((cpu.irq_ && !(cpu.status_ & CPU_STATUS_INTERUPT)) || cpu.nmi_ )
                    {
                        cpu.opcode_ = 0x00;
                    }
                }
                nanosleep(&nanosecs, NULL);
            }
            mapper_destroy();
        }
//#pragma omp single nowait
//        {
//            struct timespec
//                nanosecs = { .tv_nsec = 11, .tv_sec = 0 };
//            tui_init();
//            while(1)
//            {
//                tui_draw();
//                //tui_get_command();
//                nanosleep(&nanosecs, NULL);
//            }
//            tui_destroy();
//        }
//#pragma omp single nowait
//        {
//            display_init();
//            while(display_draw());
//            display_destroy();
//        }
    }
    return 0;
}
