/*
 * tui.c
 *
 *  Created on: May 13, 2020
 *      Author: David Jonsson
 */

#include <math.h>
#include <stdlib.h>
#include <stdint.h>

#include "tui.h"
#include "peripherals.h"
#include "cpu.h"
#include "bus.h"
#include "ppu.h"
#include "nametable.h"
#include "mapper.h"

void tui_init()
{

    initscr();
    cbreak();
    keypad(stdscr, TRUE);
    for(int i = 0; i < TUI_NR_COMPS; ++i)
    {
        tui_components[i].window_ = newwin(
            tui_components[i].h_,
            tui_components[i].w_,
            tui_components[i].y_,
            tui_components[i].x_
        );
        tui_components[i].init(&tui_components[i]);
    }
}

void tui_draw()
{
    for(int i = 0; i < TUI_NR_COMPS; ++i)
    {
        if(tui_components[i].show_)
        {
            tui_components[i].update();
            wrefresh(tui_components[i].window_);
        }
    }
}


void tui_destroy()
{
    for(int i = 0; i < TUI_NR_COMPS; ++i)
        delwin(tui_components[i].window_);
    endwin();
}

/**
 * OAM
 */
static struct tui_comp
    *tui_oam = NULL;

static uint16_t
    oam_offset = 0;

void oam_update()
{
    wmove(tui_oam->window_, 0, 0);
    wprintw(tui_oam->window_, "     ");
    for(uint8_t j = 0; j <= 0xF; ++j)
    {
        wprintw(tui_oam->window_, "%02x ", j);
    }
    uint16_t
        i = oam_offset,
        stop = i + 0xF0;

    for(;i <= stop; i += 0x10)
    {
        wprintw(tui_oam->window_, "\n%04x ",i);
        for(uint8_t j = 0; j <= 0xF; ++j)
        {
            uint8_t data = ppu.oam_prm_[i + j];
            wprintw(tui_oam->window_, "%02x ", data);
        }
    }
}

void oam_init(void *this)
{
    tui_oam = this;
}

/**
 * NAME TABLE
 */
static struct tui_comp
    *tui_ntable = NULL;

static uint16_t
    ntable_offset = 0;

void ntable_update()
{
    wmove(tui_ntable->window_, 0, 0);
    wprintw(tui_ntable->window_, "     ");
    for(uint8_t j = 0; j <= 0xF; ++j)
    {
        wprintw(tui_ntable->window_, "%02x ", j);
    }
    uint16_t
        i = ntable_offset,
        stop = i + 0xF0;

    for(;i <= stop; i += 0x10)
    {
        wprintw(tui_ntable->window_, "\n%04x ",i);
        for(uint8_t j = 0; j <= 0xF; ++j)
        {
            uint8_t data = nametable_mem[1][i + j];
            wprintw(tui_ntable->window_, "%02x ", data);
        }
    }
}

void ntable_init(void *this)
{
    tui_ntable = this;
}

/**
 * PRG ROM
 */
static struct tui_comp
    *tui_prgrom = NULL;

static uint16_t
    prgrom_offset = 0;

void prgrom_update()
{
    wmove(tui_prgrom->window_, 0, 0);
    wprintw(tui_prgrom->window_, "     ");
    for(uint8_t j = 0; j <= 0xF; ++j)
    {
        wprintw(tui_prgrom->window_, "%02x ", j);
    }
    uint16_t
        i = ntable_offset,
        stop = i + 0xF0;

    for(;i <= stop; i += 0x10)
    {
        wprintw(tui_prgrom->window_, "\n%04x ",i);
        for(uint8_t j = 0; j <= 0xF; ++j)
        {
            uint8_t data = prg_rom[i + j];
            wprintw(tui_prgrom->window_, "%02x ", data);
        }
    }
}

void prgrom_init(void *this)
{
    tui_prgrom = this;
}

///**
// * CPU
// */
//static struct tui_comp
//    *tui_cpu = NULL;
//static struct c6502
//    *per_cpu = NULL;
//
//void tui_cpu_update()
//{
//    static char flags[9] = "23456789\n";
//    for(uint8_t i = 0, j = 7; i < 8; ++i, --j)
//    {
//        flags[j] = (uint8_t) pow(2,i) & per_cpu->status_ ? '1' : '0';
//    }
//    wmove(tui_cpu->window_, 0, 0);
//    wprintw(tui_cpu->window_, "NO-BDIZC\n");
//    wprintw(tui_cpu->window_, "%s\n", flags);
//
//    wmove(tui_cpu->window_, 2, 0);
//    wprintw(tui_cpu->window_, "A: %02x", per_cpu->accumulator_);
//    wmove(tui_cpu->window_, 2, 15);
//    wprintw(tui_cpu->window_, "X: %02x", per_cpu->x_);
//    wmove(tui_cpu->window_, 2, 30);
//    wprintw(tui_cpu->window_, "Y: %02x", per_cpu->y_);
//
//    wmove(tui_cpu->window_, 0, 15);
//    wprintw(tui_cpu->window_, "SP: %04x\n", per_cpu->stack_pointer_);
//    wmove(tui_cpu->window_, 1, 15);
//    wprintw(tui_cpu->window_, "PC: %04x\n", per_cpu->program_counter_);
//    wmove(tui_cpu->window_, 0, 30);
//    wprintw(tui_cpu->window_, "INST: %02x\n", per_cpu->opcode_);
//    wmove(tui_cpu->window_, 1, 30);
//    wprintw(tui_cpu->window_, "ADDR: %04x\n", per_cpu->opcode_args_);
//}
//
//void tui_cpu_init(void *this)
//{
//    tui_cpu = this;
//    per_cpu = tui_cpu->component_;
//}
//
///**
// * Cpu bus
// */
//static struct tui_comp
//    *tui_cpubus = NULL;
//static struct bus
//    *per_cpubus = NULL;
//
//void tui_cpubus_update()
//{
//    wmove(tui_cpubus->window_, 0, 0);
//    //wprintw(tui_cpubus->window_, "CpubusWr: %i", per_cpubus->write_);
//    wmove(tui_cpubus->window_, 0, 15);
//    wprintw(tui_cpubus->window_, "DATA: %02x", per_cpubus->data_);
//    wmove(tui_cpubus->window_, 0, 30);
//    wprintw(tui_cpubus->window_, "ADDR: %04x", per_cpubus->address_);
//}
//
//void tui_cpubus_init(void *this)
//{
//    tui_cpubus = this;
//    per_cpubus = tui_cpubus->component_;
//}
//
///**
// * Commandline
// */
//static struct tui_comp
//    *tui_commandline = NULL;
//
//void tui_commandline_update()
//{
//
//}
//
//void tui_commandline_init(void *this)
//{
//    tui_commandline = this;
//}


/**
 * Components
 */
struct tui_comp
    tui_components[TUI_NR_COMPS] =
{
    {
        .update = oam_update,
        .init = oam_init,
        .x_ = 0,
        .y_ = 0,
        .w_ = 55,
        .h_ = 18,
        .show_ = true,
    },
    {
        .update = ntable_update,
        .init = ntable_init,
        .x_ = 56,
        .y_ = 0,
        .w_ = 55,
        .h_ = 18,
        .show_ = true,
    },
    {
        .update = prgrom_update,
        .init = prgrom_init,
        .x_ = 0,
        .y_ = 19,
        .w_ = 55,
        .h_ = 18,
        .show_ = true,
    },
};

