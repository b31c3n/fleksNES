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
            uint8_t data = ntable_mem[1][i + j];
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

