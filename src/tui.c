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
 * Mem1
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

//uint16_t
//    tui_mem_offsets[] = {0x23C0, 0x3f00};
//void tui_get_command()
//{
//    char
//        buffer[80],
//        temp[80],
//        *p = buffer;
//    bool
//        is_opcode = true;
//
//    wgetstr(tui_components[TUI_NR_COMPS - 1].window_, buffer);
//
//    /**
//     * Opcode stuff
//     */
//    for(; *p != '\0' && is_opcode; ++p)
//            is_opcode = isxdigit(*p);
//
//    if(is_opcode)
//    {
//        int temp = (int)strtol(buffer, NULL, 16);
//        cpu_load_instruction(temp);
//        return;
//    }
//
//    /**
//     * Memory stuff
//     */
//    strcpy(temp, buffer);
//    char
//        *arg1 = temp,
//        *arg2 = &temp[3];
//    arg1[2] = arg2[3] = '\0';
//
//    bool
//        move_mem1 = !(strcmp(arg1, "m1")),
//        move_mem2 = !(strcmp(arg1, "m2")),
//        is_hex = true;
//    uint8_t
//        target = move_mem1 ? 0 : 1;
//
//    for(p = arg2; *p != '\0' && is_hex; ++p)
//        is_hex = isxdigit(*p);
//
//    if(move_mem1 || move_mem2)
//    {
//        if(!is_hex)
//        {
//            if(!strcmp(&temp[3], "ram")) tui_components[target].component_ = &cpu_peripheral_ram;
//            if(!strcmp(&temp[3], "cr0")) tui_components[target].component_ = &cpu_peripheral_prgram;
//            if(!strcmp(&temp[3], "cr1")) tui_components[target].component_ = &cpu_peripheral_prgrom;
//            if(!strcmp(&temp[3], "cr2")) tui_components[target].component_ = &ppu_peripheral_chrrom;
//            tui_components[target].init(&tui_components[target]);
//        }
//        else
//        {
//            arg2[2] = arg2[3] = '0';
//            arg2[4] = '\0';
//
//            struct peripheral
//                *temp = (struct peripheral *) tui_components[target].component_;
//            uint16_t
//                offset = (int)strtol(arg2, NULL, 16),
//                high_limit = temp->address_max_ & temp->mirror_mask_,
//                low_limit = temp->address_min_;
//
//            if(offset >= low_limit && offset + 0xFF <= high_limit)
//                tui_mem_offsets[target] = offset;
//        }
//    }
//}
//
//
///**
// * Mem1
// */
//static struct tui_comp
//    *tui_mem1 = NULL;
//static struct peripheral
//    *per_mem1 = NULL;
//
//void tui_mem1_update()
//{
//    wmove(tui_mem1->window_, 0, 0);
//    wprintw(tui_mem1->window_, "     ");
//    for(uint8_t j = 0; j <= 0xF; ++j)
//    {
//        wprintw(tui_mem1->window_, "%02x ", j);
//    }
//    uint16_t
//        i = tui_mem_offsets[0],
//        stop = i + 0xF0;
//
//    //stop &= per_mem1->mirror_mask_;
//
//    for(;i <= stop; i += 0x10)
//    {
//        wprintw(tui_mem1->window_, "\n%04x ",i);
//        for(uint8_t j = 0; j <= 0xF; ++j)
//        {
//            uint8_t data = per_mem1->memory_[i + j - per_mem1->address_min_];
//            wprintw(tui_mem1->window_, "%02x ", data);
//        }
//    }
//}
//
//void tui_mem1_init(void *this)
//{
//    tui_mem1 = this;
//    per_mem1 = tui_mem1->component_;
////    tui_mem_offsets[0] = per_mem1->address_min_;
//    tui_mem_offsets[0] = 0x23C0;
//}
//
///**
// * Mem2
// */
//static struct tui_comp
//    *tui_mem2 = NULL;
//static struct peripheral
//    *per_mem2 = NULL;
//
//void tui_mem2_update()
//{
//    wmove(tui_mem2->window_, 0, 0);
//    wprintw(tui_mem2->window_, "     ");
//    for(uint8_t j = 0; j <= 0xF; ++j)
//    {
//        wprintw(tui_mem2->window_, "%02x ", j);
//    }
//    uint16_t
//        i = tui_mem_offsets[1],
//        stop = i + 0xF0;
//
//    stop &= per_mem2->mirror_mask_;
//
//    for(;i <= stop; i += 0x10)
//    {
//        wprintw(tui_mem2->window_, "\n%04x ",i);
//        for(uint8_t j = 0; j <= 0xF; ++j)
//        {
//            uint8_t data = per_mem2->memory_[i + j - per_mem2->address_min_];
//            wprintw(tui_mem2->window_, "%02x ", data);
//        }
//    }
//}
//
//void tui_mem2_init(void *this)
//{
//    tui_mem2 = this;
//    per_mem2 = tui_mem2->component_;
//    tui_mem_offsets[1] = per_mem2->address_min_;
//}
//
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

};

