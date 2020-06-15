/*
 * ppu.h
 *
 *  Created on: Jun 5, 2020
 *      Author: David Jonsson
 */

#ifndef SRC_PPU_H_
#define SRC_PPU_H_

#include <stdbool.h>
#include <stdint.h>
#include "peripheral.h"

/**
 * PPU-registers
 */
#define PPU_CTRL        0b00000001
#define PPU_MASK        0b00000010
#define PPU_STATUS      0b00000100
#define PPU_OAM_ADDR    0b00001000
#define PPU_OAM_DATA    0b00010000
#define PPU_SCROLL      0b00100000
#define PPU_ADDR        0b01000000
#define PPU_DATA        0b10000000

/**
 * CTRL-bits
 */
#define PPU_CTRL_NAMETABLE_ADDR0    0b00000001
#define PPU_CTRL_NAMETABLE_ADDR1    0b00000010
#define PPU_CTRL_VRAM_INCR          0b00000100
#define PPU_CTRL_SPRITE_TABLE       0b00001000
#define PPU_CTRL_BACKGROUND_ADDR    0b00010000
#define PPU_CTRL_SPRITE_SIZE        0b00100000
#define PPU_CTRL_PPU_MASTER         0b01000000
#define PPU_CTRL_PPU_GENERATE_NMI   0b10000000

/**
 * MASK-bits
 */
#define PPU_MASK_GRAYSCALE                  0b00000001
#define PPU_MASK_SHOW_LEFTMOST_BACKGROUND   0b00000010
#define PPU_MASK_SHOW_LEFTMOST_SPRITE       0b00000100
#define PPU_MASK_SHOW_BACKGROUNND           0b00001000
#define PPU_MASK_SHOW_SPRITE                0b00010000
#define PPU_MASK_EMP_RED                    0b00100000
#define PPU_MASK_EMP_GREEN                  0b01000000
#define PPU_MASK_EMP_BLUE                   0b10000000

/**
 * STATUS-bits
 */
#define PPU_STATUS_LSB0             0b00000001
#define PPU_STATUS_LSB1             0b00000010
#define PPU_STATUS_LSB2             0b00000100
#define PPU_STATUS_LSB3             0b00001000
#define PPU_STATUS_LSB4             0b00010000
#define PPU_STATUS_SPRITE_OVERFLOW  0b00100000
#define PPU_STATUS_SPRITE_ZERO_HIT  0b01000000
#define PPU_STATUS_VBLANK           0b10000000

struct ppu_2C0X
{
    uint8_t
        regs_[8],
        oam_prm_[256],
        oam_sec_[32],
        read_flags_,
        write_flags_;
    uint16_t
        scroll_,
        ppu_addr_;
    bool
        latch_;

} extern ppu;

#endif /* SRC_PPU_H_ */
