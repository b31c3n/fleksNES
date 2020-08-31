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
#define PPU_CTRL        0
#define PPU_MASK        1
#define PPU_STATUS      2
#define PPU_OAM_ADDR    3
#define PPU_OAM_DATA    4
#define PPU_SCROLL      5
#define PPU_ADDR        6
#define PPU_DATA        7

/**
 * CTRL-bits
 */
#define PPU_CTRL_NAMETABLE_ADDR0    0b00000001
#define PPU_CTRL_NAMETABLE_ADDR1    0b00000010
#define PPU_CTRL_VRAM_INCR          0b00000100
#define PPU_CTRL_SPRITE_TABLE       0b00001000
#define PPU_CTRL_BACKGROUND_ADDR    0b00010000
#define PPU_CTRL_SPRITE_SIZE        0b00100000
#define PPU_CTRL_MASTER             0b01000000
#define PPU_CTRL_GENERATE_NMI       0b10000000

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

/**
 * VRAMreg-bitmasks
 */
#define VRAM_COARSE_X           0b0000000000011111
#define VRAM_COARSE_Y           0b0000001111100000
#define VRAM_TABLESEL           0b0000110000000000
#define VRAM_FINE_Y             0b0111000000000000
#define VRAM_FINE_X             0b0000000000000111
#define VRAM_HIGH               0b0011111100000000
#define VRAM_LOW                0b0000000011111111

struct ppu_2C0X
{
    uint8_t
        regs_[8],
        oam_prm_[256],
        oam_sec_[32],
        ppu_data_buffer_,
        read_flags_,
        write_flags_,
        x_finescroll_,
        shiftreg_tile_lsb,
        shiftreg_tile_msb,
        shiftreg_tile_id_,
        shiftreg_attr_;

    uint16_t
        vram_addr_,
        tram_addr_,
        scanline_,
        cycle_,
        frame_;

    bool
        latch_;

} extern ppu;

void ppu_run(void);

#endif /* SRC_PPU_H_ */
