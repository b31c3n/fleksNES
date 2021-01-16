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

struct ppu_2C0X
{
    uint8_t
        regs_[8],
        oam_prm_[256],
        oam_sec_[4 * 8],
        oam_dummy_[4 * 2],// One extra dummy entry
        sprite_shifters_lo_[16],
        sprite_shifters_hi_[16],
        ppu_data_buffer_,
        x_finescroll_,
        next_pattern_lo_,
        next_pattern_hi_,
        next_tile_id_,
        next_attr_;

    uint16_t
        vram_addr_,
        tram_addr_,
        scanline_,
        cycle_,
        frame_,
        shift_attr_hi_,
        shift_attr_lo_,
        shift_pattern_hi_,
        shift_pattern_lo_,
        tile_idx_;
    uint32_t
        *oam_sec_p;
    uint8_t
        pixels_[256 * 240];

    bool
        latch_,
        zero_hit_possible_,
        zero_hit_detected_;

} extern ppu;

extern struct timeval
    start,
    end;

void ppu_read();
void ppu_write();
void ppu_run(void);
void ppu_tick(void);

#endif /* SRC_PPU_H_ */
