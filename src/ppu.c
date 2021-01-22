/*
 * ppu.c
 *
 *  Created on: Jun 5, 2020
 *      Author: David Jonsson
 */

#include <sys/time.h>

#include "ppu.h"
#include "clock.h"
#include "helper_funcs.h"
#include "cpu.h"
#include "ppu_comm.h"
#include "config.h"

void ppu_tick(void)
{

}

static bool rendering()
{
    return ppu.regs_[PPU_MASK] & (PPU_MASK_SHOW_BACKGROUNND | PPU_MASK_SHOW_SPRITE);
}

static void inc_x()
{
    if(!rendering()) return;

    if((ppu.vram_addr_ & 0x001F) == 31)
    {
        ppu.vram_addr_ &= ~0x001F;
        ppu.vram_addr_ ^= 0x0400;
    }
    else
        ppu.vram_addr_ += 1;
}

static void inc_y()
{
    if(!rendering()) return;

    if((ppu.vram_addr_ & 0x7000) != 0x7000)
        ppu.vram_addr_ += 0x1000;
    else
    {
        ppu.vram_addr_ &= ~0x7000;
        int y = (ppu.vram_addr_ & 0x03E0) >> 5;
        if (y == 29)
        {
            y = 0;
            ppu.vram_addr_ ^= 0x0800;
        }
        else if (y == 31)
            y = 0;
        else
            y += 1;
        ppu.vram_addr_ = (ppu.vram_addr_ & ~0x03E0) | (y << 5);
    }
}

static void reset_x()
{
    if(!rendering()) return;

    uint16_t
        mask = VRAM_COARSE_X | 0x0400;
    ppu.vram_addr_ &= ~(mask | 0x8000);
    ppu.vram_addr_ |= ppu.tram_addr_ & mask;
}

static void reset_y()
{
    if(!rendering()) return;

    uint16_t
        mask = VRAM_COARSE_Y | VRAM_FINE_Y | 0x0800;
    ppu.vram_addr_ &= ~(mask | 0x8000);
    ppu.vram_addr_ |= ppu.tram_addr_ & mask;
}

static void case0()
{
    /*
     * Nametable byte
     */
    ppu.shift_pattern_lo_   &= 0xFF00,
    ppu.shift_pattern_hi_   &= 0xFF00,
    ppu.shift_pattern_lo_   |= ppu.next_pattern_lo_,
    ppu.shift_pattern_hi_   |= ppu.next_pattern_hi_,
    ppu.shift_attr_lo_      &= 0xFF00,
    ppu.shift_attr_hi_      &= 0xFF00,
    ppu.shift_attr_lo_      |= ((bool) (ppu.next_attr_ & 0b01)) * 0xFF,
    ppu.shift_attr_hi_      |= ((bool) (ppu.next_attr_ & 0b10)) * 0xFF;

    uint16_t
        address = 0x2000 | (ppu.vram_addr_ & 0x0FFF);
    bus_read(&ppu_bus, address);
    ppu.next_tile_id_ = ppu_bus.data_;
}

static void case2()
{
    /*
     * Attribute table byte
     */
    uint16_t
        address =   0x23C0 |
                    (ppu.vram_addr_ & VRAM_TABLESEL) |
                    ((ppu.vram_addr_ >> 4) & 0b111000) |
                    ((ppu.vram_addr_ >> 2) & 0b000111);
    bus_read(&ppu_bus, address);
    ppu.next_attr_ = ppu_bus.data_;
    ppu.next_attr_ >>= 4 * ((bool)(ppu.vram_addr_ & 0b1000000));
    ppu.next_attr_ >>= 2 * ((bool)(ppu.vram_addr_ & 0b10));
    ppu.next_attr_ &= 0x03;

}

static void case4()
{
    /**
     * Getting next tile lsb
     */
    uint16_t
        address =
            (0b1 << 12) * ((bool) (ppu.regs_[PPU_CTRL] & PPU_CTRL_BACKGROUND_ADDR)) +
            (((uint16_t) ppu.next_tile_id_) << 4) +
            ((ppu.vram_addr_ & VRAM_FINE_Y) >> 12);

    bus_read(&ppu_bus, address);
    ppu.next_pattern_lo_ = ppu_bus.data_;
}

static void case6()
{
    /**
     * Getting next tile msb
     */
    uint16_t
        address =
            (0b1 << 12) * ((bool) (ppu.regs_[PPU_CTRL] & PPU_CTRL_BACKGROUND_ADDR)) +
            (((uint16_t) ppu.next_tile_id_) << 4) +
            (((ppu.vram_addr_ & VRAM_FINE_Y) >> 12) + 8);

    bus_read(&ppu_bus, address);
    ppu.next_pattern_hi_ = ppu_bus.data_;
}

static void case7()
{
    inc_x();
}

static void case_def()
{

}

static void
    (*tile_funcs[8]) () =
    {
            case0,
            case_def,
            case2,
            case_def,
            case4,
            case_def,
            case6,
            case7
    };

static void shift_bg()
{
    if(ppu.regs_[PPU_MASK] & (PPU_MASK_SHOW_BACKGROUNND))
    {
        uint64_t
            *shift_regs = &ppu.shift_attr_hi_;
        *shift_regs &= ~0x8000800080008000;
        *shift_regs <<= 1;
    }
}

static void shift_fg()
{
    bool
        shift = (ppu.regs_[PPU_MASK] & (PPU_MASK_SHOW_SPRITE));
    shift = 1;
    if(ppu.regs_[PPU_MASK] & (PPU_MASK_SHOW_BACKGROUNND))
    {
        for(uint8_t sprite_nr = 0; sprite_nr < 8; ++sprite_nr)
        {
            bool
                x_positive = ppu.oam_sec_[sprite_nr * 4 + OAM_X];
            ppu.oam_sec_[sprite_nr * 4 + OAM_X] -= x_positive & shift;
            ppu.sprite_shifters_lo_[sprite_nr] <<= (1 - x_positive) & shift;
            ppu.sprite_shifters_hi_[sprite_nr] <<= (1 - x_positive) & shift;
        }
    }
}


struct timeval
    start,
    end;

uint16_t
    frames = 0;

void ppu_run(void)
{
    ppu.cycle_ 		++,
    ppu.cycle_		%= 341,
    ppu.scanline_ 	+= 1 - (bool) ppu.cycle_,
    ppu.scanline_ 	%= 262,
    ppu.frame_      += 1 - ((bool) ppu.scanline_ | (bool) ppu.cycle_);

    /*
     * Pre-render scanline (-1 or 261)
     */
    if(ppu.scanline_ == 261)
    {
        /*
         * Set vblank at cycle 1
         */
        if(ppu.cycle_ == 1)
        {
            ppu.regs_[PPU_STATUS] &= ~(PPU_STATUS_VBLANK | PPU_STATUS_SPRITE_ZERO_HIT);
            ppu.zero_hit_possible_ = 1;

            #ifdef MEASURE_FPS
                gettimeofday(&end, NULL);
                if(end.tv_sec - start.tv_sec > 1)
                {
                    printf("%i fps\n", frames);
                    frames = 0;
                    gettimeofday(&start, NULL);
                }
                else ++frames;
            #endif
        }

        /*
         * Check if end of scanline and on odd frame
         */
        bool
            odd_end = ((bool) (ppu.cycle_ ^ 339) - 1) & (ppu.frame_ & 1);

        ppu.cycle_ *= 1 - odd_end;
        ppu.scanline_ *= 1- odd_end;


        /**
         * Load tiles
         */
        if((ppu.cycle_ >= 1 && ppu.cycle_ <= 256) ||
                (ppu.cycle_ >= 321 && ppu.cycle_ <= 336))
        {
            shift_bg();
            tile_funcs[(ppu.cycle_ - 1) % 8]();
            if(ppu.cycle_ == 256) inc_y();
        }

        /*
         * Reset x/y-scrolls
         */
        else if(ppu.cycle_ == 257)
        {
            reset_x();
        }
        else if(ppu.cycle_ >= 280 && ppu.cycle_ <= 304)
            reset_y();
    }

    /*
     * Visible scanlines (0-239)
     */
    else if(ppu.scanline_ >= 0 && ppu.scanline_ <= 239)
    {

        if(ppu.cycle_ >= 1 && ppu.cycle_ <= 256)
        {
            shift_bg();
            tile_funcs[(ppu.cycle_ - 1) % 8]();

            if(ppu.cycle_ == 256) inc_y();

            /*
             * Fill the current pixel
             */
            uint8_t
                bg_pixel = 0x00,
                fg_pixel = 0x00,
                bg_palette = 0x00,
                fg_palette = 0x00,
                pixel = 0x00,
                palette = 0x00,
                color_idx;

            bool
                zero_sprite_detected = ppu.zero_hit_detected_,
                zero_sprite_rendered,
                zero_hit,
                fg_prio,
                bg_prio;

            /*
             * Get background
             */
            if(ppu.regs_[PPU_MASK] & (PPU_MASK_SHOW_BACKGROUNND))
            {
                uint16_t bit_mux = 0x8000 >> ppu.x_finescroll_;
                bool
                    pixel_lo = ppu.shift_pattern_lo_ & bit_mux,
                    pixel_hi = ppu.shift_pattern_hi_ & bit_mux,
                    bg_pal0 = ppu.shift_attr_lo_ & bit_mux,
                    bg_pal1 = ppu.shift_attr_hi_ & bit_mux;

                bg_pixel = (pixel_hi << 1) | (pixel_lo << 0 );
                bg_palette = (bg_pal1 << 1) | bg_pal0;
            }

            /**
             * Pick the visible sprite with moste priority
             */
            if(ppu.regs_[PPU_MASK] & (PPU_MASK_SHOW_SPRITE))
            {
                for(uint8_t sprite_nr = 0; sprite_nr < 8; ++sprite_nr)
                {
                    bool
                        x_zero = 1 - (bool) ppu.oam_sec_[sprite_nr * 4 + OAM_X],
                        pixel_lo = ppu.sprite_shifters_lo_[sprite_nr] & 0x80,
                        pixel_hi = ppu.sprite_shifters_hi_[sprite_nr] & 0x80;
                    fg_pixel = pixel_lo | (pixel_hi << 1);
                    fg_pixel *= x_zero;

                    uint8_t
                        attr = ppu.oam_sec_[(sprite_nr << 2) + OAM_ATTR];
                    fg_palette = attr + 0x04;

                    fg_prio = 1 - (bool) (attr & 0x20);

                    zero_sprite_rendered = 1 - (bool) sprite_nr;
                    if(fg_pixel)
                    {
                        break;
                    }

                }
            }

            /*
             * Calculate priorities
             */
            bg_prio = (1 - ((fg_prio & ((bool) fg_pixel))) & (bool) bg_pixel);
            fg_prio = 1 - bg_prio;
            pixel = fg_pixel * fg_prio + bg_pixel * bg_prio;

            /*
             * From priorities draw either fg or bg
             */
            palette = fg_palette * fg_prio + bg_palette * bg_prio;
            palette *= (bool) ((bool) bg_pixel + (bool) fg_pixel);
            color_idx = (pixel << 0) | (palette << 2);

            bus_read(&ppu_bus, (color_idx + 0x3F00));
            uint8_t
                clr = ppu_bus.data_;
            uint16_t
                pixel_idx = ppu.cycle_ - 1 + (uint16_t) ppu.scanline_ * 256;

            ppu.pixels_[pixel_idx] = clr;

            /**
             * Zero sprite hit?
             */
            zero_hit = (bool) bg_pixel
                        & ((bool) (fg_pixel))
                        & ((bool) (ppu.regs_[PPU_MASK] & PPU_MASK_SHOW_BACKGROUNND))
                        & ((bool) (ppu.regs_[PPU_MASK] & PPU_MASK_SHOW_SPRITE))
                        & zero_sprite_detected
                        & zero_sprite_rendered
                        & ((bool) (ppu.cycle_ ^ 0b11111111));

            bool
                left_most_render =
                     (1 - (bool) ppu.regs_[PPU_MASK] &
                             (PPU_MASK_SHOW_LEFTMOST_BACKGROUND | PPU_MASK_SHOW_LEFTMOST_SPRITE))
                      & (1 - (bool) (ppu.cycle_ & (~0b111)));

            zero_hit = (1 - left_most_render) & zero_hit & ppu.zero_hit_possible_;
            ppu.regs_[PPU_STATUS] |= zero_hit << 6;
            ppu.zero_hit_possible_ -= zero_hit;

            /**
             * Needed to shift sprites here to match with background
             */
            shift_fg();
        }

        else if(ppu.cycle_ >= 321 && ppu.cycle_ <= 336)
        {
            shift_bg();
            tile_funcs[(ppu.cycle_ - 1) % 8]();
        }

        /*
         * Reset x-scroll
         */
        else if(ppu.cycle_ == 257)
        {
            reset_x();

            /*
             * Shifters and secondary OAM
             */
            memset(ppu.sprite_shifters_lo_, 0x00, sizeof(ppu.sprite_shifters_lo_));
            memset(ppu.sprite_shifters_hi_, 0x00, sizeof(ppu.sprite_shifters_hi_));
            memset(ppu.oam_sec_, 0xFFFFFFFF, sizeof(ppu.oam_sec_));

            bool
                negative,
                in_range,
                overflow;
            uint32_t
                *oam_prm_p = ppu.oam_prm_;

            ppu.oam_sec_p  = ppu.oam_sec_;
            ppu.zero_hit_detected_ = false;

            /*
             * Load sprite data into secondary OAM
             */
            for(;
                oam_prm_p < ppu.oam_sec_ &&
                ppu.oam_sec_p < ppu.oam_dummy_ + 1;
                )
            {
                uint16_t
                    oam_y = (uint16_t) *((uint8_t *) oam_prm_p),
                    scan_y = ppu.scanline_,
                    result = (scan_y - oam_y);
                bool
                    positive =  1 - (bool) (result & (1 << 15)),
                    /**
                     * Need to fix so it takes into account height of sprite, 16 / 8
                     */
                    in_range =  1 - (bool) (result & (~0b111));

                *ppu.oam_sec_p = *oam_prm_p;
                ppu.oam_sec_p += positive & in_range;

                /**
                 * Zero sprite?
                 */
                if(!ppu.zero_hit_detected_)
                {
                    ppu.zero_hit_detected_ += 1 - (bool) ((uint64_t) oam_prm_p ^ (uint64_t) ppu.oam_prm_);
                    ppu.zero_hit_detected_ &= positive & in_range;
                }
                ++oam_prm_p;
            }

            /*
             * Set sprite overflow,
             * and if zero_sprite
             */
            overflow = 1 - (bool)((uint64_t) ppu.oam_sec_p ^ (uint64_t) (ppu.oam_dummy_ + 4));
            ppu.regs_[PPU_STATUS] |=  overflow << 5;
            ppu.oam_sec_p -= overflow;
            *ppu.oam_sec_p = 0xFFFFFFFF;

            /*
             * Load next sprites into shift registers
             */
            for(uint32_t *t_p = ppu.oam_sec_,
                index = 0;
                t_p < ppu.oam_sec_p;
                ++t_p,
                ++index)
            {
                bool
                    vert_flip = (*(((uint8_t *) t_p) + OAM_ATTR)) & 0x80,
                    hor_flip  = (*(((uint8_t *) t_p) + OAM_ATTR)) & 0x40;
                uint8_t
                    sprite_pattern_lo,
                    sprite_pattern_hi,
                    y_t = (uint8_t) *(((uint8_t *) t_p) + OAM_Y);
                uint16_t
                    sprite_pattern_addr,
                    frst = (((bool) (ppu.regs_[PPU_CTRL] & PPU_CTRL_SPRITE_TABLE)) << 12),
                    scnd = (*(((uint8_t *) t_p) + OAM_ID) << 4),
                    thrd =  (ppu.scanline_ - y_t) * (1 - vert_flip)
                            | (7 - (ppu.scanline_ - y_t)) * vert_flip;

                sprite_pattern_addr = frst | scnd | thrd;
                bus_read(&ppu_bus, sprite_pattern_addr);
                sprite_pattern_lo = ppu_bus.data_;
                bus_read(&ppu_bus, sprite_pattern_addr + 8);
                sprite_pattern_hi = ppu_bus.data_;

                if(hor_flip)
                {
                    sprite_pattern_lo = (sprite_pattern_lo & 0xF0) >> 4 | (sprite_pattern_lo & 0x0F) << 4;
                    sprite_pattern_lo = (sprite_pattern_lo & 0xCC) >> 2 | (sprite_pattern_lo & 0x33) << 2;
                    sprite_pattern_lo = (sprite_pattern_lo & 0xAA) >> 1 | (sprite_pattern_lo & 0x55) << 1;

                    sprite_pattern_hi = (sprite_pattern_hi & 0xF0) >> 4 | (sprite_pattern_hi & 0x0F) << 4;
                    sprite_pattern_hi = (sprite_pattern_hi & 0xCC) >> 2 | (sprite_pattern_hi & 0x33) << 2;
                    sprite_pattern_hi = (sprite_pattern_hi & 0xAA) >> 1 | (sprite_pattern_hi & 0x55) << 1;
                }
                ppu.sprite_shifters_lo_[index] = sprite_pattern_lo;
                ppu.sprite_shifters_hi_[index] = sprite_pattern_hi;
            }
        }
    }

    /*
     * Scanline 241
     */
    else if(ppu.scanline_ == 241 && ppu.cycle_ == 1)
    {
        ppu.regs_[PPU_STATUS] = PPU_STATUS | PPU_STATUS_VBLANK;
        if(ppu.regs_[PPU_CTRL] & PPU_CTRL_GENERATE_NMI)
        {
            cpu.nmi_ = 1;
        }
    }
}

void ppu_write()
{
    ppu_comm.address_ = cpu_bus.address_ & 0x7;
//    ppu_comm.reg_ = 1 << ppu_comm.address_,
    ppu_comm.data_ = cpu_bus.data_;
    ppu_comm.write_funcs[ppu_comm.address_]();
}

void ppu_read()
{
    ppu_comm.address_ = cpu_bus.address_ & 0x7;
//    ppu_comm.reg_= 1 << ppu_comm.address_;
    ppu_comm.read_funcs[ppu_comm.address_]();
}

struct ppu_2C0X ppu =
{
        .latch_             = false,
        .scanline_          = 261,
        .cycle_             = 0,
};
