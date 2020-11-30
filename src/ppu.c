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
#include "peripherals.h"

#define CLOCK

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
    //break;
    /*
     * Nametable byte
     */
//            uint16_t
//                pal_shift = (ppu.tile_idx_ / 2) & 0b1;
//                pal_shift += (((ppu.tile_idx_ / 32) & 0b1) << 1);
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
    //break;
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
    //break;
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
    //break;
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

static void load_tile()
{
    if(ppu.regs_[PPU_MASK] & (PPU_MASK_SHOW_BACKGROUNND))
    {
        ppu.shift_attr_lo_ <<= 1;
        ppu.shift_attr_hi_ <<= 1;
        ppu.shift_pattern_lo_ <<= 1;
        ppu.shift_pattern_hi_ <<= 1;


    }

    if(ppu.regs_[PPU_MASK] & (PPU_MASK_SHOW_SPRITE))
    {
        for(uint8_t sprite_nr = 0; sprite_nr < 8; ++sprite_nr)
        {
            bool
                x_positive = ppu.oam_sec_[sprite_nr * 4 + OAM_X];
            ppu.oam_sec_[sprite_nr * 4 + OAM_X] -= x_positive;
//            if(!x_positive)
//            {
//                puts("hit");
//            }
            ppu.sprite_shifters_hi_[sprite_nr] <<= 1 - (bool) x_positive;
            ppu.sprite_shifters_hi_[sprite_nr] <<= 1 - (bool) x_positive;
        }
    }

    tile_funcs[(ppu.cycle_ - 1) % 8]();
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
    ppu.frame_      += !ppu.scanline_ && !ppu.cycle_ ? 1 : 0;

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
            ppu.regs_[PPU_STATUS] = PPU_STATUS & ~PPU_STATUS_VBLANK;

            #ifdef CLOCK
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
        if(ppu.cycle_ == 339 && !(ppu.frame_ % 2))
        {
            ppu.cycle_ = 0;
            ppu.scanline_ = 0;
        }

        /**
         * Load tiles
         */
        if((ppu.cycle_ >= 1 && ppu.cycle_ <= 256) ||
                (ppu.cycle_ >= 321 && ppu.cycle_ <= 340))
        {
            load_tile();
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
        /**
         * Load tiles
         */
        if(ppu.cycle_ >= 1 && ppu.cycle_ <= 256)
        {
            load_tile();
            if(ppu.cycle_ == 256) inc_y();

            uint8_t
                bg_pixel = 0x00,
                fg_pixel = 0x00,
                bg_palette = 0x00,
                fg_palette = 0x00,
                pixel = 0x00,
                palette = 0x00,
                fg_prio,
                bg_prio,
                color_idx;

            bool
                zero_sprite_detected = (uint64_t) ppu.oam_sec_p ^ (uint64_t) ppu.oam_sec_,
                zero_sprite_rendered;

            if(ppu.regs_[PPU_MASK] & (PPU_MASK_SHOW_BACKGROUNND))
            {

                uint16_t bit_mux = 0x8000 >> ppu.x_finescroll_;

                // Select Plane pixels by extracting from the shifter
                        // at the required location.
                uint8_t p0_pixel = (ppu.shift_pattern_lo_ & bit_mux) > 0;
                uint8_t p1_pixel = (ppu.shift_pattern_hi_ & bit_mux) > 0;

                // Combine to form pixel index
                bg_pixel = (p1_pixel << 1) | (p0_pixel << 0 );

                // Get palette
                uint8_t bg_pal0 = (ppu.shift_attr_lo_ & bit_mux) > 0;
                uint8_t bg_pal1 = (ppu.shift_attr_hi_ & bit_mux) > 0;
                bg_palette = (bg_pal1 << 1) | bg_pal0;

//                uint8_t
//                    color_idx = (bg_pixel << 0) | (bg_palette << 2);
                /*
                 * Redirect to backdrop color if 2 LSBits of color idx is 0
                 */
//                color_idx -= (color_idx + 1)  / 4;
//                color_idx = color_idx * ((color_idx & 0b1) | ((color_idx & 0b10) >> 1));
//
//                bus_read(&ppu_bus, (color_idx + 0x3F00));
//                uint8_t
//                    clr = ppu_bus.data_;
//                uint16_t
//                    pixel_idx = ppu.cycle_ - 1 + (uint16_t) ppu.scanline_ * 256;

//                ppu.pixels_[pixel_idx] = clr;
            }
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
                    fg_palette = ppu.oam_sec_[sprite_nr * 4 + OAM_ATTR] + 0x04;

                    zero_sprite_rendered = 1 - (bool) sprite_nr;
                    if(fg_pixel) break;
                }
            }
            bg_prio = (1 - (bool) fg_pixel);
            fg_prio = (bool) fg_pixel;
            pixel = fg_pixel + bg_pixel * bg_prio;
            palette = fg_palette * fg_prio + bg_palette * bg_prio;
            color_idx = (pixel << 0) | (palette << 2);

            bus_read(&ppu_bus, (color_idx + 0x3F00));
            uint8_t
                clr = ppu_bus.data_;
            uint16_t
                pixel_idx = ppu.cycle_ - 1 + (uint16_t) ppu.scanline_ * 256;

            ppu.pixels_[pixel_idx] = clr;
        }

        else if(ppu.cycle_ >= 321 && ppu.cycle_ <= 340)
        {
            load_tile();
        }

        /*
         * Reset x-scroll
         */
        else if(ppu.cycle_ == 257)
        {
            reset_x();

            /*
             * Load sprites into secondary OAM
             */
            memset(ppu.sprite_shifters_lo_, 0x00, sizeof(ppu.sprite_shifters_lo_));
            memset(ppu.sprite_shifters_hi_, 0x00, sizeof(ppu.sprite_shifters_hi_));
            memset(ppu.oam_sec_, 0xFFFFFFFF, sizeof(ppu.oam_sec_));

            bool
                negative,
                in_range,
//                zero_sprite,
                overflow;
            uint32_t
                *oam_prm_p = ppu.oam_prm_;

            ppu.oam_sec_p  = ppu.oam_sec_;

            for(;
                oam_prm_p < ppu.oam_sec_ &&
                ppu.oam_sec_p < ppu.oam_dummy_ + 1;
                )
            {
                uint16_t
                    oam_y = *((uint8_t *) oam_prm_p),
                    scan_y = ppu.scanline_,
                    result = (scan_y - oam_y);
                bool
                    positive =  1 - (bool) (result & 1 << 15),
                    /**
                     * Need to fix so it takes into account height of sprite, 16 / 8
                     */
                    in_range =  1 - (bool) (result & (~0b111));

                *ppu.oam_sec_p = *oam_prm_p;
                ++oam_prm_p;
                ppu.oam_sec_p += positive * in_range;
            }
            /*
             * Set sprite overflow,
             * and if zero_sprite
             */
//            zero_sprite = (uint64_t) ppu.oam_sec_p ^ (uint64_t) ppu.oam_sec_;
            overflow = 1 - (bool)((uint64_t) ppu.oam_sec_p ^ (uint64_t) (ppu.oam_dummy_ + 4));
            ppu.regs_[PPU_STATUS] |=  overflow << 5;
            ppu.oam_sec_p -= overflow;
            *ppu.oam_sec_p = 0xFFFFFFFF;



        }
        if(ppu.cycle_ == 340)
        {
            for(uint32_t *t_p = ppu.oam_sec_,
                index = 0;
                t_p < ppu.oam_sec_p;
                ++t_p,
                ++index)
            {
                uint8_t
                    sprite_pattern_lo,
                    sprite_pattern_hi;
                uint16_t
                    sprite_pattern_addr;

                uint8_t
                    y_t = (uint8_t) *(((uint8_t *) t_p) + OAM_Y);

                uint16_t
                    frst = (((bool) (ppu.regs_[PPU_CTRL] & PPU_CTRL_SPRITE_TABLE)) << 12),
                    scnd = (*(((uint8_t *) t_p) + OAM_ID) << 4),
                    thrd =  ppu.scanline_ - y_t;

                sprite_pattern_addr =
                        frst | scnd | thrd;

                bus_read(&ppu_bus, sprite_pattern_addr);
                sprite_pattern_lo = ppu_bus.data_;
                bus_read(&ppu_bus, sprite_pattern_addr + 8);
                sprite_pattern_hi = ppu_bus.data_;

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
    ppu_comm.address_ =
            ppu_comm.ppu_->bus_->address_ & ppu_comm.ppu_->mirror_mask_ - ppu_comm.ppu_->address_min_;
    ppu_comm.reg_ = 1 << ppu_comm.address_,
    ppu_comm.data_ = ppu_comm.ppu_->bus_->data_;
    ppu_comm.write_funcs[ppu_comm.address_]();
}

void ppu_read()
{
    ppu_comm.address_ =
            ppu_comm.ppu_->bus_->address_ & ppu_comm.ppu_->mirror_mask_ - ppu_comm.ppu_->address_min_;
    ppu_comm.reg_= 1 << ppu_comm.address_;
    ppu_comm.read_funcs[ppu_comm.address_]();
}

struct ppu_2C0X ppu =
{
        .latch_             = false,
        .scanline_          = 0,
        .cycle_             = 0,
};

struct peripheral cpu_peripheral_ppu =
{
        .bus_ = &cpu_bus,
        .address_min_ = 0x2000,
        .address_max_ = 0x3FFF,
        .mirror_mask_ = 0x2007,
        .memory_ = &ppu.regs_,
};
