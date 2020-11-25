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
    ppu.vram_addr_ &= ~mask;
    ppu.vram_addr_ |= ppu.tram_addr_ & mask;
}

static void reset_y()
{
    if(!rendering()) return;

    uint16_t
        mask = VRAM_COARSE_Y | VRAM_FINE_Y | 0x0800;
    ppu.vram_addr_ &= ~mask;
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
    ppu.next_attr_ >>= 4 * (bool)(ppu.vram_addr_ & 0b1000000);
    ppu.next_attr_ >>= 2 * (bool)(ppu.vram_addr_ & 0b10);
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
            0x1000 * ((bool) (ppu.regs_[PPU_CTRL] & PPU_CTRL_BACKGROUND_ADDR)) +
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
            0x1000 * ((bool) (ppu.regs_[PPU_CTRL] & PPU_CTRL_BACKGROUND_ADDR)) +
            (((uint16_t) ppu.next_tile_id_) << 4) +
            ((ppu.vram_addr_ & VRAM_FINE_Y) >> 12 + 8);

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
    //return;
    if(ppu.regs_[PPU_MASK] & (PPU_MASK_SHOW_BACKGROUNND))
    {
        ppu.shift_attr_lo_ <<= 1;
        ppu.shift_attr_hi_ <<= 1;
        ppu.shift_pattern_lo_ <<= 1;
        ppu.shift_pattern_hi_ <<= 1;
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
    ppu.scanline_ 	+= !ppu.cycle_ ? 1 : 0,
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

            gettimeofday(&end, NULL);
            if(end.tv_sec - start.tv_sec > 1)
            {
                printf("%i fps\n", frames);
                frames = 0;
                gettimeofday(&start, NULL);
            }
            else ++frames;
        }

        /*
         * Check if end of scanline and on odd frame
         */
        else if(ppu.cycle_ == 339 && !(ppu.frame_ % 2))
        {
            ppu.cycle_ = 0;
            ppu.scanline_ = 0;
        }

        /**
         * Load tiles
         */
        else if((ppu.cycle_ >= 1 && ppu.cycle_ <= 256) ||
                (ppu.cycle_ >= 321 && ppu.cycle_ <= 336))
        {
            load_tile();
            if(ppu.cycle_ == 256) inc_y();
        }

        /*
         * Reset x/y-scrolls
         */
        else if(ppu.cycle_ == 257)
            reset_x();
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

            uint8_t bg_pixel = 0x00;   // The 2-bit pixel to be rendered
            uint8_t bg_palette = 0x00;
            uint16_t bit_mux = 0x8000 >> ppu.x_finescroll_;

            // Select Plane pixels by extracting from the shifter
                    // at the required location.
            uint8_t p0_pixel = (ppu.shift_pattern_lo_ & bit_mux) > 0;
            uint8_t p1_pixel = (ppu.shift_pattern_hi_ & bit_mux) > 0;

            // Combine to form pixel index
            bg_pixel = (p1_pixel << 1) | p0_pixel;

            // Get palette
            uint8_t bg_pal0 = (ppu.shift_attr_lo_ & bit_mux) > 0;
            uint8_t bg_pal1 = (ppu.shift_attr_hi_ & bit_mux) > 0;
            bg_palette = (bg_pal1 << 1) | bg_pal0;
            //bg_palette = 0;

            uint8_t
                color_idx = bg_pixel | (bg_palette << 2);
//                (bg_palette << 4) | 0b1100 ;
            /*
             * Redirect to backdrop color if 2 LSBits of color idx is 0
             */
            color_idx = color_idx * ((color_idx & 0b1) | ((color_idx & 0b10) >> 1));

            uint8_t
                clr = ppu_peripheral_palette.memory_[color_idx  & 0x3F];
            uint16_t
                pixel_idx = ppu.cycle_ - 1 + (uint16_t) ppu.scanline_ * 256;

            ppu.pixels_[pixel_idx] = clr;
        }

        else if(ppu.cycle_ >= 321 && ppu.cycle_ <= 336)
        {
            load_tile();
            if(ppu.cycle_ == 256) inc_y();
        }

        /*
         * Reset x-scroll
         */
        else if(ppu.cycle_ == 257)
            reset_x();

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
