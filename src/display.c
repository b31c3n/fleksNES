/*
 * display.c
 *
 *  Created on: Jun 5, 2020
 *      Author: David Jonsson
 */

#include "display.h"
#include "colors.h"
#include "apu.h"
#include "ppu.h"

struct display display;

void display_init()
{
    SDL_Init(SDL_INIT_EVERYTHING);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

    display.window_ = SDL_CreateWindow(
            "OpenGL",
            100,
            100,
            SCREEN_W * 3,
            SCREEN_H * 3,
            SDL_WINDOW_OPENGL);

    display.renderer_ = SDL_CreateRenderer(
            display.window_,
            -1,
            SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    display.texture_ = SDL_CreateTexture(
            display.renderer_,
            SDL_PIXELFORMAT_RGBA32,
            SDL_TEXTUREACCESS_STREAMING,
            SCREEN_W,
            SCREEN_H);

    display.surface_ = SDL_CreateRGBSurfaceWithFormat(
            0,
            SCREEN_W,
            SCREEN_H,
            0,
            SDL_PIXELFORMAT_RGBA32);

}

//static void draw_background(const SDL_PixelFormat *format, Uint32 *pixels)
//{
//    for(size_t tile_idx = 0,
//        pal_shift = 0;
//        tile_idx < 32 * 30;
//        ++tile_idx,
//        pal_shift = (tile_idx / 2) & 0b1,
//        pal_shift += (((tile_idx / 32) & 0b1) << 1)
//    )
//    {
//        uint16_t
//            pattern_idx = ((uint16_t) ppu_peripheral_nametable.memory_[tile_idx]) * 0x10
//                          + 0x1000 * ((bool) (ppu.regs_[PPU_CTRL] & PPU_CTRL_BACKGROUND_ADDR)),
//            attribute = (tile_idx / 4) % (8 * 1) + tile_idx / (32 * 4) * 16 + 0x3C0,
//            palette_idx = ppu_peripheral_nametable.memory_[attribute];
//
//        for(size_t pixel_y = (tile_idx / 32) * 8,
//            cnt_y = 0;
//            cnt_y < 8;
//            ++cnt_y,
//            ++pixel_y)
//        {
//            uint16_t
//                pattern_lsb = pattern_idx + cnt_y,
//                pattern_msb = pattern_idx + cnt_y + 8,
//                low_order_b = (uint16_t) ppu_peripheral_chrrom.memory_[pattern_lsb],
//                high_order_b = ((uint16_t) ppu_peripheral_chrrom.memory_[pattern_msb]) << 1;
//            if(palette_idx)
//            {
//                puts("test");
//            }
//
//            for(size_t cnt_x = 0,
//                pixel_x = (tile_idx % 32) * 8 + 7;
//                cnt_x < 8;
//                ++cnt_x,
//                --pixel_x,
//                low_order_b >>= 1,
//                high_order_b >>= 1)
//            {
//                uint8_t
//                    pal_val = (high_order_b & 0b10) + (low_order_b & 0b01);
//                uint8_t
//                    color_idx = pal_val | (((palette_idx >> (pal_shift * 2)) & 0b11) << 2);
//                if
//                (!((pal_val & 0b01) && (pal_val & 0b10)) && pal_val)
//                {
//                    puts("test");
//                }
//                /*
//                 * Redirect to backdrop color if 2 LSBits of color idx is 0
//                 */
//                color_idx = color_idx * ((color_idx & 0b1) | ((color_idx & 0b10) >> 1));
//
//                uint32_t
//                     color = SDL_MapRGB(
//                                 format,
//                                 COLORS[ppu_peripheral_palette.memory_[color_idx]].r,
//                                 COLORS[ppu_peripheral_palette.memory_[color_idx]].g,
//                                 COLORS[ppu_peripheral_palette.memory_[color_idx]].b);
//
//                pixels[pixel_x + pixel_y * 256] = color;
//            }
//        }
//    }
//}

void capture_events()
{
    while(SDL_PollEvent(&display.event_))
    {
        if(display.event_.type == SDL_KEYDOWN)
        {
            switch (display.event_.key.keysym.sym)
            {
                case SDLK_f :
                {
                    controller_buffer |= 0b10000000;
                    break;
                }
                case SDLK_s :
                {
                    controller_buffer |= 0b01000000;
                    break;
                }
                case SDLK_d :
                {
                    controller_buffer |= 0b00100000;
                    break;
                }
                case SDLK_e :
                {
                    controller_buffer |= 0b00010000;
                    break;
                }
                case SDLK_o :
                {
                    controller_buffer |= 0b00001000;
                    break;
                }
                case SDLK_u :
                {
                    controller_buffer |= 0b00000100;
                    break;
                }
                case SDLK_j :
                {
                    controller_buffer |= 0b00000010;
                    break;
                }
                case SDLK_l :
                {
                    controller_buffer |= 0b00000001;
                    break;
                }
            }
        }

        if(display.event_.type == SDL_KEYUP)
        {
            switch (display.event_.key.keysym.sym)
            {
                case SDLK_f :
                {
                    controller_buffer ^= 0b10000000;
                    break;
                }
                case SDLK_s :
                {
                    controller_buffer ^= 0b01000000;
                    break;
                }
                case SDLK_d :
                {
                    controller_buffer ^= 0b00100000;
                    break;
                }
                case SDLK_e :
                {
                    controller_buffer ^= 0b00010000;
                    break;
                }
                case SDLK_o :
                {
                    controller_buffer ^= 0b00001000;
                    break;
                }
                case SDLK_u :
                {
                    controller_buffer ^= 0b00000100;
                    break;
                }
                case SDLK_j :
                {
                    controller_buffer ^= 0b00000010;
                    break;
                }
                case SDLK_l :
                {
                    controller_buffer ^= 0b00000001;
                    break;
                }
            }
        }
    }
}

bool display_draw()
{
    const SDL_PixelFormat
        *format;
    Uint32
        *pixels,
        format_enum;
    int
        pixelcount,
        pitch,
        rgb32 = SDL_PIXELFORMAT_RGBA32;

    SDL_QueryTexture(
            display.texture_,
            &format_enum,
            NULL,
            NULL,
            NULL);

    format = SDL_AllocFormat(format_enum);

    SDL_LockTexture(
            display.texture_,
            NULL,
            &pixels,
            &pitch);

    //pixelcount = pitch / 4 * SCREEN_H;

//    draw_background(format, pixels);

    for(size_t i = 0; i < 256 * 240; ++i)
    {
        uint8_t
            pal_idx = ppu.pixels_[i];

        uint32_t
             color = SDL_MapRGB(
                         format,
                         COLORS[pal_idx].r,
                         COLORS[pal_idx].g,
                         COLORS[pal_idx].b);

        pixels[i] = color;
    }

    SDL_FreeFormat(format);
    SDL_UnlockTexture(display.texture_);

    SDL_SetRenderDrawColor(display.renderer_, 0xFF, 0xFF, 0xFF, 0xFF);
    SDL_RenderClear(display.renderer_);

    SDL_RenderCopyEx(
            display.renderer_,
            display.texture_,
            NULL,
            NULL,
            0,
            NULL,
            SDL_FLIP_NONE);

    SDL_RenderPresent(display.renderer_);

    return true;
}

void display_destroy()
{
    SDL_FreeSurface(display.surface_);
    SDL_DestroyTexture(display.texture_);
    SDL_DestroyRenderer(display.renderer_);
    SDL_DestroyWindow(display.window_);
    SDL_Quit();
}
