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
#include "cpu.h"

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
                case SDLK_p :
                {
                    cpu_pause = 1 - cpu_pause;
                    break;
                }
                case SDLK_F5 :
                {
                    cpu_save = 1;
                    break;
                }
                case SDLK_F6 :
                {
                    cpu_load = 1;
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
