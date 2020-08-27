/*
 * display.c
 *
 *  Created on: Jun 5, 2020
 *      Author: David Jonsson
 */

#include "display.h"
#include "colors.h"
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

bool display_draw()
{
    if(SDL_PollEvent(&display.event_))
    {
        if(display.event_.type == SDL_QUIT) return false;
        if(display.event_.type == SDL_KEYUP &&
                display.event_.key.keysym.sym == SDLK_ESCAPE) return false;
    }

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

    pixelcount = pitch / 4 * SCREEN_H;

    for(size_t i = 0; i < pixelcount; ++i)
    {
        uint32_t
            color = SDL_MapRGB(
                        format,
                        COLORS[i % 64].r,
                        COLORS[i % 64].g,
                        COLORS[i % 64].b);

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
