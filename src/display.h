/*
 * gui.h
 *
 *  Created on: Jun 5, 2020
 *      Author: David Jonsson
 */

#ifndef SRC_DISPLAY_H_
#define SRC_DISPLAY_H_
#define GLEW_STATIC

#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <stdbool.h>

#define SCREEN_W 256
#define SCREEN_H 240

struct display
{
    SDL_Window
        *window_;
    SDL_Event
        event_;
    SDL_Renderer
        *renderer_;

    SDL_Surface
        *surface_;
    SDL_Texture
        *texture_;

} extern display;


void display_init();
bool display_draw();
void display_destroy();
void capture_events();

#endif /* SRC_DISPLAY_H_ */
