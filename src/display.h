/*
 * gui.h
 *
 *  Created on: Jun 5, 2020
 *      Author: David Jonsson
 */

#ifndef SRC_DISPLAY_H_
#define SRC_DISPLAY_H_
#define GLEW_STATIC
#include <GL/glew.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#include <stdbool.h>

struct display
{
    SDL_Window*
        sdl_window_;
    SDL_GLContext
        sdl_context_;
    SDL_Event
        sdl_event_;
} extern display;

struct gl_program
{
    GLuint
        gl_vao_,
        gl_vbo_,
        gl_ebo_,
        gl_program_;
} extern program;

struct gl_shape
{
    float
        *vertices_;
    GLuint
        nr_vertices_,
        nr_elements_,
        *elements_,
        vert_shader_,
        frag_shader_;
    GLchar
        **vert_src_,
        **frag_src_;
} extern rectangle;

void display_init();
bool dispaly_draw();
void display_destroy();

#endif /* SRC_DISPLAY_H_ */
