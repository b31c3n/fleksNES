/*
 * display.c
 *
 *  Created on: Jun 5, 2020
 *      Author: David Jonsson
 */

#include "display.h"

const char* vert_src =
"#version 150 core\n"

"in vec2 position;"

"void main()"
"{"
    "gl_Position = vec4(position, 0.0, 1.0);"
"}";

const char* frag_src =
"#version 150 core\n"

"out vec4 outColor;"

"void main()"
"{"
"    outColor = vec4(1.0, 1.0, 1.0, 1.0);"
"}";


struct display display;
struct gl_program program;

GLfloat rect_vertices[] =
{
        -1.0, 1.0,
        -1.0, -1.0,
        1.0,  1.0,
        1.0,  -1.0
};

GLuint rect_elements[] =
{
        0, 1, 2,
        2, 3, 1
};

struct gl_shape rectangle =
{
        .vertices_ = rect_vertices,
        .elements_ = rect_elements,
        .nr_vertices_ = 8,
        .nr_elements_ = 6,
        .vert_src_ = &vert_src,
        .frag_src_ = &frag_src,
};

void create_shader(
        GLuint shader_id,
        char **shader_src,
        GLuint program_id,
        GLenum shader_type)
{
    shader_id = glCreateShader(shader_type);
    glShaderSource(shader_id, 1, shader_src, NULL);
    glCompileShader(shader_id);

    GLint status;
    glGetShaderiv(shader_id, GL_COMPILE_STATUS, &status);

    if(status != GL_TRUE)
    {
        char buffer[512];
        glGetShaderInfoLog(shader_id, 512, NULL, buffer);
        puts(buffer);
    }
    glAttachShader(program_id, shader_id);
}

void display_init()
{
    SDL_Init(SDL_INIT_EVERYTHING);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

    display.sdl_window_ = SDL_CreateWindow("OpenGL", 100, 100, 800, 600, SDL_WINDOW_OPENGL);
    display.sdl_context_ = SDL_GL_CreateContext(display.sdl_window_);

    glewExperimental = GL_TRUE;
    glewInit();

    /**
     * Buffers
     */
    glGenVertexArrays(1, &program.gl_vao_);
    glBindVertexArray(program.gl_vao_);
    glGenBuffers(1, &program.gl_vbo_);
    glGenBuffers(1, &program.gl_ebo_);
    glBindBuffer(GL_ARRAY_BUFFER, program.gl_vbo_);
    glBufferData(
            GL_ARRAY_BUFFER,
            rectangle.nr_vertices_ * sizeof(GLfloat),
            rectangle.vertices_,
            GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, program.gl_ebo_);
    glBufferData(
            GL_ELEMENT_ARRAY_BUFFER,
            rectangle.nr_elements_ * sizeof(GLfloat),
            rectangle.elements_,
            GL_STATIC_DRAW);

    /*
     * Shader compile/linking stuff
     */
    program.gl_program_ = glCreateProgram();
    create_shader(
            rectangle.vert_shader_,
            rectangle.vert_src_,
            program.gl_program_,
            GL_VERTEX_SHADER);
    create_shader(
            rectangle.frag_shader_,
            rectangle.frag_src_,
            program.gl_program_,
            GL_FRAGMENT_SHADER);

    glLinkProgram(program.gl_program_);
    glUseProgram(program.gl_program_);

    GLint isLinked = 0;
    glGetProgramiv(program.gl_program_, GL_LINK_STATUS, &isLinked);
    if (isLinked == GL_FALSE)
    {
        GLint maxLength = 0;
        glGetProgramiv(program.gl_program_, GL_INFO_LOG_LENGTH, &maxLength);
        char buffer[512];
        glGetProgramInfoLog(program.gl_program_, 512, &maxLength, buffer);
        glDeleteProgram(program.gl_program_);
        puts(buffer);
    }



    GLint posAttrib = glGetAttribLocation(program.gl_program_, "position");
    glVertexAttribPointer(posAttrib, 2, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(posAttrib);
}

bool display_draw()
{
    if(SDL_PollEvent(&display.sdl_event_))
    {
        if(display.sdl_event_.type == SDL_QUIT) return false;
        if(display.sdl_event_.type == SDL_KEYUP &&
                display.sdl_event_.key.keysym.sym == SDLK_ESCAPE) return false;
    }
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    SDL_GL_SwapWindow(display.sdl_window_);
    return true;
}

void display_destroy()
{
    SDL_GL_DeleteContext(display.sdl_context_);
    SDL_Quit();
}
