#include <iostream>
#include <GL/glew.h>
#include <GL/gl.h>

#include "Display.hpp"

using namespace std;

Display::Display(int width, int height, const string& title) {

    SDL_Init(SDL_INIT_EVERYTHING);

    SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_BUFFER_SIZE, 32);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

    this->window = SDL_CreateWindow(title.c_str(),  SDL_WINDOWPOS_CENTERED,
                                                    SDL_WINDOWPOS_CENTERED,
                                                    width,
                                                    height,
                                                    SDL_WINDOW_OPENGL);

    this->context = SDL_GL_CreateContext(this->window);

    if(glewInit() != GLEW_OK)
        cout << "Error: Could not initialize glew" << endl;

    glViewport(0, 0, (GLsizei) width, (GLsizei) height);

    glMatrixMode(GL_PROJECTION);
    glOrtho(0.0, (GLdouble) width, 0.0, (GLdouble) height, -1.0, 1.0);

    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);

    this->window_is_closed = false;
}

Display::~Display() {
    SDL_GL_DeleteContext(this->context);
    SDL_DestroyWindow(this->window);
    SDL_Quit();
}

void Display::update() {
    SDL_GL_SwapWindow(this->window);

    SDL_Event e;

    while(SDL_PollEvent(&e)) {
        if(e.type == SDL_QUIT) {
            this->window_is_closed = true;
        }
    }
}
    
bool Display::is_closed() {
    return this->window_is_closed;
}

