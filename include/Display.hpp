#ifndef DISPLAY_H
#define DISPLAY_H

#include <string>
#include <SDL2/SDL.h>

/*
 * Class to setup OpenGL and create context
 */
class Display {

private:
    SDL_Window* window;
    SDL_GLContext context;
    bool window_is_closed;

public:
    Display(int width, int height, const std::string& title);
    ~Display();

    void update();
    bool is_closed();
};

#endif // DISPLAY_H
