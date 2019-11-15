#pragma once
#include <SDL2/SDL.h>
#include "Keyboard.h"
#include "JumpTable.h"

namespace rdt{
    class Manager{
    private:
        int width;
        int height;
        SDL_Window* window;
        SDL_GLContext context;

        unsigned int keyCompressed;

        void gl_setup();
        void handle_key_down(SDL_Keysym* key);
        void handle_key_up(SDL_Keysym* key);

    public:
        /**
        * Initialization of GUI handler class
        */
        Manager(const char* title = "RDT Command List", int width = 640, int height = 480);
        ~Manager();

        void runloop();

        //jin calls this to get and parse data
        std::vector<unsigned char> getCommandData();
        void parseCommandData(std::vector<unsigned char> data);

        FunctionTable commands;
    };
}
