#include "Manager.h"

void rdt::Manager::handle_key_down(SDL_Keysym* key){
    std::map<int,Keyboard>::const_iterator iter = SDLtoRDT.find(key->sym);
    if(iter != SDLtoRDT.end()){
        keyCompressed |= 1<<((int)iter->second);
    }
    //printf("%08x\n",keyCompressed);
}

void rdt::Manager::handle_key_up(SDL_Keysym* key){
    std::map<int,Keyboard>::const_iterator iter = SDLtoRDT.find(key->sym);
    if(iter != SDLtoRDT.end()){
        keyCompressed &= ~(1<<((int)iter->second));
    }
    //printf("%08x\n",keyCompressed);

}

rdt::Manager::Manager(const char* title, int width, int height):width(width),height(height){
    if(SDL_Init( SDL_INIT_VIDEO ) < 0) {
        printf( "SDL could not initialize! SDL_Error: %s\n", SDL_GetError() );
        exit(0);
    }
    window = SDL_CreateWindow( title, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, SDL_WINDOW_SHOWN );
    if(window == NULL) {
        printf( "Window could not be created! SDL_Error: %s\n", SDL_GetError());
        exit(0);
    }
    context = SDL_GL_CreateContext(window);
    keyCompressed = 0U;

}

rdt::Manager::~Manager(){
    SDL_DestroyWindow(window);
    SDL_Quit();
}

void rdt::Manager::runloop(){
    printf("starting:\n");
    while(1){
        SDL_Event event;

        /* Grab all the events off the queue. */
        while( SDL_PollEvent( &event ) ) {
            switch( event.type ) {
            case SDL_KEYDOWN:
                /* Handle key presses. */
                handle_key_down(&event.key.keysym);
                break;
            case SDL_KEYUP:
                /* Handle key presses. */
                handle_key_up(&event.key.keysym);
                break;
            case SDL_QUIT:
                /* exits loop */
                return;
            }
        }
    }
}

std::vector<unsigned char> rdt::Manager::getCommandData(){
    return commands.update(keyCompressed,SDL_GetTicks());
}

void rdt::Manager::parseCommandData(std::vector<unsigned char> data){
    commands.parse(data);
}
