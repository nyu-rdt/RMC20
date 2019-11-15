#include "Manager.h"

std::vector<unsigned char> movement(std::vector<bool> input){
    std::vector<unsigned char> out;
    out.push_back(0);
    if(input[0]){out[0]|=1;}
    if(input[1]){out[0]|=2;}
    if(input[2]){out[0]|=4;}
    if(input[3]){out[0]|=8;}
    return out;
}

void decoder(std::vector<unsigned char> data){
    if(data[0]&1){printf("W");}
    if(data[0]&2){printf("A");}
    if(data[0]&4){printf("S");}
    if(data[0]&8){printf("D");}
    printf("\n");
}

int main(int argc, char** argv){
    rdt::Manager manager;
    manager.commands.insert(
    movement,decoder,nullptr,
    {rdt::Keyboard::W,rdt::Keyboard::A,rdt::Keyboard::S,rdt::Keyboard::D},
    1
    );
    manager.runloop();
    return 0;
}
