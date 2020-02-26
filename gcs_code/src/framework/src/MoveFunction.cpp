#include "FunctionList.h"

std::vector<rdt::Keyboard> jumpK = {rdt::Keyboard::E};

std::vector<char> jumpE(const std::vector<bool>& input){
    std::vector<char> out;
    out.push_back(0);
    if (input[0]){out[0] = 1;}
    return out;
}

void jumpD(const std::vector<char>& data){
    if(data[0] == 0){ ROS_INFO("FALL"); }

    if(data[0] == 1){ ROS_INFO("JUMP"); }
}

void jumpS(bool sender){
    ROS_INFO("--Press E for Hops--");
}

void jumpC(bool sender){
    ROS_INFO("--Done with Hops--");
}
