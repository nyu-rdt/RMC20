#pragma once
#include "Keyboard.h"
#include "ros/ros.h"
#include <vector>

extern std::vector<rdt::Keyboard> jumpK;

std::vector<char> jumpE(const std::vector<bool>& input);
void jumpD(const std::vector<char>& data);
void jumpS(bool sender);
void jumpC(bool sender);
