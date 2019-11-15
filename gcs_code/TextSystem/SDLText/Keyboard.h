#pragma once
#include <SDL2/SDL.h>
#include <map>
namespace rdt{
    enum class Keyboard{
        A=0,
        B,
        C,
        D,
        E,
        F,
        G,
        H,
        I,
        J,
        K,
        L,
        M,
        N,
        O,
        P,
        Q,
        R,
        S,
        T,
        U,
        V,
        W,
        X,
        Y,
        Z
    };

    static const std::map<int,Keyboard> SDLtoRDT = {
        {SDLK_a,Keyboard::A},
        {SDLK_b,Keyboard::B},
        {SDLK_c,Keyboard::C},
        {SDLK_d,Keyboard::D},
        {SDLK_e,Keyboard::E},
        {SDLK_f,Keyboard::F},
        {SDLK_g,Keyboard::G},
        {SDLK_h,Keyboard::H},
        {SDLK_i,Keyboard::I},
        {SDLK_j,Keyboard::J},
        {SDLK_k,Keyboard::K},
        {SDLK_l,Keyboard::L},
        {SDLK_m,Keyboard::M},
        {SDLK_n,Keyboard::N},
        {SDLK_o,Keyboard::O},
        {SDLK_p,Keyboard::P},
        {SDLK_q,Keyboard::Q},
        {SDLK_r,Keyboard::R},
        {SDLK_s,Keyboard::S},
        {SDLK_t,Keyboard::T},
        {SDLK_u,Keyboard::U},
        {SDLK_v,Keyboard::V},
        {SDLK_w,Keyboard::W},
        {SDLK_x,Keyboard::X},
        {SDLK_y,Keyboard::Y},
        {SDLK_z,Keyboard::Z}
    };
}
