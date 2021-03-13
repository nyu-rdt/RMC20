# translation of Keyboard.h
# when the user hits the keyboard, this maps the key by storing it as a bit

import pygame 
from enum import Enum

class Key_library(Enum):
    def __new__(cls):
        value = len(cls.__members__) 
        obj = object.__new__(cls)
        obj._value_ = value
        return obj

    #def _generate_next_value_(name, start, count, last_values):
    #    return count
    A = ()
    B = ()
    C = ()
    D = ()
    E = ()
    F = ()
    G = ()
    H = ()
    I = ()
    J = ()
    K = ()
    L = ()
    M = ()
    N = ()
    O = ()
    P = ()
    Q = ()
    R = ()
    S = ()
    T = ()
    U = ()
    V = ()
    W = ()
    X = ()
    Y = ()
    Z = ()

pygame_to_keys = {
    pygame.K_a: Key_library.A, 
    pygame.K_b: Key_library.B,
    pygame.K_c: Key_library.C,
    pygame.K_d: Key_library.D,
    pygame.K_e: Key_library.E,
    pygame.K_f: Key_library.F,
    pygame.K_g: Key_library.G,
    pygame.K_h: Key_library.H,
    pygame.K_i: Key_library.I,
    pygame.K_j: Key_library.J,
    pygame.K_k: Key_library.K,
    pygame.K_l: Key_library.L,
    pygame.K_m: Key_library.M,
    pygame.K_n: Key_library.N,
    pygame.K_o: Key_library.O,
    pygame.K_p: Key_library.P,
    pygame.K_q: Key_library.Q,
    pygame.K_r: Key_library.R,
    pygame.K_s: Key_library.S,
    pygame.K_t: Key_library.T,
    pygame.K_u: Key_library.U,
    pygame.K_v: Key_library.V,
    pygame.K_w: Key_library.W,
    pygame.K_x: Key_library.X,
    pygame.K_y: Key_library.Y,
    pygame.K_z: Key_library.Z}


