# translation of Keyboard.h
# when the user hits the keyboard, this maps the key by storing it as a bit

import pygame
from enum import Enum, auto

class Key_library(Enum):
    def _generate_next_value_(name, start, count, last_values):
        return count
    A = auto()
    B = auto()
    C = auto()
    D = auto()
    E = auto()
    F = auto()
    G = auto()
    H = auto()
    I = auto()
    J = auto()
    K = auto()
    L = auto()
    M = auto()
    N = auto()
    O = auto()
    P = auto()
    Q = auto()
    R = auto()
    S = auto()
    T = auto()
    U = auto()
    V = auto()
    W = auto()
    X = auto()
    Y = auto()
    Z = auto()

pygames_to_keys = {
    pygames.K_a: Key_library.A, 
    pygames.K_b: Key_library.B,
    pygames.K_c: Key_library.C,
    pygames.K_d: Key_library.D,
    pygames.K_e: Key_library.E,
    pygames.K_f: Key_library.F,
    pygames.K_g: Key_library.G,
    pygames.K_h: Key_library.H,
    pygames.K_i: Key_library.I,
    pygames.K_j: Key_library.J,
    pygames.K_k: Key_library.K,
    pygames.K_l: Key_library.L,
    pygames.K_m: Key_library.M,
    pygames.K_n: Key_library.N,
    pygames.K_o: Key_library.O,
    pygames.K_p: Key_library.P,
    pygames.K_q: Key_library.Q,
    pygames.K_r: Key_library.R,
    pygames.K_s: Key_library.S,
    pygames.K_t: Key_library.T,
    pygames.K_u: Key_library.U,
    pygames.K_v: Key_library.V,
    pygames.K_w: Key_library.W,
    pygames.K_x: Key_library.X,
    pygames.K_y: Key_library.Y,
    pygames.K_z: Key_library.Z}


