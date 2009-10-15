# -*- coding: utf-8 -*-

import math

from Vec2 import Vec2
from Rot2 import Rot2

class Mat3:
    def __init__(self, *f):
        self.f = list(f) or [1,0,0,
                             0,1,0,
                             0,0,1]
        
    def __getitem__(self, i):
        if isinstance(i,tuple):
            return self.f[i[0] * 3 + i[1]]
        else:
            return self.f[i]
            
    def __setitem__(self, i, v):
        if isinstance(i,tuple):
            self.f[i[0] * 3 + i[1]] = v
        else:
            self.f[i] = v
            
    def __mul__(self, m):
        if isinstance(m,Mat3):
            result = Mat3.zero()
            for i in range(3):
                for j in range(3):
                    for k in range(3):
                        result[i,j] += self[i,k] * m[k,j]
            return result
        elif isinstance(m,Vec2):
            return Vec2(self[0] * m.x + self[1] * m.y + self[2],
                        self[3] * m.x + self[4] * m.y + self[5])
                        
    @staticmethod
    def zero():
        return Mat3(0,0,0,
                    0,0,0,
                    0,0,0)
        
    @staticmethod
    def identity():
        return Mat3(1,0,0,
                    0,1,0,
                    0,0,1)
        
    @staticmethod
    def translation(v):
        return Mat3(1,0,v.x,
                    0,1,v.y,
                    0,0,1)
                    
    @staticmethod
    def scale(v):
        return Mat3(v.x,0,0,
                    0,v.y,0,
                    0,0,1)
                    
    @staticmethod
    def rotation(a):
        if isinstance(a,Rot2):
            a = a.radians()
        sa = math.sin(-a)
        ca = math.cos(-a)
        return Mat3(ca,sa,0,
                    -sa,ca,0,
                    0,0,1)
                    
    def det(self):
        return self[0] * (self[4] * self[8] - self[7] * self[5]) -\
               self[1] * (self[3] * self[8] - self[6] * self[5]) +\
               self[2] * (self[3] * self[7] - self[6] * self[4])   

    def inverse(self):
        det = self.det()
        return Mat3( (self[4] * self[8] - self[5] * self[7]) / det,
                    -(self[1] * self[8] - self[7] * self[2]) / det,
                     (self[1] * self[5] - self[4] * self[2]) / det,
                    -(self[3] * self[8] - self[5] * self[6]) / det,
                     (self[0] * self[8] - self[6] * self[2]) / det,
                    -(self[0] * self[5] - self[3] * self[2]) / det,
                     (self[3] * self[7] - self[6] * self[4]) / det,
                    -(self[0] * self[7] - self[6] * self[1]) / det,
                     (self[0] * self[4] - self[1] * self[3]) / det )
    