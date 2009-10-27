# -*- coding: utf-8 -*-

import math

from OpenGL.GL import *

class Vec2:
    def __init__(self, x = 0, y = 0):
        self.x = x
        self.y = y
        
    def toRot2(self):
        return Rot2(math.atan2(self.y,self.x))
        
    @staticmethod
    def fromRot2(r, l = 1):
        return Vec2(l * math.cos(r.angle), l * math.sin(r.angle))
        
    def __repr__(self):
        return "Vec2(%.2f,%.2f)" % (self.x,self.y)
        
    def __getitem__(self, i):
        if i == 0:
            return self.x
        elif i == 1:
            return self.y
        else:
            raise IndexError
            
    def __setitem__(self, i, v):
        if i == 0:
            self.x = v
        elif i == 1:
            self.y = v
        else:
            raise IndexError
            
    def __add__(self, v):
        if isinstance(v,Vec2):
            return Vec2(self.x+v.x,self.y+v.y)
        else:
            return Vec2(self.x+v,self.y+v)
            
    def __sub__(self, v):
        if isinstance(v,Vec2):
            return Vec2(self.x-v.x,self.y-v.y)
        else:
            return Vec2(self.x-v,self.y-v)
            
    def __mul__(self, v):
        if isinstance(v,Vec2):
            return Vec2(self.x*v.x,self.y*v.y)
        elif isinstance(v,Rot2):
            return Vec2.fromRot2(self.toRot2() + v,self.length())
        else:
            return Vec2(self.x*v,self.y*v)
            
    def __div__(self, v):
        if isinstance(v,Vec2):
            return Vec2(self.x/v.x,self.y/v.y)
        else:
            return Vec2(self.x/v,self.y/v)
            
    def length(self):
        return math.sqrt(self.x * self.x + self.y * self.y)
        
    def norm(self):
        return self / self.length()
        
    def dot(self, v):
        return self.x * v.x + self.y * v.y
        
    def lerp(self, v, i):
        return self + (v - self) * i
        
    def glTranslate(self):
        glTranslate(self.x,self.y,0)
        
    def glScale(self):
        glScale(self.x,self.y,1)

from Rot2 import Rot2
