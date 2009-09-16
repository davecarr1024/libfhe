# -*- coding: utf-8 -*-
import math
from OpenGL.GL import *

class Vec2:
    def __init__(self, x = 0, y = 0):
        self.x = float(x)
        self.y = float(y)
        
    def __repr__(self):
        return "Vec2(%.3f, %.3f)" % (self.x,self.y)

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

    def __neg__(self):
        return self * -1
        
    def __add__(self, v):
        return Vec2(self.x + v.x, self.y + v.y)
        
    def __sub__(self, v):
        return Vec2(self.x - v.x, self.y - v.y)
        
    def __mul__(self, v):
        if isinstance(v,Vec2):
            return Vec2(self.x * v.x, self.y * v.y)
        else:
            return Vec2(self.x * v, self.y * v)
            
    def __div__(self, v):
        if isinstance(v,Vec2):
            return Vec2(self.x / v.x, self.y / v.y)
        else:
            return Vec2(self.x / v, self.y / v)

    def length(self):
        return math.sqrt(self.x * self.x + self.y * self.y)
        
    def angle(self):
        return math.atan2(self.y,self.x)
        
    def norm(self):
        return self / self.length()
        
    def dot(self, v):
        return self.x * v.x + self.y * v.y
        
    def lerp(self, v, i):
        return self + (v - self) * i

    def rotate(self, a):
        myAngle = self.angle()
        l = self.length()
        return Vec2(l * math.cos(myAngle + a), l * math.sin(myAngle + a))

    def glTranslate(self):
        glTranslatef(self.x,self.y,0)
        
    def glScale(self):
        glScalef(self.x,self.y,1)
        
    def glVertex(self):
        glVertex2f(self.x,self.y)
        
    def glTexCoord(self):
        glTexCoord2f(self.x,self.y)
        
Vec2.UNIT_X = Vec2(1,0)
Vec2.UNIT_Y = Vec2(0,1)
Vec2.ZERO = Vec2(0,0)
        
if __name__ == "__main__":
    def equal(f1, f2):
        return abs(f1 - f2) < 0.1
    
    from random import random
    v1 = Vec2(random(),random())
    v2 = Vec2(random(),random())

    v = v2 - v1
    assert equal(v.x,v2.x-v1.x) and equal(v.y,v2.y-v1.y)
