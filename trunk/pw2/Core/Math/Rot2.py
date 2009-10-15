# -*- coding: utf-8 -*-

import math

from OpenGL.GL import *

class Rot2:
    def __init__(self, angle = 0):
        self.angle = angle
        
    def toVec2(self, l = 1):
        return Vec2(l * math.cos(self.angle),l * math.sin(self.angle))
        
    @staticmethod
    def fromVec2(v):
        return Rot2(math.atan2(v.y,v.x))
        
    def __mul__(self, v):
        if isinstance(v,Vec2):
            return (self + Rot2.fromVec2(v)).toVec2(v.length())
        else:
            return Rot2(self.angle*v)
            
    def __div__(self, v):
        return Rot(self.angle/v)
            
    def __add__(self, v):
        if isinstance(v,Rot2):
            return Rot2(self.angle + v.angle)
        else:
            return Rot2(self.angle + v)
            
    def __sub__(self, v):
        if isinstance(v,Rot2):
            return Rot2(self.angle - v.angle)
        else:
            return Rot2(self.angle - v)
            
    def norm(self):
        return Rot2(math.fmod(self.angle,math.pi))
        
    def degrees(self):
        return math.degrees(self.angle)
    
    def radians(self):
        return self.angle
        
    def glRotate(self):
        glRotate(self.degrees(),0,0,-1)

from Vec2 import Vec2
