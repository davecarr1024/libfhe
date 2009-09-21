# -*- coding: utf-8 -*-

from SceneNode import SceneNode
from Core.Math.Vec2 import Vec2
from Core.Math.Box2 import Box2

from OpenGL.GL import *
import math

class Circle(SceneNode):
    def onAttach(self):
        SceneNode.onAttach(self)
        self.setVar("box",Box2(Vec2(-1,-1),Vec2(1,1)))
    
    def geom(self):
        if self.getVar("filled",True):
            glBegin(GL_TRIANGLE_FAN)
            glTexCoord2f(0.5,0.5)
            glVertex2f(0,0)
        else:
            glBegin(GL_LINE_LOOP)
        
        slices = self.getVar("slices",8)
        for i in range(slices+1):
            th = float(i)/float(slices)*math.pi*2
            x = math.cos(th)
            y = math.sin(th)
            glTexCoord2f((x+1)/2,(y+1)/2)
            glVertex2f(x,y)
        glEnd()

    def collideVec(self, v):
        return v.length() < 1
