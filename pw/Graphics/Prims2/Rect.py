# -*- coding: utf-8 -*-

from SceneNode import SceneNode

from Core.Math.Vec2 import Vec2
from Core.Math.Box2 import Box2

from OpenGL.GL import *

class Rect(SceneNode):
    def onAttach(self):
        SceneNode.onAttach(self)
        self.setVar("box",Box2(Vec2(0,0),Vec2(1,1)))
    
    def geom(self):
        if self.getVar("filled",True):
            glBegin(GL_QUADS)
        else:
            glBegin(GL_LINE_LOOP)
            
        glTexCoord2f(0,0)
        glVertex2f(0,0)
        glTexCoord2f(0,1)
        glVertex2f(0,1)
        glTexCoord2f(1,1)
        glVertex2f(1,1)
        glTexCoord2f(1,0)
        glVertex2f(1,0)
        glEnd()
        
    def collideVec(self, v):
        return v.x > 0 and v.y > 0 and v.x < 1 and v.y < 1
