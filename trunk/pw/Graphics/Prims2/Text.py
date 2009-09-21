# -*- coding: utf-8 -*-

from Core.Math.Vec2 import Vec2

from SceneNode import SceneNode

from OpenGL.GL import *
from OpenGL.GLUT import *

class Text(SceneNode):
    def getScale(self):
        font = GLUT_STROKE_ROMAN
        text = self.getVar("text","")
        size = self.getVar("size",Vec2(1,1))
        textSize = Vec2(sum([glutStrokeWidth(font,ord(c)) for c in text]),glutStrokeHeight(font,text))
        return self.getVar("size",Vec2(1,1)) / textSize
    
    def geom(self):
        font = GLUT_STROKE_ROMAN
        text = self.getVar("text","")
        w,h = self.getScale()
        glPushMatrix()
        glScalef(w,-h,1)
        glTranslatef(0,-glutStrokeHeight(font,text),0)
        glutStrokeString(font,text)
        glPopMatrix()
        
    def collideVec(self, v):
        w,h = self.getScale()
        return v.x > 0 and v.y > 0 and v.x < w and v.y < h
