# -*- coding: utf-8 -*-

from Core.Math.Vec2 import Vec2

from SceneNode import SceneNode

from OpenGL.GL import *
from OpenGL.GLUT import *

class Text(SceneNode):
    def getScale(self):
        font = GLUT_STROKE_ROMAN
        text = self.getVar("text","")
        height = self.getVar("height",1)
        h = glutStrokeHeight(font,text)
        return height/h*self.getVar("wscale",0.5),height/h
    
    def geom(self):
        font = GLUT_STROKE_ROMAN
        text = self.getVar("text","")
        w,h = self.getScale()
        
        textw = sum([glutStrokeWidth(font,ord(c)) for c in text])
        align = self.getVar("align","left")
        if align == "left":
            x = 0
        elif align == "center":
            x = -textw/2
        else:
            x = -textw
            
        glPushMatrix()
        glScalef(w,-h,1)
        glTranslatef(x,-glutStrokeHeight(font,text),0)
        glutStrokeString(font,text)
        glPopMatrix()
        
    def collideVec(self, v):
        w,h = self.getScale()
        return v.x > 0 and v.y > 0 and v.x < w and v.y < h
