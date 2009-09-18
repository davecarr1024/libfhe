# -*- coding: utf-8 -*-

from SceneNode import SceneNode

from OpenGL.GL import *
from OpenGL.GLUT import *

class Text(SceneNode):
    def geom(self):
        text = self.getVar("text","")
        height = self.getVar("height",1)
        
        font = GLUT_STROKE_ROMAN
        l = glutStrokeLength(font,text)
        h = glutStrokeHeight(font,text)
        r = height / h
        
        glPushMatrix()
        glScalef(l*r,h*r,1)
        glutStrokeString(font,text)
        glPopMatrix()
