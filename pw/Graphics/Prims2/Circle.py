# -*- coding: utf-8 -*-

from SceneNode import SceneNode

from OpenGL.GL import *
import math

class Circle(SceneNode):
    def geom(self):
        SpatialNode2.msg_render2()
        
        center = self.getVar("center",False)
        
        if not center:
            glTranslatef(0.5,0.5,0)
        
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

        if not center:
            glTranslatef(-0.5,-0.5,0)
