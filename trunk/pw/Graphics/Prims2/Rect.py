# -*- coding: utf-8 -*-

from SceneNode import SceneNode

from OpenGL.GL import *

class Rect(SceneNode):
    def geom(self):
        center = self.getVar("center",False)
        if center:
            glTranslatef(-0.5,-0.5,0)
            
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
        
        if center:
            glTranslatef(0.5,0.5,0)
