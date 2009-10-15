# -*- coding: utf-8 -*-

from Core.Aspect import Aspect

from OpenGL.GL import *

class Rect(Aspect):
    def msg_render(self):
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
