from SceneNode2 import SceneNode2

from OpenGL.GL import *

class Rect(SceneNode2):
    def __init__(self):
        SceneNode2.__init__(self)

        self.filled = True
    
    def geom(self):
        if self.filled:
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
