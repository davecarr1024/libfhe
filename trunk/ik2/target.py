from OpenGL.GL import *
import prims

class Target:
    def __init__(self, pos):
        self.pos = pos

    def render(self):
        glColor(1,0,0,0.5)
        glPushMatrix()
        self.pos.translate()
        prims.circle(10,8)
        glPopMatrix()
