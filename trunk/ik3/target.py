from OpenGL.GL import *
import prims

class Target:
    def __init__(self, pos):
        self.pos = pos

    def render(self):
        glPushMatrix()
        glColor(1,0,0)
        self.pos.translate()
        prims.sphere(0.5,8,8)
        glPopMatrix()
