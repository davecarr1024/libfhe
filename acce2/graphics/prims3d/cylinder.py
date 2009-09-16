from sceneNode import SceneNode

import math

from OpenGL.GL import *

class Cylinder(SceneNode):
    def onAttach(self):
        SceneNode.onAttach(self)

        self.defaultVar("center",True)
        self.defaultVar("slices",20)

    def geom(self):
        if self.center:
            glTranslatef(0,0,-0.5)

        th = [float(i)/float(self.slices)*math.pi*2 for i in range(self.slices+1)]
        x = map(math.cos,th)
        z = map(math.sin,th)
        
        glBegin(GL_QUAD_STRIP)
        for i in range(self.slices+1):
            f = float(i)/float(self.slices)
            glTexCoord2f(f,0)
            glVertex3f(x[i],z[i],0)
            glTexCoord2f(f,1)
            glVertex3f(x[i],z[i],1)
        glEnd()

        glBegin(GL_TRIANGLE_FAN)
        glTexCoord2f(0.5,0.5)
        glVertex3f(0,0,0)
        for i in range(self.slices+1):
            glTexCoord2f((x[i]+1)/2,(z[i]+1)/2)
            glVertex3f(x[i],z[i],0)
        glEnd()

        glBegin(GL_TRIANGLE_FAN)
        glTexCoord2f(0.5,0.5)
        glVertex3f(0,1,0)
        for i in range(self.slices+1):
            glTexCoord2f((x[i]+1)/2,(z[i]+1)/2)
            glVertex3f(x[i],z[i],1)
        glEnd()

        if self.center:
            glTranslatef(0,0,0.5)
