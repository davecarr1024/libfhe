from sceneNode import SceneNode

from OpenGL.GL import *

class Poly(SceneNode):
    def onAttach(self):
        self.defaultVar("vertices",[])
        self.defaultVar("center",True)
        SceneNode.onAttach(self)

    def geom(self):
        if self.center:
            glTranslatef(0,0,-0.5)

        glBegin(GL_QUAD_STRIP)
        for i, v in enumerate(self.vertices + self.vertices[1:]):
            f = float(i)/float(len(self.vertices))
            glTexCoord2f(f,0)
            glVertex3f(v.x,v.y,0)
            glTexCoord2f(f,1)
            glVertex3f(v.x,v.y,1)
        glEnd()

        glBegin(GL_POLYGON)
        for v in self.vertices:
            glTexCoord2f(v.x,v.y)
            glVertex3f(v.x,v.y,0)
        glEnd()

        glBegin(GL_POLYGON)
        for v in self.vertices:
            glTexCoord2f(v.x,v.y)
            glVertex3f(v.x,v.y,1)
        glEnd()

        if self.center:
            glTranslatef(0,0,0.5)
