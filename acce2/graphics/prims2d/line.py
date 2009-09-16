from sceneNode import SceneNode
from core.vec2 import Vec2

from OpenGL.GL import *

class Line(SceneNode):
    def onAttach(self):
        SceneNode.onAttach(self)

        self.defaultVar("v1",Vec2())
        self.defaultVar("v2",Vec2())
        
    def geom(self):
        glBegin(GL_LINES)
        self.v1.vertex()
        self.v2.vertex()
        glEnd()
