# -*- coding: utf-8 -*-
from SceneNode import SceneNode
from Core.Math.Vec2 import Vec2

from OpenGL.GL import *

class Line(SceneNode):
    def geom(self):
        glBegin(GL_LINE_STRIP)
        for v in self.getVar("vertices",[]):
            v.glVertex()
        glEnd()
