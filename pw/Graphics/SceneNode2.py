# -*- coding: utf-8 -*-

from Core.SpatialNode2 import SpatialNode2
from Core.Math.Vec2 import Vec2

from OpenGL.GL import *
import math

class SceneNode2(SpatialNode2):
    def msg_render2(self):
        glPushMatrix()
        self.getVar("pos",Vec2()).glTranslate()
        glRotate(math.degrees(self.getVar("rot",0)),0,0,1)
        
    def unmsg_render2(self):
        glPopMatrix()
