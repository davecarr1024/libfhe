# -*- coding: utf-8 -*-

from Core.Math.BoundedNode2 import BoundedNode2
from Core.Math.Vec2 import Vec2
from Core.Math.Rot2 import Rot2

from OpenGL.GL import *

class SceneNode2(BoundedNode2):
    def msg_render2(self):
        glPushMatrix()
        self.getVar("pos",Vec2()).glTranslate()
        self.getVar("rot",Rot2()).glRotate()
        self.getVar("scale",Vec2(1,1)).glScale()
        
    def unmsg_render2(self):
        glPopMatrix()
