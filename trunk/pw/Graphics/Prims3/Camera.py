# -*- coding: utf-8 -*-

from SceneNode import SceneNode
from Core.Math.Vec3 import Vec3

from OpenGL.GL import *
from OpenGL.GLU import *

class Camera(SceneNode):
    def transform(self):
        glPushMatrix()
        p = self.getVar("pos",Vec3(0,0,100))
        l = self.getVar("lookAt",Vec3(0,0,0))
        u = self.getVar("up",Vec3(0,1,0))
        gluLookAt(p.x,p.y,p.z,l.x,l.y,l.z,u.x,u.y,u.z)
