# -*- coding: utf-8 -*-
from Joint import Joint
from Core.Math.Vec2 import Vec2

import pymunk

class PinJoint(Joint):
    def makeJoint(self, a, b):
        return pymunk.PinJoint(a,b,self.getVar("anchor1",Vec2()),self.getVar("anchor2",Vec2()))
