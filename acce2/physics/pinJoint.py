from joint import Joint
from core.vec2 import Vec2

import pymunk

class PinJoint(Joint):
    def onAttach(self):
        self.defaultVar("anchor1",Vec2())
        self.defaultVar("anchor2",Vec2())
        Joint.onAttach(self)
    
    def makeJoint(self, a, b):
        return pymunk.PinJoint(a,b,self.anchor1,self.anchor2)
