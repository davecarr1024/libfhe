# -*- coding: utf-8 -*-
from Body import Body
from Core.Math.Vec2 import Vec2
import pymunk

class Segment(Body):
    def makeBody(self):
        self.setVar("staticBody",True)
        body = pymunk.Body(pymunk.inf,pymunk.inf)
        shape = pymunk.Segment(body,self.getVar("v1",Vec2()),self.getVar("v2",Vec2()),self.getVar("radius",1))
        return body, shape
