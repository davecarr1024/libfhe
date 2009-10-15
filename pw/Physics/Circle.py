# -*- coding: utf-8 -*-
from Body import Body
from Core.Math.Vec2 import Vec2

import pymunk

class Circle(Body):
    def makeBody(self):
        radius = self.getVar("radius",1)
        if self.getVar("staticBody",False):
            mass = pymunk.inf
            inertia = pymunk.inf
        else:
            mass = self.mass
            inertia = pymunk.moment_for_circle(mass,0,radius,(0,0))
        body = pymunk.Body(mass,inertia)
        shape = pymunk.Circle(body,radius,(0,0))
        return body, shape
