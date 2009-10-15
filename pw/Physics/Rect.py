# -*- coding: utf-8 -*-
from Body import Body
from Core.Math.Vec2 import Vec2

import pymunk

class Rect(Body):
    def makeBody(self):
        x,y = self.getVar("size",Vec2(1,1)) / 2.0
        vertices = [pymunk.Vec2d(-x,-y),
                    pymunk.Vec2d(-x,y),
                    pymunk.Vec2d(x,y),
                    pymunk.Vec2d(x,-y)]
        if self.staticBody:
            mass = pymunk.inf
            inertia = pymunk.inf
        else:
            mass = self.getVar("mass",1)
            inertia = pymunk.moment_for_poly(mass,vertices,(0,0))
        body = pymunk.Body(mass,inertia)
        shape = pymunk.Poly(body,vertices,(0,0))
        return body,shape
