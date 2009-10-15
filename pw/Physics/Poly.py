# -*- coding: utf-8 -*-
from Body import Body
from Core.Math.Vec2 import Vec2

import pymunk

class Poly(Body):
    def makeBody(self):
        vertices = self.getVar("vertices",[])

        self.center = Vec2(*pymunk.util.calc_center(vertices))
        vertices = map(lambda v: v-self.center,vertices)
        
        if self.getVar("staticBody",False):
            mass = pymunk.inf
            inertia = pymunk.inf
        else:
            mass = self.getVar("mass",1)
            inertia = pymunk.moment_for_poly(mass,vertices,(0,0))
        body = pymunk.Body(mass,inertia)
        shape = pymunk.Poly(body,vertices,(0,0))
        return body,shape

    def transform(self):
        Body.transform(self)
        if hasattr(self,"center"):
            (-self.center).translate()
