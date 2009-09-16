from body import Body
from core.vec2 import Vec2
from core.bb import BB

import pymunk

class Rect(Body):
    def onAttach(self):
        self.defaultVar("bodySize",Vec2(1,1))
        self.defaultVar("mass",1)

        Body.onAttach(self)

    def makeBody(self):
        x,y = (self.bodySize / 2.0).toTuple()
        vertices = [pymunk.Vec2d(-x,-y),
                    pymunk.Vec2d(-x,y),
                    pymunk.Vec2d(x,y),
                    pymunk.Vec2d(x,-y)]
        if self.staticBody:
            mass = pymunk.inf
            inertia = pymunk.inf
        else:
            mass = self.mass
            inertia = pymunk.moment_for_poly(mass,vertices,(0,0))
        body = pymunk.Body(mass,inertia)
        shape = pymunk.Poly(body,vertices,(0,0))
        return body,shape

    def getBB(self):
        x,y = (self.bodySize / 2.0).toTuple()
        return BB(Vec2(-x,-y),Vec2(x,y))
