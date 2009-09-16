from body import Body
from core.bb import BB
from core.vec2 import Vec2

import pymunk

class Circle(Body):
    def onAttach(self):
        self.defaultVar("radius",0.5)
        self.defaultVar("mass",1)
        
        Body.onAttach(self)
        
    def makeBody(self):
        if self.staticBody:
            mass = pymunk.inf
            inertia = pymunk.inf
        else:
            mass = self.mass
            inertia = pymunk.moment_for_circle(mass,0,self.radius,(0,0))
        body = pymunk.Body(mass,inertia)
        shape = pymunk.Circle(body,self.radius,(0,0))
        return body, shape

    def getBB(self):
        return BB(Vec2(-self.radius,-self.radius),Vec2(self.radius,self.radius))
