from body import Body
from core.vec2 import Vec2
from core.bb import BB

import pymunk

class Poly(Body):
    def onAttach(self):
        self.defaultVar("vertices",[])
        self.defaultVar("mass",1)
        self.center = None

        Body.onAttach(self)

    def makeBody(self):
        vertices = map(lambda v: v.toTuple(),self.vertices)

        self.center = Vec2(*pymunk.util.calc_center(vertices))
        vertices = map(lambda v: (v-self.center).toTuple(),self.vertices)
        
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
        minx = min([v.x for v in self.vertices])
        maxx = max([v.x for v in self.vertices])
        miny = min([v.y for v in self.vertices])
        maxy = max([v.y for v in self.vertices])
        return BB(Vec2(minx,miny),Vec2(maxx,maxy))

    def transform(self):
        Body.transform(self)
        if self.center:
            (-self.center).translate()
