from body import Body
from core.vec2 import Vec2
from core.bb import BB
import pymunk

class Segment(Body):
    def onAttach(self):
        self.defaultVar("v1",Vec2(0,0))
        self.defaultVar("v2",Vec2(10,0))
        self.defaultVar("radius",0.5)
        self.staticBody = True
        
        Body.onAttach(self)
        
    def makeBody(self):
        body = pymunk.Body(pymunk.inf,pymunk.inf)
        shape = pymunk.Segment(body,self.v1.toTuple(),self.v2.toTuple(),self.radius)
        return body, shape

    def getBB(self):
        minx = min(self.v1.x,self.v2.x)
        maxx = max(self.v1.x,self.v2.x)
        miny = min(self.v1.y,self.v2.y)
        maxy = max(self.v1.y,self.v2.y)
        return BB(Vec2(minx,miny),Vec2(maxx,maxy))
