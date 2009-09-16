from core.object import Object
from physics.rect import Rect
from graphics.prims3d.cube import Cube
from core.vec2 import Vec2
from core.vec3 import Vec3

class Block(Rect):
    def onAttach(self):
        self.defaultVar("material",{})
        self.defaultVar("stretchMaterial",True)
        self.defaultVar("staticBody",True)
        self.defaultVar("friction",1)

        Rect.onAttach(self)

        scale = Vec3(self.bodySize.x,self.bodySize.y,1)
        if self.stretchMaterial:
            self.material['scale'] = self.bodySize
        self.graphics = Cube(parent = self,
                             vars = dict(scale = scale,
                                         material = self.material))
