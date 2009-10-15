# -*- coding: utf-8 -*-

from Core.Math.SpatialNode2 import SpatialNode2
from Core.Math.Box2 import Box2

class BoundedNode2(SpatialNode2):
    def get_globalBox(self):
        box = self.getVar("box",Box2())
        for ent in self.filterDescendents(lambda ent: ent.hasVar("globalBox")):
            box.expand(ent.getVar("globalBox").transform(self.getRelTransform(ent)))
        return box
        
    def overlaps(self, ent):
        return self.getVar("globalBox").overlaps(ent.getVar("globalBox",Box2()).transform(self.getRelTransform(ent)))
