# -*- coding: utf-8 -*-

from Vec2 import Vec2

class Box2:
    def __init__(self, min = None, max = None):
        self.min = Vec2(*(min or Vec2()))
        self.max = Vec2(*(max or Vec2()))
        
    def __repr__(self):
        return "Box2(%s, %s)" % (self.min,self.max)
        
    def corners(self):
        minx,miny = self.min
        maxx,maxy = self.max
        return Vec2(minx,miny), Vec2(minx,maxy), Vec2(maxx,maxy), Vec2(maxx,miny)
        
    def transform(self, mat):
        corners = map(mat.__mul__, self.corners())
        box = Box2(corners[0],corners[0])
        box.expand(corners[1:])
        return box
        
    def expand(self, x):
        if isinstance(x,list):
            map(self.expand,x)
        elif isinstance(x,Box2):
            self.expand(x.corners())
        elif isinstance(x,Vec2):
            self.min.x = min(self.min.x,x.x)
            self.min.y = min(self.min.y,x.y)
            self.max.x = max(self.max.x,x.x)
            self.max.y = max(self.max.y,x.y)
        else:
            raise TypeError
            
    def overlaps(self, box):
        return self.max.x > box.min.x and self.max.y > box.min.y and self.min.x < box.max.x and self.min.y < box.max.y
