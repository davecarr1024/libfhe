# -*- coding: utf-8 -*-

from Vec2 import Vec2

class Box2:
    def __init__(self, min = None, max = None):
        self.min = Vec2(*(min or Vec2()))
        self.max = Vec2(*(max or Vec2()))
        
    @staticmethod
    def compose(v):
        if all([isinstance(i,Box2) for i in v]):
            box = Box2(v[0].min,v[0].max)
            map(box.expand,v[1:])
            return box
        elif all([isinstance(i,Vec2) for i in v]):
            box = Box2(v[0],v[0])
            map(box.expand(v[1:]))
            return box
        else:
            raise TypeError
        
    def expand(self, v):
        if isinstance(v,Vec2):
            self.min.x = min(self.min.x,v.x)
            self.min.y = min(self.min.y,v.y)
            self.max.x = max(self.max.x,v.x)
            self.max.y = max(self.max.y,v.y)
        elif isinstance(v,Box2):
            self.min.x = min(self.min.x,v.min.x)
            self.min.y = min(self.min.y,v.min.y)
            self.max.x = max(self.max.x,v.max.x)
            self.max.y = max(self.max.y,v.max.y)
        else:
            raise TypeError

    def corners(self):
        return [Vec2(self.min.x,self.min.y),
                Vec2(self.min.x,self.max.y),
                Vec2(self.max.x,self.max.y),
                Vec2(self.max.x,self.min.y)]
                
    def transform(self, mat):
        return Box2.compose(map(mat.__mul__,self.corners()))
    
    def overlaps(self, box):
        return self.min.x < box.max.x and self.max.x > box.min.x and self.min.y < box.max.y and self.max.y > box.min.y
