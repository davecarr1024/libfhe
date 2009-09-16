# -*- coding: utf-8 -*-

from Vec3 import Vec3

class Box3:
    def __init__(self, min = None, max = None):
        self.min = Vec3(*(min or Vec3()))
        self.max = Vec3(*(max or Vec3()))
        
    def __repr__(self):
        return "Box3(%s, %s)" % (self.min, self.max)
        
    def corners(self):
        return Vec3(self.min.x,self.min.y,self.min.z), \
               Vec3(self.min.x,self.min.y,self.max.z), \
               Vec3(self.min.x,self.max.y,self.max.z), \
               Vec3(self.max.x,self.max.y,self.max.z), \
               Vec3(self.max.x,self.max.y,self.min.z), \
               Vec3(self.max.x,self.min.y,self.min.z), \
               Vec3(self.min.x,self.max.y,self.min.z)
                               
    def transform(self, mat):
        corners = map(mat.__mul__, self.corners())
        box = Box3(corners[0],corners[0])
        box.expand(corners[1:])
        return box

    def expand(self, v):
        if isinstance(v,Box3):
            self.expand(v.corners())
        elif isinstance(v,list):
            map(self.expand,v)
        elif isinstance(v,Vec3):
            self.min.x = min(self.min.x,v.x)
            self.min.y = min(self.min.y,v.y)
            self.min.z = min(self.min.z,v.z)
            self.max.x = max(self.max.x,v.x)
            self.max.y = max(self.max.y,v.y)
            self.max.z = max(self.max.z,v.z)
        else:
            raise TypeError
            
    def overlaps(self, box):
        return self.max.x > box.min.x and self.max.y > box.min.y and self.max.z > box.min.z and \
            self.min.x < box.max.x and self.min.y < box.max.y and self.min.z < box.max.z
