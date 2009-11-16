import math

class Rot:
    def __init__(self, a = 0):
        self.a = a
        
    @staticmethod
    def fromDegrees(f):
        return Rot(math.radians(f))
        
    @staticmethod
    def fromRadians(f):
        return Rot(f)
        
    @staticmethod
    def fromVec(v):
        return Rot(math.atan2(v.y,v.x))
    
    def toVec(self, l = 1):
        return Vec2(l * math.cos(self.a), l * math.sin(self.a))
        
    def __repr__(self):
        return "Rot(%.3f)" % self.a
        
    def __eq__(self, r):
        return abs(self.a - r.a) < 1e-4
        
    def __neg__(self):
        return Rot(-self.a)
        
    def __add__(self, r):
        return Rot(self.a + r.a)
        
    def __sub__(self, r):
        return Rot(self.a - r.a)
        
    def __mul__(self, v):
        if isinstance(v,Vec2):
            return Vec2.fromRot(self + v.toRot(),v.length())
        else:
            return Rot(self.a * v)
        
    def __div__(self, f):
        return Rot(self.a / f)
        
    def norm(self):
        return Rot(math.fmod(self.a,math.pi))
        
    def degrees(self):
        return math.degrees(self.a)
        
    def radians(self):
        return self.a

from Vec2 import Vec2
