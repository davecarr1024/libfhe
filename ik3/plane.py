import math
from vec3 import Vec3

class Plane:
    def __init__(self, normal, dist):
        self.normal = normal
        self.dist = dist

    def __repr__(self):
        return "<Plane %s %.3f>" % (self.normal, self.dist)

    @staticmethod
    def fromPointNormal(point, normal):
        return Plane(normal, -normal.dot(point))

    def distToPoint(self, v):
        return (self.normal.dot(v) + self.dist) / self.normal.length()

    def projectPoint(self, v):
        return v + self.normal * -self.distToPoint(v)

if __name__ == "__main__":
    def fequal(f1, f2):
        return abs(f1 - f2) < 0.1
    
    def vequal(v1, v2):
        return fequal(v1.x,v2.x) and fequal(v1.y,v2.y) and fequal(v1.z,v2.z)
    
    def pequal(p1, p2):
        return fequal(p1.dist,p2.dist) and vequal(p1.normal,p2.normal)
    
    assert pequal(Plane.fromPointNormal(Vec3(0,1,0),Vec3(1,0,0)),Plane(Vec3(1,0,0),0))

    p = Plane(Vec3.UNIT_X,0)
    v = Vec3(5,10,-100)

    assert fequal(p.distToPoint(v),5)
    assert vequal(p.projectPoint(v),Vec3(0,10,-100))
