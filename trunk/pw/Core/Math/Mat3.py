import math
from Vec2 import Vec2
from Vec3 import Vec3

class Mat3:
    def __init__(self, *f):
        self.f = list(f) or [1,0,0,0,1,0,0,0,1]

    def __getitem__(self, pos):
        if isinstance(pos,tuple):
            return self.f[pos[0] * 3 + pos[1]]
        else:
            return self.f[pos]

    def __setitem__(self, pos, val):
        if isinstance(pos,tuple):
            self.f[pos[0] * 3 + pos[1]] = val
        else:
            self.f[pos] = val
    
    def __repr__(self):
        return "Mat3(%s)" % ', '.join(map(lambda f: "%.3f" % f,tuple(self)))

    def __mul__(self, m):
        if isinstance(m,Mat3):
            result = Mat3.zero()
            for i in range(3):
                for j in range(3):
                    for k in range(3):
                        result[i,j] += self[i,k] * m[k,j]
            return result
        elif isinstance(m,Vec2):
            return Vec2(self[0] * m.x + self[1] * m.y + self[2], 
                        self[3] * m.x + self[4] * m.y + self[5])
        elif isinstance(m,Vec3):
            return Vec3(self[0] * m.x + self[1] * m.y + self[2] * m.z,
                        self[3] * m.x + self[4] * m.y + self[5] * m.z,
                        self[6] * m.x + self[7] * m.y + self[8] * m.z)
        else:
            raise TypeError

    @staticmethod
    def zero():
        return Mat3(0,0,0,0,0,0,0,0,0)

    @staticmethod
    def identity():
        return Mat3(1,0,0,0,1,0,0,0,1)

    @staticmethod
    def translation(v):
        return Mat3(1,0,v.x,
                    0,1,v.y,
                    0,0,1)
    
    @staticmethod
    def scale(v):
        return Mat3(v.x,0,0,
                    0,v.y,0,
                    0,0,1)
    
    @staticmethod
    def rotation(a):
        sa = math.sin(-a)
        ca = math.cos(-a)
        return Mat3(ca,sa,0,
                    -sa,ca,0,
                    0,0,1)

    def det(self):
        return self[0] * (self[4] * self[8] - self[7] * self[5]) -\
               self[1] * (self[3] * self[8] - self[6] * self[5]) +\
               self[2] * (self[3] * self[7] - self[6] * self[4])

    def inverse(self):
        det = self.det()
        return Mat3( (self[4] * self[8] - self[5] * self[7]) / det,
                    -(self[1] * self[8] - self[7] * self[2]) / det,
                     (self[1] * self[5] - self[4] * self[2]) / det,
                    -(self[3] * self[8] - self[5] * self[6]) / det,
                     (self[0] * self[8] - self[6] * self[2]) / det,
                    -(self[0] * self[5] - self[3] * self[2]) / det,
                     (self[3] * self[7] - self[6] * self[4]) / det,
                    -(self[0] * self[7] - self[6] * self[1]) / det,
                     (self[0] * self[4] - self[1] * self[3]) / det )
            
if __name__ == "__main__":
    def equal(v1, v2):
        return abs(v2.x - v1.x) < 0.1 and abs(v2.y - v1.y) < 0.1
    v = Vec2(1,2)

    assert equal(v,Mat3.identity() * v)

    t = Vec2(50,-10)
    assert equal(v + t, Mat3.translation(t) * v)

    s = Vec2(-7,3)
    assert equal(v * s, Mat3.scale(s) * v)

    r = math.pi/2
    assert equal(v.rotate(r),Mat3.rotation(r) * v)
    
    simple = Mat3.scale(s) * (Mat3.translation(t) * (Mat3.rotation(r) * v))
    compound = (Mat3.scale(s) * Mat3.translation(t) * Mat3.rotation(r)) * v
    assert equal(simple,compound)

    assert equal(v,Mat3.identity().inverse() * v)
    assert equal(v - t, Mat3.translation(t).inverse() * v)
    assert equal(v / s, Mat3.scale(s).inverse() * v)
    assert equal(v.rotate(-r),Mat3.rotation(r).inverse() * v)
