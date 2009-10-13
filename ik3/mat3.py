import math
from vec2 import Vec2
from vec3 import Vec3

class Mat3:
    def __init__(self):
        self.f = [0 for i in range(9)]

    def __getitem__(self, pos):
        assert isinstance(pos,tuple) and len(pos) == 2 and all([isinstance(i,int) for i in pos]), "index mat3 with two ints"
        return self.f[pos[0] * 3 + pos[1]]

    def __setitem__(self, pos, val):
        assert isinstance(pos,tuple) and len(pos) == 2 and all([isinstance(i,int) for i in pos]), "index mat3 with two ints"
        self.f[pos[0] * 3 + pos[1]] = val

    def __repr__(self):
        return "<Mat3 %s>" % self.f

    def __mul__(self, m):
        if isinstance(m,Mat3):
            result = Mat3()
            for i in range(3):
                for j in range(3):
                    for k in range(3):
                        result[i,j] += self[i,k] * m[k,j]
            return result
        elif isinstance(m,Vec2):
            v = self * Vec3(m.x,m.y,1)
            return Vec2(v.x,v.y)
        elif isinstance(m,Vec3):
            v = Vec3()
            for i in range(3):
                for j in range(3):
                    v[i] += self[i,j] * m[j]
            return v
        else:
            raise RuntimeError("unknown type to multiply mat3 by %s" % type(m))

    @staticmethod
    def identity():
        m = Mat3()
        m[0,0] = m[1,1] = m[2,2] = 1
        return m

    @staticmethod
    def translation(v):
        m = Mat3.identity()
        m[0,2] = v.x
        m[1,2] = v.y
        return m

    @staticmethod
    def scale(v):
        m = Mat3()
        m[0,0] = v.x
        m[1,1] = v.y
        m[2,2] = 1
        return m

    @staticmethod
    def rotation(a):
        sa = math.sin(-a)
        ca = math.cos(-a)
        m = Mat3()
        m[0,0] = ca
        m[0,1] = sa
        m[1,0] = -sa
        m[1,1] = ca
        m[2,2] = 1
        return m

    def det(self):
        return self.f[0] * (self.f[4] * self.f[8] - self.f[7] * self.f[5]) -\
               self.f[1] * (self.f[3] * self.f[8] - self.f[6] * self.f[5]) +\
               self.f[2] * (self.f[3] * self.f[7] - self.f[6] * self.f[4])

    def inverse(self):
        det = self.det()
        m = Mat3()
        m.f[0] =  (self.f[4] * self.f[8] - self.f[5] * self.f[7]) / det
        m.f[1] = -(self.f[1] * self.f[8] - self.f[7] * self.f[2]) / det
        m.f[2] =  (self.f[1] * self.f[5] - self.f[4] * self.f[2]) / det
        m.f[3] = -(self.f[3] * self.f[8] - self.f[5] * self.f[6]) / det
        m.f[4] =  (self.f[0] * self.f[8] - self.f[6] * self.f[2]) / det
        m.f[5] = -(self.f[0] * self.f[5] - self.f[3] * self.f[2]) / det
        m.f[6] =  (self.f[3] * self.f[7] - self.f[6] * self.f[4]) / det
        m.f[7] = -(self.f[0] * self.f[7] - self.f[6] * self.f[1]) / det
        m.f[8] =  (self.f[0] * self.f[4] - self.f[1] * self.f[3]) / det
        return m
            
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
