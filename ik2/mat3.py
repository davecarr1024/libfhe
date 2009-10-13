import math
from vec2 import Vec2

class Mat3:
    def __init__(self):
        self.f = [0 for i in range(9)]

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
        return "<Mat3 %s %.2f>" % (self.getTranslation(),math.degrees(self.getRotation()))

    def __mul__(self, m):
        if isinstance(m,Mat3):
            result = Mat3()
            for i in range(3):
                for j in range(3):
                    for k in range(3):
                        result[i,j] += self[i,k] * m[k,j]
            return result
        elif isinstance(m,Vec2):
            v = [0,0,0]
            m = [m[0],m[1],1]
            for i in range(3):
                for j in range(3):
                    v[i] += self[i,j] * m[j]
            return Vec2(v[0],v[1])
        else:
            raise TypeError

    def getTranslation(self):
        return Vec2(self[0,2],self[1,2])

    def getRotation(self):
        return math.asin(self[0,1])

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
        sa = math.sin(a)
        ca = math.cos(a)
        m = Mat3()
        m[0,0] = ca
        m[0,1] = sa
        m[1,0] = -sa
        m[1,1] = ca
        m[2,2] = 1
        return m

    def det(self):
        return self[0] * (self[4] * self[8] - self[7] * self[5]) -\
               self[1] * (self[3] * self[8] - self[6] * self[5]) +\
               self[2] * (self[3] * self[7] - self[6] * self[4])

    def inverse(self):
        det = self.det()
        m = Mat3()
        m[0] =  (self[4] * self[8] - self[5] * self[7]) / det
        m[1] = -(self[1] * self[8] - self[7] * self[2]) / det
        m[2] =  (self[1] * self[5] - self[4] * self[2]) / det
        m[3] = -(self[3] * self[8] - self[5] * self[6]) / det
        m[4] =  (self[0] * self[8] - self[6] * self[2]) / det
        m[5] = -(self[0] * self[5] - self[3] * self[2]) / det
        m[6] =  (self[3] * self[7] - self[6] * self[4]) / det
        m[7] = -(self[0] * self[7] - self[6] * self[1]) / det
        m[8] =  (self[0] * self[4] - self[1] * self[3]) / det
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

    r = 1.5
    assert equal(v.rotate(r),Mat3.rotation(r) * v)

    simple = Mat3.scale(s) * (Mat3.translation(t) * (Mat3.rotation(r) * v))
    compound = (Mat3.scale(s) * Mat3.translation(t) * Mat3.rotation(r)) * v
    assert equal(simple,compound)

    assert equal(v,Mat3.identity().inverse() * v)
    assert equal(v - t, Mat3.translation(t).inverse() * v)
    assert equal(v / s, Mat3.scale(s).inverse() * v)
    assert equal(v.rotate(-r),Mat3.rotation(r).inverse() * v)
