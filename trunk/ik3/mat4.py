import math
from vec3 import Vec3
from quat import Quat
from mat3 import Mat3

class Mat4:
    def __init__(self):
        self.f = [0 for i in range(16)]

    def __getitem__(self, pos):
        if isinstance(pos,tuple):
            return self.f[pos[0] * 4 + pos[1]]
        else:
            return self.f[pos]

    def __setitem__(self, pos, val):
        if isinstance(pos,tuple):
            self.f[pos[0] * 4 + pos[1]] = val
        else:
            self.f[pos] = val

    def __repr__(self):
        return "<Mat4 %s> " % self.f

    def __mul__(self, m):
        if isinstance(m,Mat4):
            result = Mat4()
            for i in range(4):
                for j in range(4):
                    for k in range(4):
                        result[i,j] += self[i,k] * m[k,j]
            return result
        elif isinstance(m,Vec3):
            u = m.x,m.y,m.z,1
            v = [0,0,0,0]
            for i in range(4):
                for j in range(4):
                    v[i] += self[i,j] * u[j]
            return Vec3(v[0],v[1],v[2])
        else:
            raise TypeError

    def getTranslation(self):
        return Vec3(self[0,3],self[1,3],self[2,3])

    def getRotation(self):
        t = 1 + self[0] + self[5] + self[10]
        if t > 1e-5:
            s = math.sqrt(t) * 2
            return Quat(0.25 * s,
                        (self[9] - self[6]) / s,
                        (self[2] - self[8]) / s,
                        (self[4] - self[1]) / s)
        elif self[0] > self[5] and self[0] > self[10]:
            s = math.sqrt(1 + self[0] - self[5] - self[10]) * 2
            return Quat((self[9] - self[6]) / s,
                        0.25 * s,
                        (self[4] + self[1]) / s,
                        (self[2] + self[8]) / s)
        elif self[5] > self[10]:
            s = math.sqrt(1 + self[5] - self[0] - self[10]) * 2
            return Quat((self[2] - self[8]) / s,
                        (self[4] + self[1]) / s,
                        0.25 * s,
                        (self[2] - self[8]) / s)
        else:
            s = math.sqrt(1 + self[10] - self[0] - self[5]) * 2
            return Quat((self[4] - self[1]) / s,
                        (self[2] + self[8]) / s,
                        (self[9] + self[6]) / s,
                        0.25 * s)

    @staticmethod
    def identity():
        m = Mat4()
        m[0,0] = m[1,1] = m[2,2] = m[3,3] = 1
        return m

    @staticmethod
    def translation(v):
        m = Mat4.identity()
        m[0,3] = v.x
        m[1,3] = v.y
        m[2,3] = v.z
        return m

    @staticmethod
    def scale(v):
        m = Mat4()
        m[0,0] = v.x
        m[1,1] = v.y
        m[2,2] = v.z
        m[3,3] = 1
        return m

    @staticmethod
    def rotation(q):
        q = q.inverse()
        xx = q.x * q.x
        xy = q.x * q.y
        xz = q.x * q.z
        xw = q.x * q.w
        yy = q.y * q.y
        yz = q.y * q.z
        yw = q.y * q.w
        zz = q.z * q.z
        zw = q.z * q.w
        m = Mat4()
        m[0] = 1 - 2 * (yy + zz)
        m[4] =     2 * (xy - zw)
        m[8] =     2 * (xz + yw)
        m[1] =     2 * (xy + zw)
        m[5] = 1 - 2 * (xx + zz)
        m[9] =     2 * (yz - xw)
        m[2] =     2 * (xz - yw)
        m[6] =     2 * (yz + xw)
        m[10] = 1 - 2 * (xx + yy)
        m[15] = 1
        return m

    def submat(self, i, j):
        m = Mat3()
        for di in range(3):
            for dj in range(3):
                if di >= i:
                    si = di + 1
                else:
                    si = di
                if dj >= j:
                    sj = dj + 1
                else:
                    sj = dj
                m[di,dj] = self[si,sj]
        return m

    def det(self):
        result = 0.
        i = 1
        for n in range(4):
            m = self.submat(n,0)
            det = m.det()
            result += self[n*4] * det * i
            i *= -1
        return result

    def inverse(self):
        det = self.det()
        m = Mat4()
        if abs(det) < 1e-5:
            print "i"
            return Mat4().identity()
        for i in range(4):
            for j in range(4):
                sign = 1 - ((i + j) % 2) * 2
                subm = self.submat(i,j)
                m[i + j*4] = (subm.det() * sign) / det
        return m

if __name__ == "__main__":
    def equal(v1, v2):
        return abs(v1.x - v2.x) < 0.1 and abs(v1.y - v2.y) < 0.1 and abs(v1.z - v2.z) < 0.1
    
    v = Vec3(10,11,12)
    t = Vec3(5,6,7)
    s = Vec3(4,-2,5)
    r = Quat.fromAxisAngle(Vec3.UNIT_X,1.2)

    assert equal(v,Mat4.identity() * v)
    assert equal(v + t, Mat4.translation(t) * v)
    assert equal(v * s, Mat4.scale(s) * v)
    assert equal(r * v, Mat4.rotation(r) * v)

    simple = Mat4.translation(t) * (Mat4.scale(s) * (Mat4.rotation(r) * v))
    compound = (Mat4.translation(t) * Mat4.scale(s) * Mat4.rotation(r)) * v
    assert equal(simple,compound)

    assert equal(v,Mat4().identity().inverse() * v)
    assert equal(v - t, Mat4.translation(t).inverse() * v)
    assert equal(v / s, Mat4.scale(s).inverse() * v)
    assert equal(r.inverse() * v, Mat4.rotation(r).inverse() * v)

    q = Mat4.rotation(r).getRotation()
    rAxis, rAngle = r.toAxisAngle()
    qAxis, qAngle = q.toAxisAngle()
    assert abs(rAngle - qAngle) < 1e-3 and equal(rAxis,qAxis)

    assert equal(t,Mat4.translation(t).getTranslation())
