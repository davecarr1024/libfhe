import math
from Vec3 import Vec3
from Quat import Quat
from Mat3 import Mat3

class Mat4:
    def __init__(self, *f):
        self.f = list(f) or [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]

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
        return "Mat4(%s)" % ', '.join(map(lambda f: "%.3f" % f,tuple(self)))

    def __mul__(self, m):
        if isinstance(m,Mat4):
            result = Mat4.zero()
            for i in range(4):
                for j in range(4):
                    for k in range(4):
                        result[i,j] += self[i,k] * m[k,j]
            return result
        elif isinstance(m,Vec3):
            return Vec3(self[0] * m.x + self[1] * m.y + self[2] * m.z + self[3],
                        self[4] * m.x + self[5] * m.y + self[6] * m.z + self[7],
                        self[8] * m.x + self[9] * m.y + self[10] * m.z + self[11])
        else:
            raise TypeError

    @staticmethod
    def identity():
        return Mat4(1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1)
    
    @staticmethod
    def zero():
        return Mat4(0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0)

    @staticmethod
    def translation(v):
        return Mat4(1,0,0,v.x, 0,1,0,v.y, 0,0,1,v.z, 0,0,0,1)

    @staticmethod
    def scale(v):
        return Mat4(v.x,0,0,0, 0,v.y,0,0, 0,0,v.z,0, 0,0,0,1)

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

    @staticmethod
    def test():
        v = Vec3(10,11,12)
        t = Vec3(5,6,7)
        r = Quat.fromAxisAngle(Vec3.UNIT_X,math.pi/2.0)
        s = Vec3(4,-2,5)

        assert t + v == Vec3(15,17,19)
        assert r * v == Vec3(10,-12,11)
        assert s * v == Vec3(40,-22,60)
        
        mt = Mat4.translation(t)
        mr = Mat4.rotation(r)
        ms = Mat4.scale(s)
        
        assert t + v == mt * v
        assert r * v == mr * v
        assert s * v == ms * v
        
        assert s * ( r * ( t + v ) ) == ms * ( mr * ( mt * v ) )
        assert s * ( r * ( t + v ) ) == ( ms * mr * mt ) * v
        
        mti = mt.inverse()
        mri = mr.inverse()
        msi = ms.inverse()
        
        assert v - t == mti * v
        assert r.inverse() * v == mri * v
        assert v / s == msi * v
        
        assert ( r.inverse() * ( v - t ) ) / s == msi * ( mri * ( mti * v ) )
        assert ( r.inverse() * ( v - t ) ) / s == ( msi * mri * mti ) * v
