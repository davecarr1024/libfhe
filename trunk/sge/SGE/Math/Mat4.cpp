#include "Mat4.h"
#include "Mat3.h"
#include "Vec3.h"
#include "Quat.h"
#include <cassert>

namespace SGE
{

    const Mat4 Mat4::ZERO(0,0,0,0,
                          0,0,0,0,
                          0,0,0,0,
                          0,0,0,0);
    const Mat4 Mat4::IDENTITY(1,0,0,0,
                              0,1,0,0,
                              0,0,1,0,
                              0,0,0,1);

    Mat4::Mat4()
    {
        f[0] = f[5] = f[10] = f[15] = 1;
        f[1] = f[2] = f[3] = f[4] = f[6] = f[7] = f[8] = f[9] = f[11] = f[12] = f[13] = f[14] = 0;
    }
    
    Mat4::Mat4(float* _f)
    {
        for (int i = 0; i < 16; ++i) f[i] = _f[i];
    }
    
    Mat4::Mat4(float f0, float f1, float f2, float f3, 
               float f4, float f5, float f6, float f7, 
               float f8, float f9, float f10, float f11, 
               float f12, float f13, float f14, float f15)
    {
        f[0] = f0;
        f[1] = f1;
        f[2] = f2;
        f[3] = f3;
        f[4] = f4;
        f[5] = f5;
        f[6] = f6;
        f[7] = f7;
        f[8] = f8;
        f[9] = f9;
        f[10] = f10;
        f[11] = f11;
        f[12] = f12;
        f[13] = f13;
        f[14] = f14;
        f[15] = f15;
    }
    
    Mat4::Mat4(const Mat4& m)
    {
        for (int i = 0; i < 16; ++i) f[i] = m.f[i];
    }
    
    Mat4& Mat4::operator=(const Mat4& m)
    {
        for (int i = 0; i < 16; ++i) f[i] = m.f[i];
        return *this;
    }
    
    float Mat4::get(int i) const
    {
        assert(i >= 0 && i < 16);
        return f[i];
    }
    
    float Mat4::get(int i, int j) const
    {
        assert(i >= 0 && i < 4 && j >= 0 && j < 4);
        return f[i * 4 + j];
    }
    
    void Mat4::set(int i, float val)
    {
        assert(i >= 0 && i < 16);
        f[i] = val;
    }
    
    void Mat4::set(int i, int j, float val)
    {
        assert(i >= 0 && i < 4 && j >= 0 && j < 4);
        f[i * 4 + j] = val;
    }
    
    Mat4 Mat4::operator*(const Mat4& m) const
    {
        Mat4 result = Mat4::ZERO;
        int i, j, k;
        for (i = 0; i < 4; ++i)
            for (j = 0; j < 4; ++j)
                for (k = 0; k < 4; ++k)
                    result.set(i, j, result.get(i, j) + get(i,k) * m.get(k,j));
        return result;
    }
    
    Vec3 Mat4::operator*(const Vec3& v) const
    {
        return Vec3(f[0] * v.x + f[1] * v.y + f[2] * v.z + f[3],
                    f[4] * v.x + f[5] * v.y + f[6] * v.z + f[7],
                    f[8] * v.x + f[9] * v.y + f[10] * v.z + f[11]);
    }
    
    Vec3 Mat4::getTranslation() const
    {
        return Vec3(f[3],f[7],f[11]);
    }
    
    Quat Mat4::getRotation() const
    {
        float t = 1 + f[0] + f[5] + f[10];
        if (t > 0.00001)
        {
            float s = Math::sqrt(t) * 2.0;
            return Quat(0.25 * s,
                        (f[9] - f[6]) / s,
                        (f[2] - f[8]) / s,
                        (f[4] - f[1]) / s);
        }
        else if (f[0] > f[5] && f[0] > f[10])
        {
            float s = Math::sqrt(1.0 + f[0] - f[5] - f[10]) * 2.0;
            return Quat((f[9] - f[6]) / s,
                        0.25 * s,
                        (f[4] + f[1]) / s,
                        (f[2] + f[8]) / s);
        }
        else if (f[5] > f[10])
        {
            float s = Math::sqrt(1.0 + f[5] - f[0] - f[10]) * 2.0;
            return Quat((f[2] - f[8]) / s,
                        (f[4] + f[1]) / s,
                        0.25 * s,
                        (f[2] - f[8]) / s);
        }
        else
        {
            float s = Math::sqrt(1.0 + f[10] - f[0] - f[5]) * 2.0;
            return Quat((f[4] - f[1]) / s,
                        (f[2] + f[8]) / s,
                        (f[9] + f[6]) / s,
                        0.25 * s);
        }
    }
    
    Mat3 Mat4::getRotationMat() const
    {
        return Mat3(f[0],f[1],f[2],
                    f[4],f[5],f[6],
                    f[7],f[8],f[9]);
    }
    
    void Mat4::getBasis(Vec3& xAxis, Vec3& yAxis, Vec3& zAxis) const
    {
        xAxis = Vec3(f[0],f[4],f[8]);
        yAxis = Vec3(f[1],f[5],f[9]);
        zAxis = Vec3(f[2],f[6],f[10]);
    }
    
    Mat4 Mat4::translation(const Vec3& v)
    {
        return Mat4(1,0,0,v.x,
                    0,1,0,v.y,
                    0,0,1,v.z,
                    0,0,0,1);
    }
    
    Mat4 Mat4::scale(const Vec3& v)
    {
        return Mat4(v.x,0,0,0,
                    0,v.y,0,0,
                    0,0,v.z,0,
                    0,0,0,1);
    }
    
    Mat4 Mat4::rotation(const Quat& q)
    {
        float xx = q.x * q.x,
              xy = q.x * q.y,
              xz = q.x * q.z,
              xw = q.x * q.w,
              yy = q.y * q.y,
              yz = q.y * q.z,
              yw = q.y * q.w,
              zz = q.z * q.z,
              zw = q.z * q.w;
        return Mat4(1 - 2 * (yy + zz),  //0
                    2 * (xy - zw),      //1
                    2 * (xz + yw),      //2
                    0,                  //3
                    2 * (xy + zw),      //4
                    1 - 2 * (xx + zz),  //5
                    2 * (yz - xw),      //6
                    0,                  //7
                    2 * (xz - yw),      //8
                    2 * (yz + xw),      //9
                    1 - 2 * (xx + yy),  //10
                    0,                  //11
                    0,0,0,1);
    }
    
    Mat3 Mat4::submat(int i, int j) const
    {
        Mat3 m;
        int si, sj, di, dj;
        for (di = 0; di < 3; ++di)
            for (dj = 0; dj < 3; ++dj)
            {
                if (di >= i)
                    si = di + 1;
                else
                    si = di;
                if (dj >= j)
                    sj = dj + 1;
                else
                    sj = dj;
                m.set(di,dj,get(si,sj));
            }
        return m;
    }
    
    float Mat4::det() const
    {
        float result = 0;
        int i = 1;
        for (int n = 0; n < 4; ++n)
        {
            result += f[n*4] * submat(n,0).det() * i;
            i *= -1;
        }
        return result;
    }
    
    Mat4 Mat4::inverse() const
    {
        float d = det();
        if (Math::fabs(d) < 0.00001)
            return Mat4::IDENTITY;
        else
        {
            Mat4 m;
            int i, j, sign;
            for (i = 0; i < 4; ++i)
                for (j = 0; j < 4; ++j) 
                {
                    sign = 1 - ((i + j) % 2) * 2;
                    m.f[i + j*4] = (submat(i,j).det() * sign) / d;
                }
            return m;
        }
    }
    
    void Mat4::transform(Vec3List& vl) const
    {
        for (Vec3List::iterator i = vl.begin(); i != vl.end(); ++i)
            *i = *this * *i;
    }
    
    bool Mat4::equals(const Mat4& m, float eps) const
    {
        bool equal = true;
        for (int i = 0; i < 16 && equal; ++i)
            if (!Math::equal(f[0],m.f[0],eps))
                equal = false;
        return equal;
    }
    
    std::ostream& operator<<(std::ostream& os, const Mat4& m)
    {
        os << "<Mat4\t";
        for (int i = 0; i < 16; ++i)
        {
            if (i && !(i%4))
                os << "\n\t";
            os << m.get(i) << " ";
        }
        os << ">";
        return os;
    }
}
