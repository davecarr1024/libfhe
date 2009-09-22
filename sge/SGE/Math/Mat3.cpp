#include "Mat3.h"
#include "Vec2.h"
#include "Vec3.h"
#include "Rot.h"
#include <cassert>

namespace SGE
{
    const Mat3 Mat3::ZERO;
    const Mat3 Mat3::IDENTITY(1,0,0,0,1,0,0,0,1);
    
    Mat3::Mat3()
    {
        for (int i = 0; i < 9; ++i) f[i] = !(i % 4);
    }
    
    Mat3::Mat3(float f0, float f1, float f2, float f3, float f4, float f5, float f6, float f7, float f8)
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
    }
    
    Mat3::Mat3(float* _f)
    {
        for (int i = 0; i < 9; ++i) f[i] = _f[i];
    }
    
    Mat3::Mat3(const Mat3& m)
    {
        for (int i = 0; i < 9; ++i) f[i] = m.f[i];
    }
    
    Mat3& Mat3::operator=(const Mat3& m)
    {
        for (int i = 0; i < 9; ++i) f[i] = m.f[i];
        return *this;
    }
    
    float Mat3::get(int i) const
    {
        assert(i >= 0 && i < 9);
        return f[i];
    }
    
    float Mat3::get(int i, int j) const
    {
        assert(i >= 0 && i < 3 && j >= 0 && j < 3);
        return f[i * 3 + j];
    }
    
    void Mat3::set(int i, float val)
    {
        assert(i >= 0 && i < 9);
        f[i] = val;
    }
    
    void Mat3::set(int i, int j, float val)
    {
        assert(i >= 0 && i < 3 && j >= 0 && j < 3);
        f[i * 3 + j] = val;
    }
    
    Mat3 Mat3::operator*(const Mat3& m) const
    {
        Mat3 result;
        int i, j, k;
        for (i = 0; i < 3; ++i)
            for (j = 0; j < 3; ++j)
                for (k = 0; k < 3; ++k)
                    result.set(i, j, result.get(i,j) + get(i,k) * m.get(k,j));
        return result;
    }
    
    Vec2 Mat3::operator*(const Vec2& v) const
    {
        return Vec2(f[0] * v.x + f[1] * v.y + f[2],
                    f[3] * v.x + f[4] * v.y + f[5]);
    }
    
    Vec3 Mat3::operator*(const Vec3& v) const
    {
        return Vec3(f[0] * v.x + f[1] * v.y + f[2] * v.z,
                    f[3] * v.x + f[4] * v.y + f[5] * v.z,
                    f[6] * v.x + f[7] * v.y + f[8] * v.z);
    }
    
    void Mat3::getBasis(Vec3& xAxis, Vec3& yAxis, Vec3& zAxis) const
    {
        xAxis = Vec3(f[0],f[3],f[6]);
        yAxis = Vec3(f[1],f[4],f[7]);
        zAxis = Vec3(f[2],f[5],f[8]);
    }
    
    Mat3 Mat3::translation(const Vec2& v)
    {
        return Mat3(1,0,v.x,
                    0,1,v.y,
                    0,0,1);
    }
    
    Mat3 Mat3::scale(const Vec2& v)
    {
        return Mat3(v.x,0,0,
                    0,v.y,0,
                    0,0,1);
    }
    
    Mat3 Mat3::rotation(const Rot& r)
    {
        float sa = Math::sin(r.angle);
        float ca = Math::cos(r.angle);
        return Mat3(ca,sa,0,
                    -sa,ca,0,
                    0,0,1);
    }
    
    float Mat3::det() const
    {
        return f[0] * (f[4] * f[8] - f[7] * f[5]) -
               f[1] * (f[3] * f[8] - f[6] * f[5]) +
               f[2] * (f[3] * f[7] - f[6] * f[4]);
    }
    
    Mat3 Mat3::inverse() const
    {
        float d = det();
        return Mat3( (f[4] * f[8] - f[5] * f[7]) / d,
                    -(f[1] * f[8] - f[7] * f[2]) / d,
                     (f[1] * f[5] - f[4] * f[2]) / d,
                    -(f[3] * f[8] - f[5] * f[6]) / d,
                     (f[0] * f[8] - f[6] * f[2]) / d,
                    -(f[0] * f[5] - f[3] * f[2]) / d,
                     (f[3] * f[7] - f[6] * f[4]) / d,
                    -(f[0] * f[7] - f[6] * f[1]) / d,
                     (f[0] * f[4] - f[1] * f[3]) / d);
    }
    
    Vec2List Mat3::transform(const Vec2List& vl) const
    {
        Vec2List result;
        for (Vec2List::const_iterator i = vl.begin(); i != vl.end(); ++i)
            result.push_back((*this) * (*i));
        return result;
    }
    
    bool Mat3::equals(const Mat3& m, float eps) const
    {
        bool equal = true;
        for (int i = 0; i < 9 && equal; ++i)
            if (!Math::equal(f[0],m.f[0],eps))
                equal = false;
        return equal;
    }
}
