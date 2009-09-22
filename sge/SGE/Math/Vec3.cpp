#include "Vec3.h"
#include "Quat.h"
#include <cassert>

namespace SGE
{
    
    const Vec3 Vec3::UNIT_X(1,0,0);
    const Vec3 Vec3::UNIT_Y(0,1,0);
    const Vec3 Vec3::UNIT_Z(0,0,1);
    const Vec3 Vec3::ZERO;
    
    Vec3::Vec3() :
        x(0),
        y(0),
        z(0)
    {
    }
    
    Vec3::Vec3(float _x, float _y, float _z) :
        x(_x),
        y(_y),
        z(_z)
    {
    }
    
    Vec3::Vec3(const Quat& q, float length)
    {
        *this = (q * Vec3::UNIT_X) * length;
    }
    
    Vec3::Vec3(const Vec3& v) :
        x(v.x),
        y(v.y),
        z(v.z)
    {
    }
    
    Vec3& Vec3::operator=(const Vec3& v)
    {
        x = v.x;
        y = v.y;
        z = v.z;
        return *this;
    }
    
    float& Vec3::operator[](int i)
    {
        assert(i >= 0 && i < 3);
        if (i == 0) 
            return x;
        else if (i == 1)
            return y;
        else
            return z;
    }
    
    float Vec3::operator[](int i) const
    {
        assert(i >= 0 && i < 3);
        if (i == 0) 
            return x;
        else if (i == 1)
            return y;
        else
            return z;
    }
    
    Vec3 Vec3::operator+(const Vec3& v) const
    {
        return Vec3(x + v.x, y + v.y, z + v.z);
    }
    
    Vec3 Vec3::operator-(const Vec3& v) const
    {
        return Vec3(x - v.x, y - v.y, z - v.z);
    }
    
    Vec3 Vec3::operator*(float f) const
    {
        return Vec3(x * f, y * f, z * f);
    }
    
    Vec3 Vec3::operator/(float f) const
    {
        assert(f != 0);
        return Vec3(x / f, y / f, z / f);
    }
    
    Vec3 Vec3::operator-() const
    {
        return Vec3(-x,-y,-z);
    }
    
    void Vec3::operator+=(const Vec3& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
    }
    
    void Vec3::operator-=(const Vec3& v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
    }
    
    void Vec3::operator*=(float f)
    {
        x *= f;
        y *= f;
        z *= f;
    }
    
    void Vec3::operator/=(float f)
    {
        assert(f != 0);
        x /= f;
        y /= f;
        z /= f;
    }
    
    float Vec3::length() const
    {
        return Math::sqrt(x * x + y * y + z * z);
    }
    
    Vec3 Vec3::norm() const
    {
        return *this / length();
    }
    
    float Vec3::dot(const Vec3& v) const
    {
        return x * v.x + y * v.y + z * v.z;
    }
    
    Vec3 Vec3::cross(const Vec3& v) const
    {
        return Vec3(y * v.z - z * v.y,
                    z * v.x - x * v.z,
                    x * v.y - y * v.x);
    }
    
    Vec3 Vec3::project(const Vec3& v) const
    {
        float l = length();
        assert(l != 0);
        return *this * (dot(v) / (l * l));
    }
    
    Quat Vec3::getRotTo(const Vec3& v) const
    {
        float d = dot(v);
        if (d > 0.99999)
            return Quat();
        else if (d < -0.99999)
            return Quat(makePerp(), Math::PI);
        else
            return Quat(cross(v).norm(), Math::acos(norm().dot(v.norm())));
    }
    
    Vec3 Vec3::makePerp() const
    {
        if (dot(Vec3::UNIT_Y) < 0.75)
            return cross(Vec3::UNIT_Y);
        else
            return cross(Vec3::UNIT_Z);
    }
    
    Vec3 Vec3::lerp(const Vec3& v, float i) const
    {
        return Vec3((v.x - x) * i + x,
                    (v.y - y) * i + y,
                    (v.z - z) * i + z);
    }
    
    bool Vec3::equals(const Vec3& v, float eps) const
    {
        return Math::equal(x,v.x,eps) && Math::equal(y,v.y,eps) && Math::equal(z,v.z,eps);
    }
    
    std::ostream& operator<<(std::ostream& os, const Vec3& v)
    {
        os << "<Vec3 " << v.x << " " << v.y << " " << v.z << ">";
        return os;
    }
}
