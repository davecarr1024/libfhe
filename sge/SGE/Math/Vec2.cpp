#include "Vec2.h"
#include "Rot.h"
#include <cassert>

namespace SGE
{
    const Vec2 Vec2::UNIT_X(1,0);
    const Vec2 Vec2::UNIT_Y(0,1);
    const Vec2 Vec2::ZERO(0,0);
    
    Vec2::Vec2() :
        x(0),
        y(0)
    {
    }
    
    Vec2::Vec2(float _x, float _y) :
        x(_x),
        y(_y)
    {
    }
    
    Vec2::Vec2(const Rot& r, float length)
    {
        x = Math::cos(r.angle) * length;
        y = Math::sin(r.angle) * length;
    }
    
    Vec2::Vec2(const Vec2& v) :
        x(v.x),
        y(v.y)
    {
    }
    
    Vec2& Vec2::operator=(const Vec2& v)
    {
        x = v.x;
        y = v.y;
        return *this;
    }
    
    float& Vec2::operator[](int i)
    {
        assert(i == 0 || i == 1);
        if (i == 0)
            return x;
        else
            return y;
    }
    
    Vec2 Vec2::operator+(const Vec2& v) const
    {
        return Vec2(x + v.x, y + v.y);
    }
    
    Vec2 Vec2::operator-(const Vec2& v) const
    {
        return Vec2(x - v.x, y - v.y);
    }
    
    Vec2 Vec2::operator*(float f) const
    {
        return Vec2(x * f, y * f);
    }
    
    Vec2 Vec2::operator/(float f) const
    {
        assert(f != 0);
        return Vec2(x / f, y / f);
    }
    
    Vec2 Vec2::operator-() const
    {
        return Vec2(-x,-y);
    }
    
    void Vec2::operator+=(const Vec2& v)
    {
        x += v.x;
        y += v.y;
    }
    
    void Vec2::operator-=(const Vec2& v)
    {
        x -= v.x;
        y -= v.y;
    }
    
    void Vec2::operator*=(float f)
    {
        x *= f;
        y *= f;
    }
    
    void Vec2::operator/=(float f)
    {
        assert(f != 0);
        x /= f;
        y /= f;
    }
    
    float Vec2::length() const
    {
        return Math::sqrt(x * x + y * y);
    }
    
    Vec2 Vec2::norm() const
    {
        return *this / length();
    }
    
    float Vec2::dot(const Vec2& v) const
    {
        return x * v.x + y * v.y;
    }
    
    Rot Vec2::getRotTo(const Vec2& v) const
    {
        return (Rot(v) - Rot(*this)).norm();
    }
    
    Vec2 Vec2::lerp(const Vec2& v, float i) const
    {
        return Vec2((v.x - x) * i + x, (v.y - y) * i + y);
    }
    
    bool Vec2::equals(const Vec2& v, float eps) const
    {
        return Math::equal(x,v.x,eps) && Math::equal(y,v.y,eps);
    }
}
