#include "Quat.h"
#include "Vec3.h"
#include <cassert>

namespace SGE
{
    
    const Quat Quat::IDENTITY(1,0,0,0);
    
    Quat::Quat() :
        w(1),
        x(0),
        y(0),
        z(0)
    {
    }
    
    Quat::Quat(float _w, float _x, float _y, float _z) :
        w(_w),
        x(_x),
        y(_y),
        z(_z)
    {
    }
    
    Quat::Quat(const Vec3& axis, float angle)
    {
        Vec3 axisNorm = axis.norm();
        float sa = Math::sin(angle/2.0);
        float ca = Math::cos(angle/2.0);
        *this = Quat(ca, axisNorm.x * sa, axisNorm.y * sa, axisNorm.z * sa);
    }
    
    Quat::Quat(const Quat& q) :
        w(q.w),
        x(q.x),
        y(q.y),
        z(q.z)
    {
    }
    
    Quat& Quat::operator=(const Quat& q)
    {
        w = q.w;
        x = q.x;
        y = q.y;
        z = q.z;
        return *this;
    }
    
    void Quat::toAxisAngle(Vec3& axis, float& angle) const
    {
        Quat q = norm();
        float ca = q.w;
        angle = Math::acos(ca) * 2.0;
        float sa = Math::sqrt(1.0 - ca * ca);
        if (Math::fabs(sa) < 0.00001)
            sa = 1;
        axis = Vec3(q.x / sa, q.y / sa, q.z /sa);
    }
    
    Quat Quat::operator*(const Quat& q) const
    {
        return Quat(w * q.w - x * q.x - y * q.y - z * q.z,
                    w * q.x + x * q.w + y * q.z - z * q.y,
                    w * q.y + y * q.w + z * q.x - x * q.z,
                    w * q.z + z * q.w + x * q.y - y * q.x);
    }
    
    Vec3 Quat::operator*(const Vec3& v) const
    {
        Quat q = *this * Quat(0,v.x,v.y,v.z) * inverse();
        return Vec3(q.x,q.y,q.z);
    }
    
    Quat Quat::operator*(float f) const
    {
        return Quat(w * f, x * f, y * f, z * f);
    }
    
    Quat Quat::operator/(float f) const
    {
        assert(f != 0);
        return Quat(w / f, x / f, y / f, z / f);
    }
    
    Quat Quat::conjugate() const
    {
        return Quat(w,-x,-y,-z);
    }
    
    float Quat::magnitude() const
    {
        return Math::sqrt(w * w + x * x + y * y + z * z);
    }
    
    Quat Quat::norm() const
    {
        return *this / magnitude();
    }
    
    Quat Quat::inverse() const
    {
        return norm().conjugate();
    }
    
    bool Quat::equals(const Quat& q, float eps) const
    {
        return Math::equal(w,q.w,eps) && Math::equal(x,q.x,eps) && Math::equal(y,q.y,eps) && Math::equal(z,q.z,eps);
    }
    
    std::ostream& operator<<(std::ostream& os, const Quat& q)
    {
        Vec3 axis;
        float angle;
        q.toAxisAngle(axis,angle);
        os << "<Quat " << axis << " " << angle << ">";
    }
}
