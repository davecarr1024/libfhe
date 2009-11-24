#include "Quat.h"
#include "Vec3.h"
#include <cassert>
#include <sstream>

namespace fhe
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
    
    Quat::Quat(const Vec3& v)
    {
        *this = Vec3::UNIT_X.getRotTo(v);
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
    
    std::string Quat::toString()
    {
        Vec3 axis;
        float angle;
        toAxisAngle(axis,angle);
        std::ostringstream outs;
        outs << "Quat(" << axis.toString() << ", " << angle << ")";
        return outs.str();
    }
    
    bool Quat::pyEquals( const Quat& q )
    {
        return equals(q);
    }
    
    boost::python::object Quat::pyToAxisAngle()
    {
        Vec3 axis;
        float angle;
        toAxisAngle(axis,angle);
        return boost::python::make_tuple(axis,angle);
    }
    
    boost::python::object Quat::defineClass()
    {
        return boost::python::class_<Quat>("Quat",boost::python::init<>())
            .def(boost::python::init<float,float,float,float>())
            .def(boost::python::init<Vec3,float>())
            .def(boost::python::init<Vec3>())
            .def_readwrite("w", &Quat::w)
            .def_readwrite("x", &Quat::x)
            .def_readwrite("y", &Quat::y)
            .def_readwrite("z", &Quat::z)
            .def("__repr__", &Quat::toString)
            .def("__eq__", &Quat::pyEquals )
            .def(boost::python::self * boost::python::other<Quat>())
            .def(boost::python::self * boost::python::other<Vec3>())
            .def(boost::python::self * float())
            .def(boost::python::self / float())
            .def("norm",&Quat::norm)
            .def("inverse",&Quat::inverse)
            .def("toAxisAngle",&Quat::pyToAxisAngle)
        ;
    }
}
