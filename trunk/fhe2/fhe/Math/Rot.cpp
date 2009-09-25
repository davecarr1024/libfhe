#include "Rot.h"
#include "Vec2.h"
#include <cassert>
#include <sstream>

namespace fhe
{
    
    const Rot Rot::ZERO;
    
    Rot::Rot() :
        angle(0)
    {
    }
    
    Rot::Rot(float _angle) :
        angle(_angle)
    {
    }
    
    Rot::Rot(const Vec2& v)
    {
        angle = Math::atan2(v.y,v.x);
    }
    
    Rot::Rot(const Rot& r) :
        angle(r.angle)
    {
    }
    
    Rot& Rot::operator=(const Rot& r)
    {
        angle = r.angle;
        return *this;
    }
    
    Rot Rot::operator+(const Rot& r) const
    {
        return Rot(angle + r.angle);//.norm();
    }
    
    Rot Rot::operator-(const Rot& r) const
    {
        return Rot(angle - r.angle);//.norm();
    }
    
    Rot Rot::operator*(float f) const
    {
        return Rot(angle * f);//.norm();
    }
    
    Rot Rot::operator/(float f) const
    {
        assert(f != 0);
        return Rot(angle / f);//.norm();
    }
    
    Vec2 Rot::operator*(const Vec2& v) const
    {
        return Vec2(Rot(v) + *this) * v.length();
    }
    
    Rot Rot::norm() const
    {
        return Rot(Math::fmod(angle,Math::PI));
    }
    
    bool Rot::equals(const Rot& r, float eps) const
    {
        return Math::equal(angle,r.angle,eps);
    }
    
    std::string Rot::toString()
    {
        std::ostringstream sout;
        sout << "Rot(" << angle << ")";
        return sout.str();
    }
    
    bool Rot::pyEquals( const Rot& r )
    {
        return equals(r);
    }
    
    boost::python::object Rot::defineClass()
    {
        return boost::python::class_<Rot>("Rot",boost::python::init<float>() )
            .def_readwrite("angle", &Rot::angle)
            .def("__repr__", &Rot::toString)
            .def("__eq__", &Rot::pyEquals)
            .def( boost::python::self + boost::python::other<Rot>() )
            .def( boost::python::self - boost::python::other<Rot>() )
            .def( boost::python::self * float() )
            .def( boost::python::self / float() )
            .def( boost::python::self * boost::python::other<Vec2>() )
            .def("norm",&Rot::norm)
        ;
    }
}
