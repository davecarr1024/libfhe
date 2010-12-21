#include <fhe/core/Rot.h>
#include <fhe/core/Vec.h>
#include <fhe/core/Util.h>
#include <sstream>

namespace fhe
{
    
    const Rot2 Rot2::IDENTITY;
    
    Rot2::Rot() :
        m_a( 0 )
    {
    }
    
    Rot2::Rot( double a ) :
        m_a( a )
    {
    }
    
    Rot2 Rot2::fromDegrees( double degrees )
    {
        return Rot( Math::radians( degrees ) );
    }
    
    Rot2 Rot2::fromRadians( double radians )
    {
        return Rot( radians );
    }
    
    double Rot2::degrees() const
    {
        return Math::degrees( m_a );
    }
    
    double Rot2::radians() const
    {
        return m_a;
    }
    
    Rot2 Rot2::inverse() const
    {
        return Rot( -m_a );
    }
    
    Rot2 Rot2::operator+( const Rot2& r ) const
    {
        return Rot( m_a + r.m_a );
    }
    
    Rot2 Rot2::operator-( const Rot& r ) const
    {
        return Rot( m_a - r.m_a );
    }
    
    Rot2 Rot2::operator*( double d ) const
    {
        return Rot( m_a * d );
    }
    
    Rot2 Rot2::operator/( double d ) const
    {
        FHE_ASSERT( !Math::equal( d, 0 ) );
        return Rot( m_a / d );
    }
    
    Rot2 Rot2::norm() const
    {
        double d = m_a;
        while ( d < -Math::PI )
        {
            d += Math::PI * 2;
        }
        while ( d > Math::PI )
        {
            d -= Math::PI * 2;
        }
        return d;
    }
    
    bool Rot2::equals( const Rot& r, double eps ) const
    {
        return Math::equal( m_a, r.m_a, eps );
    }
    
    bool Rot2::operator==( const Rot& r ) const
    {
        return equals( r );
    }
    
    std::string Rot2::toString() const
    {
        std::ostringstream os;
        os << "Rot2(" << degrees() << ")";
        return os.str();
    }
    
    std::ostream& operator<<( std::ostream& os, const Rot2& r )
    {
        return os << r.toString();
    }
    
    boost::python::object Rot2::defineClass()
    {
        boost::python::scope c = boost::python::class_<Rot2>( "Rot2", boost::python::init<>() )
            .def( "fromDegrees", &Rot2::fromDegrees )
            .staticmethod( "fromDegrees" )
            .def( "fromRadians", &Rot2::fromRadians )
            .staticmethod( "fromRadians" )
            .def( "__eq__", &Rot2::operator== )
            .def( "__repr__", &Rot2::toString )
            .def( boost::python::self + boost::python::other<Rot2>() )
            .def( boost::python::self - boost::python::other<Rot2>() )
            .def( boost::python::self * double() )
            .def( boost::python::self / double() )
            .def( "inverse", &Rot2::inverse )
            .def( "norm", &Rot2::norm )
        ;
        c.attr( "IDENTITY" ) = IDENTITY;
        return c;
    }
    
    const Rot3 Rot3::IDENTITY;
    
    Rot3::Rot() :
        w( 1 ),
        x( 0 ),
        y( 0 ),
        z( 0 )
    {
    }
    
    Rot3::Rot( double _w, double _x, double _y, double _z ) :
        w( _w ),
        x( _x ),
        y( _y ),
        z( _z )
    {
    }
    
    Rot3 Rot3::operator*(const Rot3& q) const
    {
        return Rot3(w * q.w - x * q.x - y * q.y - z * q.z,
                    w * q.x + x * q.w + y * q.z - z * q.y,
                    w * q.y + y * q.w + z * q.x - x * q.z,
                    w * q.z + z * q.w + x * q.y - y * q.x);
    }
    
    Rot3 Rot3::operator*(double f) const
    {
        return Rot3(w * f, x * f, y * f, z * f);
    }
    
    Rot3 Rot3::operator/(double f) const
    {
        FHE_ASSERT( !Math::equal( f, 0 ) );
        return Rot3(w / f, x / f, y / f, z / f);
    }
    
    Rot3 Rot3::conjugate() const
    {
        return Rot3(w,-x,-y,-z);
    }
    
    double Rot3::magnitude() const
    {
        return Math::sqrt(w * w + x * x + y * y + z * z);
    }
    
    Rot3 Rot3::norm() const
    {
        return *this / magnitude();
    }
    
    Rot3 Rot3::inverse() const
    {
        return norm().conjugate();
    }
    
    bool Rot3::equals(const Rot3& q, double eps) const
    {
        return Math::equal(w,q.w,eps) && Math::equal(x,q.x,eps) && Math::equal(y,q.y,eps) && Math::equal(z,q.z,eps);
    }
    
    bool Rot3::operator==( const Rot& r ) const
    {
        return equals( r );
    }
    
    std::string Rot3::toString() const
    {
        Vec3d axis;
        double angle;
        toAxisAngle(axis,angle);
        std::ostringstream outs;
        outs << "Rot3(" << axis << "," << angle << ")";
        return outs.str();
    }
    
    std::ostream& operator<<( std::ostream& os, const Rot3& r )
    {
        return os << r.toString();
    }
    
    boost::python::object Rot3::pyToAxisAngle()
    {
        Vec3d axis;
        double angle;
        toAxisAngle(axis,angle);
        return boost::python::make_tuple(axis,angle);
    }
    
    boost::python::object Rot3::defineClass()
    {
        boost::python::scope c = boost::python::class_<Rot3>("Rot3",boost::python::init<>())
            .def(boost::python::init<double,double,double,double>())
            .def(boost::python::init<Vec3d,double>())
            .def(boost::python::init<Vec3d>())
            .def_readwrite("w", &Rot3::w)
            .def_readwrite("x", &Rot3::x)
            .def_readwrite("y", &Rot3::y)
            .def_readwrite("z", &Rot3::z)
            .def("__repr__", &Rot3::toString)
            .def("__eq__", &Rot3::operator== )
            .def(boost::python::self * boost::python::other<Rot3>())
            .def(boost::python::self * boost::python::other<Vec3d>())
            .def(boost::python::self * double())
            .def(boost::python::self / double())
            .def("norm",&Rot3::norm)
            .def("inverse",&Rot3::inverse)
            .def("toAxisAngle",&Rot3::pyToAxisAngle)
        ;
        c.attr( "IDENTITY" ) = IDENTITY;
        return c;
    }
}
