#include <fhe/Vec.h>
#include <fhe/Rot.h>
#include <fhe/Util.h>
#include <sstream>

namespace fhe
{
    const Vec2 Vec2::ZERO( 0, 0 );
    const Vec2 Vec2::UNIT_X( 1, 0 );
    const Vec2 Vec2::UNIT_Y( 0, 1 );
    
    Vec2::Vec() :
        x( 0 ),
        y( 0 )
    {
    }

    Vec2::Vec( double _x, double _y ) :
        x( _x ),
        y( _y )
    {
    }
    
    Vec2::Vec( const Rot2& r, double length ) :
        x( Math::cos( r.radians() ) * length ),
        y( Math::sin( r.radians() ) * length )
    {
    }
    
    Rot2 Vec2::getRotTo( const Vec& v ) const
    {
        return Rot2( v ) - Rot2( *this );
    }

    Vec2 Vec2::operator-() const
    {
        return Vec2( -x, -y );
    }

    Vec2 Vec2::operator+( const Vec2& v ) const
    {
        return Vec2( x + v.x, y + v.y );
    }

    Vec2 Vec2::operator-( const Vec2& v ) const
    {
        return Vec2( x - v.x, y - v.y );
    }

    Vec2 Vec2::operator*( const Vec2& v ) const
    {
        return Vec2( x * v.x, y * v.y );
    }

    Vec2 Vec2::operator/( const Vec2& v ) const
    {
        FHE_ASSERT( !Math::equal( v.x, 0 ) && !Math::equal( v.y, 0 ) );
        return Vec2( x / v.x, y / v.y );
    }

    Vec2 Vec2::operator*( double d ) const
    {
        return Vec2( x * d, y * d );
    }

    Vec2 Vec2::operator/( double d ) const
    {
        FHE_ASSERT( !Math::equal( d, 0 ) );
        return Vec2( x / d, y / 2 );
    }

    void Vec2::operator+=( const Vec2& v )
    {
        x += v.x;
        y += v.y;
    }

    void Vec2::operator-=( const Vec2& v )
    {
        x -= v.x;
        y -= v.y;
    }

    void Vec2::operator*=( const Vec2& v )
    {
        x *= v.x;
        y *= v.y;
    }

    void Vec2::operator/=( const Vec2& v )
    {
        FHE_ASSERT( !Math::equal( v.x, 0 ) && !Math::equal( v.y, 0 ) );
        x /= v.x;
        y /= v.y;
    }

    void Vec2::operator*=( double d )
    {
        x *= d;
        y *= d;
    }

    void Vec2::operator/=( double d )
    {
        FHE_ASSERT( !Math::equal( d, 0 ) );
        x /= d;
        y /= d;
    }

    double Vec2::length() const
    {
        return Math::sqrt( x * x + y * y );
    }

    Vec2 Vec2::norm() const
    {
        double l = length();
        FHE_ASSERT( !Math::equal( l, 0 ) );
        return operator/( l );
    }

    double Vec2::dot( const Vec2& v ) const
    {
        return x * v.x + y * v.y;
    }

    Vec2 Vec2::lerp( const Vec2& v, double i ) const
    {
        return *this + ( v - *this ) * i;
    }

    bool Vec2::equals( const Vec2& v, double eps ) const
    {
        return Math::equal( x, v.x, eps ) && Math::equal( y, v.y, eps );
    }
    
    std::string Vec2::toString() const
    {
        std::ostringstream os;
        os << "Vec2(" << x << "," << y << ")";
        return os.str();
    }
    
    bool Vec2::pyEquals( const Vec& v ) const
    {
        return equals( v );
    }
    
    boost::python::object Vec2::defineClass()
    {
        boost::python::scope c = boost::python::class_<Vec2>( "Vec2", boost::python::init<>() )
            .def( boost::python::init< double, double >() )
            .def( boost::python::init< Rot2, double >() )
            .def( boost::python::init< Rot2 >() )
            .def_readwrite( "x", &Vec2::x )
            .def_readwrite( "y", &Vec2::y )
            .def( "__repr__", &Vec2::toString )
            .def( "__eq__", &Vec2::pyEquals )
            .def( boost::python::self + boost::python::other<Vec2>() )
            .def( boost::python::self - boost::python::other<Vec2>() )
            .def( boost::python::self * boost::python::other<Vec2>() )
            .def( boost::python::self / boost::python::other<Vec2>() )
            .def( boost::python::self * double() )
            .def( boost::python::self / double() )
            .def("length", &Vec2::length)
            .def("norm", &Vec2::norm)
            .def("dot", &Vec2::dot)
            .def("lerp", &Vec2::lerp)
            .def("getRotTo", &Vec2::getRotTo)
        ;
        c.attr( "ZERO" ) = ZERO;
        c.attr( "UNIT_X" ) = UNIT_X;
        c.attr( "UNIT_Y" ) = UNIT_Y;
        return c;
    }

    const Vec3 Vec3::ZERO( 0, 0, 0 );
    const Vec3 Vec3::UNIT_X( 1, 0, 0 );
    const Vec3 Vec3::UNIT_Y( 0, 1, 0 );
    const Vec3 Vec3::UNIT_Z( 0, 0, 1 );
    
    Vec3::Vec() :
        x( 0 ),
        y( 0 ),
        z( 0 )
    {
    }

    Vec3::Vec( double _x, double _y, double _z ) :
        x( _x ),
        y( _y ),
        z( _z )
    {
    }
    
    Vec3::Vec( const Rot3& r, double l ) 
    {
        *this = ( r * UNIT_X ) * l;
    }

    Vec3 Vec3::operator-() const
    {
        return Vec3( -x, -y, -z );
    }

    Vec3 Vec3::operator+( const Vec3& v ) const
    {
        return Vec3( x + v.x, y + v.y, z + v.z );
    }

    Vec3 Vec3::operator-( const Vec3& v ) const
    {
        return Vec3( x - v.x, y - v.y, z - v.z );
    }

    Vec3 Vec3::operator*( const Vec3& v ) const
    {
        return Vec3( x * v.x, y * v.y, z * v.z );
    }

    Vec3 Vec3::operator/( const Vec3& v ) const
    {
        FHE_ASSERT( !Math::equal( v.x, 0 ) && !Math::equal( v.y, 0 ) && !Math::equal( v.z, 0 ) );
        return Vec3( x / v.x, y / v.y, z / v.z );
    }

    Vec3 Vec3::operator*( double d ) const
    {
        return Vec3( x * d, y * d, z / d );
    }

    Vec3 Vec3::operator/( double d ) const
    {
        FHE_ASSERT( !Math::equal( d, 0 ) );
        return Vec3( x / d, y / d, z / d );
    }

    void Vec3::operator+=( const Vec3& v )
    {
        x += v.x;
        y += v.y;
        z += v.z;
    }

    void Vec3::operator-=( const Vec3& v )
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
    }

    void Vec3::operator*=( const Vec3& v )
    {
        x *= v.x;
        y *= v.y;
        z *= v.z;
    }

    void Vec3::operator/=( const Vec3& v )
    {
        FHE_ASSERT( !Math::equal( v.x, 0 ) && !Math::equal( v.y, 0 ) && !Math::equal( v.z, 0 ) );
        x /= v.x;
        y /= v.y;
        z /= v.z;
    }

    void Vec3::operator*=( double d )
    {
        x *= d;
        y *= d;
        z *= d;
    }

    void Vec3::operator/=( double d )
    {
        FHE_ASSERT( !Math::equal( d, 0 ) );
        x /= d;
        y /= d;
        z /= d;
    }

    double Vec3::length() const
    {
        return Math::sqrt( x * x + y * y + z * z );
    }

    Vec3 Vec3::norm() const
    {
        double l = length();
        FHE_ASSERT( !Math::equal( l, 0 ) );
        return operator/( l );
    }

    double Vec3::dot( const Vec3& v ) const
    {
        return x * v.x + y * v.y + z * v.z;
    }

    Vec3 Vec3::cross( const Vec3& v ) const
    {
        return Vec3( y * v.z - z * v.y,
                    z * v.x - x * v.z,
                    x * v.y - y * v.x );
    }

    Vec3 Vec3::project( const Vec3& v ) const
    {
        double l = length();
        FHE_ASSERT( !Math::equal( l, 0 ) );
        return *this * ( dot( v ) / ( l * l ) );
    }

    Vec3 Vec3::lerp( const Vec3& v, double i ) const
    {
        return *this + ( v - *this ) * i;
    }
            
    bool Vec3::equals( const Vec3& v, double eps ) const
    {
        return Math::equal( x, v.x, eps ) && Math::equal( y, v.y, eps ) && Math::equal( z, v.z, eps );
    }
    
    std::string Vec3::toString() const
    {
        std::ostringstream os;
        os << "Vec3(" << x << "," << y << "," << z << ")";
        return os.str();
    }
    
    Vec3 makePerp( const Vec3& v )
    {
        return v.cross( v.dot( Vec3::UNIT_Y ) < 0.75 ? Vec3::UNIT_Y : Vec3::UNIT_Z );
    }
    
    Rot3 Vec3::getRotTo( const Vec& v ) const
    {
        double d = dot( v );
        if ( d > 1 - Math::EPS )
        {
            return Rot3();
        }
        else if ( d < -1 + Math::EPS )
        {
            return Rot3( makePerp( v ), Math::PI );
        }
        else
        {
            return Rot3( cross( v ).norm(), Math::acos( norm().dot( v.norm() ) ) );
        }
    }
    
    bool Vec3::pyEquals( const Vec& v ) const
    {
        return equals( v );
    }
    
    boost::python::object Vec3::defineClass()
    {
        boost::python::scope c = boost::python::class_<Vec3>( "Vec3", boost::python::init<>() )
            .def( boost::python::init< double, double, double >() )
            .def( boost::python::init< Rot3, double >() )
            .def( boost::python::init< Rot3 >() )
            .def_readwrite( "x", &Vec3::x )
            .def_readwrite( "y", &Vec3::y )
            .def_readwrite( "z", &Vec3::z )
            .def( "__repr__", &Vec3::toString )
            .def( "__eq__", &Vec3::pyEquals )
            .def( boost::python::self + boost::python::other<Vec3>() )
            .def( boost::python::self - boost::python::other<Vec3>() )
            .def( boost::python::self * boost::python::other<Vec3>() )
            .def( boost::python::self / boost::python::other<Vec3>() )
            .def( boost::python::self * double() )
            .def( boost::python::self / double() )
            .def("length",&Vec3::length)
            .def("norm",&Vec3::norm)
            .def("dot",&Vec3::dot)
            .def("cross",&Vec3::cross)
            .def("project",&Vec3::project)
            .def("lerp",&Vec3::lerp)
            .def("getRotTo",&Vec3::getRotTo)
        ;
        c.attr( "ZERO" ) = ZERO;
        c.attr( "UNIT_X" ) = UNIT_X;
        c.attr( "UNIT_Y" ) = UNIT_Y;
        c.attr( "UNIT_Z" ) = UNIT_Z;
        return c;
    }
    
}
