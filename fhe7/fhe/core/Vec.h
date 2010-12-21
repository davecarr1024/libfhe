#ifndef FHE_VEC_H
#define FHE_VEC_H

#include <fhe/core/fhe_math.h>
#include <fhe/core/PyEnv.h>
#include <fhe/core/Rot.h>
#include <fhe/core/Util.h>

namespace fhe
{
    
    template <typename T>
    class Vec<2, T>
    {
        public:
            T x, y;
            
            Vec() :
                x( 0 ),
                y( 0 )
            {
            }
            
            Vec( T _x, T _y ) :
                x( _x ),
                y( _y )
            {
            }
            
            explicit Vec( const Rot2& rot, T length = 1 ) :
                x( length * Math::cos( rot.radians() ) ),
                y( length * Math::sin( rot.radians() ) )
            {
            }
            
            Vec operator-() const
            {
                return Vec( -x, -y );
            }
            
            Vec operator+( const Vec& v ) const
            {
                return Vec( x + v.x, y + v.y );
            }
            
            Vec operator-( const Vec& v ) const
            {
                return Vec( x - v.x, y - v.y );
            }
            
            Vec operator*( const Vec& v ) const
            {
                return Vec( x * v.x, y * v.y );
            }
            
            Vec operator/( const Vec& v ) const
            {
                FHE_ASSERT( !Math::equal( v.x, 0 ) && !Math::equal( v.y, 0 ) );
                return Vec( x / v.x, y / v.y );
            }
            
            Vec operator*( T d ) const
            {
                return Vec( x * d, y * d );
            }
            
            Vec operator/( T d ) const
            {
                FHE_ASSERT( !Math::equal( d, 0 ) );
                return Vec( x / d, y / d );
            }
            
            void operator+=( const Vec& v )
            {
                x += v.x;
                y += v.y;
            }
                
            void operator-=( const Vec& v )
            {
                x -= v.x;
                y -= v.y;
            }
            
            void operator*=( const Vec& v )
            {
                x *= v.x;
                y *= v.y;
            }
            
            void operator/=( const Vec& v )
            {
                FHE_ASSERT( !Math::equal( v.x, 0 ) && !Math::equal( v.y, 0 ) );
                x /= v.x;
                y /= v.y;
            }
            
            void operator*=( T d )
            {
                x *= d;
                y *= d;
            }
            
            void operator/=( T d )
            {
                FHE_ASSERT( !Math::equal( d, 0 ) );
                x /= d;
                y /= d;
            }
            
            bool operator==( const Vec& v ) const
            {
                return equals( v, Math::EPS );
            }
            
            T length() const
            {
                return Math::sqrt( x * x + y * y );
            }
            
            Vec norm() const
            {
                T l = length();
                FHE_ASSERT( !Math::equal( l, 0 ) );
                return *this / l;
            }
            
            T dot( const Vec& v ) const
            {
                return x * v.x + y * v.y;
            }
            
            Vec lerp( const Vec& v, T i ) const
            {
                return *this + ( v - *this ) * i;
            }
            
            bool equals( const Vec& v, T eps = Math::EPS ) const
            {
                return Math::equal( x, v.x, eps ) && Math::equal( y, v.y, eps );
            }
            
            std::string toString() const
            {
                std::ostringstream os;
                os << typeName() << "(" << x << "," << y << ")";
                return os.str();
            }
            
            Rot2 getRotTo( const Vec& v ) const
            {
                return Rot2( v ) - Rot2( *this );
            }
            
            static std::string typeName()
            {
                return std::string( "Vec2" ) + typeid(T).name();
            }
            
            static boost::python::object defineClass()
            {
                boost::python::scope c = boost::python::class_<Vec>( typeName().c_str(), boost::python::init<>() )
                    .def( boost::python::init< T, T >() )
                    .def( boost::python::init< Rot2, T >() )
                    .def( boost::python::init< Rot2 >() )
                    .def_readwrite( "x", &Vec::x )
                    .def_readwrite( "y", &Vec::y )
                    .def( "__repr__", &Vec::toString )
                    .def( "__eq__", &Vec::operator== )
                    .def( boost::python::self + boost::python::other<Vec>() )
                    .def( boost::python::self - boost::python::other<Vec>() )
                    .def( boost::python::self * boost::python::other<Vec>() )
                    .def( boost::python::self / boost::python::other<Vec>() )
                    .def( boost::python::self * T() )
                    .def( boost::python::self / T() )
                    .def("length", &Vec::length)
                    .def("norm", &Vec::norm)
                    .def("dot", &Vec::dot)
                    .def("lerp", &Vec::lerp)
                    .def("getRotTo", &Vec::getRotTo)
                ;
                c.attr( "ZERO" ) = ZERO;
                c.attr( "UNIT_X" ) = UNIT_X;
                c.attr( "UNIT_Y" ) = UNIT_Y;
                return c;
            }
            
            const static Vec ZERO;
            const static Vec UNIT_X;
            const static Vec UNIT_Y;
    };
    
    template <typename T>
    const Vec<2,T> Vec<2,T>::ZERO( 0, 0 );

    template <typename T>
    const Vec<2,T> Vec<2,T>::UNIT_X( 1, 0 );

    template <typename T>
    const Vec<2,T> Vec<2,T>::UNIT_Y( 0, 1 );

    template <typename T>
    class Vec<3, T>
    {
        public:
            T x, y, z;
            
            Vec() :
                x( 0 ),
                y( 0 ),
                z( 0 )
            {
            }
            
            Vec( T _x, T _y, T _z ) :
                x( _x ),
                y( _y ),
                z( _z )
            {
            }
            
            explicit Vec( const Rot3& rot, T length = 1 )
            {
                *this = ( rot * UNIT_X ) * length;
            }
            
            Vec operator-() const
            {
                return Vec( -x, -y, -z );
            }
            
            Vec operator+( const Vec& v ) const
            {
                return Vec( x + v.x, y + v.y, z + v.z );
            }
            
            Vec operator-( const Vec& v ) const
            {
                return Vec( x - v.x, y - v.y, z - v.z );
            }
            
            Vec operator*( const Vec& v ) const
            {
                return Vec( x * v.x, y * v.y, z * v.z );
            }
            
            Vec operator/( const Vec& v ) const
            {
                FHE_ASSERT( !Math::equal( v.x, 0 ) && !Math::equal( v.y, 0 ) && !Math::equal( v.z, 0 ) );
                return Vec( x / v.x, y / v.y, z / v.z );
            }
            
            Vec operator*( T d ) const
            {
                return Vec( x * d, y * d, z * d );
            }
            
            Vec operator/( T d ) const
            {
                FHE_ASSERT( !Math::equal( d, 0 ) );
                return Vec( x / d, y / d, z / d );
            }
            
            void operator+=( const Vec& v )
            {
                x += v.x;
                y += v.y;
                z += v.z;
            }
            
            void operator-=( const Vec& v )
            {
                x -= v.x;
                y -= v.y;
                z -= v.z;
            }
            
            void operator*=( const Vec& v )
            {
                x *= v.x;
                y *= v.y;
                z *= v.z;
            }
            
            void operator/=( const Vec& v )
            {
                FHE_ASSERT( !Math::equal( v.x, 0 ) && !Math::equal( v.y, 0 ) && !Math::equal( v.z, 0 ) );
                x /= v.x;
                y /= v.y;
                z /= v.z;
            }
            
            void operator*=( T d )
            {
                x *= d;
                y *= d;
                z *= d;
            }
                    
            void operator/=( T d )
            {
                FHE_ASSERT( !Math::equal( d, 0 ) );
                x /= d;
                y /= d;
                z /= d;
            }
            
            bool operator==( const Vec& v ) const
            {
                return equals( v );
            }
            
            T length() const
            {
                return Math::sqrt( x * x + y * y + z * z );
            }
            
            Vec norm() const
            {
                T l = length();
                FHE_ASSERT( !Math::equal( l, 0 ) );
                return *this / l;
            }
            
            T dot( const Vec& v ) const
            {
                return x * v.x + y * v.y + z * v.z;
            }
            
            Vec cross( const Vec& v ) const
            {
                return Vec( y * v.z - z * v.y,
                            z * v.x - x * v.z,
                            x * v.y - y * v.x );
            }
            
            Vec project( const Vec& v ) const
            {
                T l = length();
                FHE_ASSERT( !Math::equal( l, 0 ) );
                return *this * ( dot( v ) / ( l * l ) );
            }
            
            Vec lerp( const Vec& v, T i ) const
            {
                return *this + ( v - *this ) * i;
            }
            
            bool equals( const Vec& v, T eps = Math::EPS ) const
            {
                return Math::equal( x, v.x, eps ) && Math::equal( y, v.y, eps ) && Math::equal( z, v.z, eps );
            }
            
            std::string toString() const
            {
                std::ostringstream os;
                os << "Vec3(" << x << "," << y << "," << z << ")";
                return os.str();
            }
            
            Vec makePerp( const Vec& v ) const
            {
                return v.cross( v.dot( UNIT_Y ) < 0.75 ? UNIT_Y : UNIT_Z );
            }

            Rot3 getRotTo( const Vec& v ) const
            {
                T d = dot( v );
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
            
            static std::string typeName()
            {
                return std::string( "Vec3" ) + typeid(T).name();
            }
            
            static boost::python::object defineClass()
            {
                boost::python::scope c = boost::python::class_<Vec>( typeName().c_str(), boost::python::init<>() )
                    .def( boost::python::init< T, T, T >() )
                    .def( boost::python::init< Rot3, T >() )
                    .def( boost::python::init< Rot3 >() )
                    .def_readwrite( "x", &Vec::x )
                    .def_readwrite( "y", &Vec::y )
                    .def_readwrite( "z", &Vec::z )
                    .def( "__repr__", &Vec::toString )
                    .def( "__eq__", &Vec::operator== )
                    .def( boost::python::self + boost::python::other<Vec>() )
                    .def( boost::python::self - boost::python::other<Vec>() )
                    .def( boost::python::self * boost::python::other<Vec>() )
                    .def( boost::python::self / boost::python::other<Vec>() )
                    .def( boost::python::self * T() )
                    .def( boost::python::self / T() )
                    .def("length",&Vec::length)
                    .def("norm",&Vec::norm)
                    .def("dot",&Vec::dot)
                    .def("cross",&Vec::cross)
                    .def("project",&Vec::project)
                    .def("lerp",&Vec::lerp)
                    .def("getRotTo",&Vec::getRotTo)
                ;
                c.attr( "ZERO" ) = ZERO;
                c.attr( "UNIT_X" ) = UNIT_X;
                c.attr( "UNIT_Y" ) = UNIT_Y;
                c.attr( "UNIT_Z" ) = UNIT_Z;
                return c;
            }
                
            
            static const Vec ZERO;
            static const Vec UNIT_X;
            static const Vec UNIT_Y;
            static const Vec UNIT_Z;
    };
    
    template <typename T>
    const Vec<3,T> Vec<3,T>::ZERO( 0, 0, 0 );
    template <typename T>
    const Vec<3,T> Vec<3,T>::UNIT_X( 1, 0, 0 );
    template <typename T>
    const Vec<3,T> Vec<3,T>::UNIT_Y( 0, 1, 0 );
    template <typename T>
    const Vec<3,T> Vec<3,T>::UNIT_Z( 0, 0, 1 );
    
    template <size_t dim, typename T>
    std::ostream& operator<<( std::ostream& os, const Vec<dim,T>& v )
    {
        return os << v.toString();
    }
    
}

#endif
