#ifndef FHE_VEC_H
#define FHE_VEC_H

#include <fhe/fhe_math.h>
#include <fhe/PyEnv.h>
#include <string>

namespace fhe
{
    
    template <size_t dim>
    class Rot;
    
    template <size_t dim>
    class Vec;
    
    template <>
    class Vec<2>
    {
        public:
            double x, y;
            
            Vec();
            Vec( double _x, double _y );
            Vec( const Rot<2>& rot, double length = 1 );
            
            Vec operator-() const;
            Vec operator+( const Vec& v ) const;
            Vec operator-( const Vec& v ) const;
            Vec operator*( const Vec& v ) const;
            Vec operator/( const Vec& v ) const;
            Vec operator*( double d ) const;
            Vec operator/( double d ) const;
            
            void operator+=( const Vec& v );
            void operator-=( const Vec& v );
            void operator*=( const Vec& v );
            void operator/=( const Vec& v );
            void operator*=( double d );
            void operator/=( double d );
            
            double length() const;
            Vec norm() const;
            double dot( const Vec& v ) const;
            Vec lerp( const Vec& v, double i ) const;
            bool equals( const Vec& v, double eps = Math::EPS ) const;
            std::string toString() const;
            Rot<2> getRotTo( const Vec& v ) const;
            
            bool pyEquals( const Vec& v ) const;
            static boost::python::object defineClass();
            
            const static Vec ZERO;
            const static Vec UNIT_X;
            const static Vec UNIT_Y;
    };
    
    template <>
    class Vec<3>
    {
        public:
            double x, y, z;
            
            Vec();
            Vec( double _x, double _y, double _z );
            Vec( const Rot<3>& rot, double length = 1 );
            
            Vec operator-() const;
            Vec operator+( const Vec& v ) const;
            Vec operator-( const Vec& v ) const;
            Vec operator*( const Vec& v ) const;
            Vec operator/( const Vec& v ) const;
            Vec operator*( double d ) const;
            Vec operator/( double d ) const;
            
            void operator+=( const Vec& v );
            void operator-=( const Vec& v );
            void operator*=( const Vec& v );
            void operator/=( const Vec& v );
            void operator*=( double d );
            void operator/=( double d );
            
            double length() const;
            Vec norm() const;
            double dot( const Vec& v ) const;
            Vec cross( const Vec& v ) const;
            Vec project( const Vec& v ) const;
            Vec lerp( const Vec& v, double i ) const;
            bool equals( const Vec& v, double eps = Math::EPS ) const;
            std::string toString() const;
            Rot<3> getRotTo( const Vec& v ) const;
            
            bool pyEquals( const Vec& v ) const;
            static boost::python::object defineClass();
            
            static const Vec ZERO;
            static const Vec UNIT_X;
            static const Vec UNIT_Y;
            static const Vec UNIT_Z;
    };
    
    typedef Vec<2> Vec2;
    typedef Vec<3> Vec3;
    
}

#endif
