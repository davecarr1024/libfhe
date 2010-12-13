#ifndef FHE_ROT_H
#define FHE_ROT_H

#include <fhe/fhe_math.h>
#include <fhe/PyEnv.h>
#include <string>

namespace fhe
{
    
    template <size_t dim>
    class Vec;
    
    template <size_t dim>
    class Rot;
    
    typedef Rot<2> Rot2;
    typedef Rot<3> Rot3;
    
    template <>
    class Rot<2>
    {
        private:
            double m_a;
            
            Rot( double a );
            
        public:
            Rot();
            explicit Rot( const Vec<2>& v );
            static Rot fromDegrees( double degrees );
            static Rot fromRadians( double radians );
            
            double degrees() const;
            double radians() const;
            
            Rot operator+( const Rot& r ) const;
            Rot operator-( const Rot& r ) const;
            Rot operator*( double d ) const;
            Rot operator/( double d ) const;
            Vec<2> operator*( const Vec<2>& v ) const;
            
            bool operator==( const Rot& r ) const;

            Rot inverse() const;
            Rot norm() const;
            bool equals( const Rot& r, double eps = Math::EPS ) const;
            std::string toString() const;
            
            static boost::python::object defineClass();
            
            static const Rot IDENTITY;
    };
    
    std::ostream& operator<<( std::ostream& os, const Rot2& r );
    
    template <>
    class Rot<3>
    {
        public:
            double w, x, y, z;
            
            Rot();
            Rot( double _w, double _x, double _y, double _z );
            Rot( const Vec<3>& axis, double angle );
            explicit Rot( const Vec<3>& v );
            
            void toAxisAngle( Vec<3>& axis, double& angle ) const;
            
            Rot operator*( const Rot& r ) const;
            Vec<3> operator*( const Vec<3>& v ) const;
            Rot operator*( double d ) const;
            Rot operator/( double d ) const;
            
            bool operator==( const Rot& r ) const;
            
            Rot conjugate() const;
            double magnitude() const;
            Rot norm() const;
            Rot inverse() const;
            bool equals( const Rot& r, double eps = Math::EPS ) const;
            std::string toString() const;

            boost::python::object pyToAxisAngle();
            static boost::python::object defineClass();
            
            static const Rot IDENTITY;
    };
    
    std::ostream& operator<<( std::ostream& os, const Rot3& r );
    
}

#endif