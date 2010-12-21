#ifndef FHE_ROT_H
#define FHE_ROT_H

#include <fhe/core/fhe_math.h>
#include <fhe/core/PyEnv.h>
#include <string>

namespace fhe
{
    
    template <>
    class Rot<2>
    {
        private:
            double m_a;
            
            Rot( double a );
            
        public:
            Rot();
            
            template <typename T>
            explicit Rot( const Vec<2,T>& v ) :
                m_a( Math::atan2( v.y, v.x ) )
            {
            }
            
            static Rot fromDegrees( double degrees );
            static Rot fromRadians( double radians );
            
            double degrees() const;
            double radians() const;
            
            Rot operator+( const Rot& r ) const;
            Rot operator-( const Rot& r ) const;
            Rot operator*( double d ) const;
            Rot operator/( double d ) const;
            
            template <typename T>
            Vec<2,T> operator*( const Vec<2,T>& v ) const
            {
                return Vec<2,T>( Rot( v ) + *this ) * v.length();
            }
            
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
            
            template <typename T>
            Rot( const Vec<3,T>& axis, double angle )
            {
                Vec<3,T> axisNorm = axis.norm();
                double sa = Math::sin( angle / 2 );
                double ca = Math::cos( angle / 2 );
                *this = Rot( ca, axisNorm.x * sa, axisNorm.y * sa, axisNorm.z * sa );
            }
            
            template <typename T>
            explicit Rot( const Vec<3,T>& v )
            {
                *this = Vec<3,T>::UNIT_X.getRotTo( v );
            }
            
            template <typename T>
            void toAxisAngle( Vec<3,T>& axis, double& angle ) const
            {
                Rot3 q = norm();
                double ca = q.w;
                angle = Math::acos(ca) * 2.0;
                double sa = Math::sqrt(1.0 - ca * ca);
                if (Math::abs(sa) < Math::EPS)
                    sa = 1;
                axis = Vec<3,T>(q.x / sa, q.y / sa, q.z /sa);
            }
            
            Rot operator*( const Rot& r ) const;
            
            template <typename T>
            Vec<3,T> operator*( const Vec<3,T>& v ) const
            {
                Rot3 q = *this * Rot3(0,v.x,v.y,v.z) * inverse();
                return Vec<3,T>(q.x,q.y,q.z);
            }
            
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
