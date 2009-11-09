#ifndef ROT_H
#define ROT_H

#include "fheMath.h"

#include <boost/python.hpp>

namespace fhe
{
    
    class Vec2;
    
    class Rot
    {
        public:
            float angle;
            
            Rot();
            Rot(float _angle);
            Rot(const Vec2& v);
            Rot(const Rot& r);
            
            Rot& operator=(const Rot& r);
            
            Rot operator+(const Rot& r) const;
            Rot operator-(const Rot& r) const;
            Rot operator*(float f) const;
            Vec2 operator*(const Vec2& v) const;
            Rot operator/(float f) const;
            Rot operator-() const;
            
            Rot norm() const;
            
            bool equals(const Rot& r, float eps = Math::EPSILON) const;
            
            float degrees() const;
            
            float radians() const;
            
            const static Rot ZERO;
            
            std::string toString();
            
            bool pyEquals( const Rot& r );
            
            static boost::python::object defineClass();
    };
    
}

#endif
