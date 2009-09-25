#ifndef VEC2_H
#define VEC2_H

#include <boost/python.hpp>
#include <vector>
#include "fheMath.h"

namespace fhe
{
    class Rot;
    class Vec2;
    
    typedef std::vector<Vec2> Vec2List;
    
    class Vec2
    {
        public:
            float x, y;
            
            Vec2();
            Vec2(float _x, float _y);
            Vec2(const Rot& r, float length = 1);
            Vec2(const Vec2& v);
            
            Vec2& operator=(const Vec2& v);
            
            float& operator[](int i);
            
            Vec2 operator+(const Vec2& v) const;
            Vec2 operator-(const Vec2& v) const;
            Vec2 operator*(float f) const;
            Vec2 operator/(float f) const;
            
            void operator+=(const Vec2& v);
            void operator-=(const Vec2& v);
            void operator*=(float f);
            void operator/=(float f);
            
            Vec2 operator-() const;
            
            float length() const;
            Vec2 norm() const;
            float dot(const Vec2& v) const;
            Rot getRotTo(const Vec2& v) const;
            Vec2 lerp(const Vec2& v, float i) const;

            bool equals(const Vec2& v, float eps = Math::EPSILON) const;
            
            const static Vec2 UNIT_X;
            const static Vec2 UNIT_Y;
            const static Vec2 ZERO;
            
            std::string toString();
            bool pyEquals( const Vec2& v );
            
            static boost::python::object defineClass();
    };
    
}

#endif
