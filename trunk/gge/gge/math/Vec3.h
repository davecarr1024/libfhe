#ifndef VEC3_H
#define VEC3_H

#include "ggeMath.h"

#include <vector>
#include <iostream>
#include <boost/python.hpp>

namespace gge
{
    
    class Quat;
    class Vec3;
    
    typedef std::vector<Vec3> Vec3List;
    
    class Vec3
    {
        public:
            float x, y, z;
            
            Vec3();
            Vec3(float _x, float _y, float _z);
            Vec3(const Quat& q, float length = 1);
            Vec3(const Vec3& v);
            
            Vec3& operator=(const Vec3& v);
            
            float& operator[](int i);
            float operator[](int i) const;
            
            Vec3 operator+(const Vec3& v) const;
            Vec3 operator-(const Vec3& v) const;
            Vec3 operator*(float f) const;
            Vec3 operator/(float f) const;
            Vec3 operator-() const;
            
            void operator+=(const Vec3& v);
            void operator-=(const Vec3& v);
            void operator*=(float f);
            void operator/=(float f);
            
            float length() const;
            Vec3 norm() const;
            float dot(const Vec3& v) const;
            Vec3 cross(const Vec3& v) const;
            Vec3 project(const Vec3& v) const;
            Quat getRotTo(const Vec3& v) const;
            Vec3 makePerp() const;
            Vec3 lerp(const Vec3& v, float i) const;
            
            bool equals(const Vec3& v, float eps = Math::EPSILON) const;
            
            friend std::ostream& operator<<(std::ostream& os, const Vec3& v);
            
            const static Vec3 UNIT_X;
            const static Vec3 UNIT_Y;
            const static Vec3 UNIT_Z;
            const static Vec3 ZERO;
            
            std::string toString();
            
            bool pyEquals( const Vec3& v );
            
            static boost::python::object defineClass();
    };
    
}

#endif
