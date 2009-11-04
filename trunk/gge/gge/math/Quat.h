#ifndef QUAT_H
#define QUAT_H

#include <iostream>
#include "ggeMath.h"
#include <boost/python.hpp>

namespace gge
{
    
    class Vec3;
    
    class Quat
    {
        public:
            float w, x, y, z;
            
            Quat();
            Quat(float _w, float _x, float _y, float _z);
            Quat(const Vec3& axis, float angle);
            Quat(const Vec3& v);
            Quat(const Quat& q);
            
            Quat& operator=(const Quat& q);
            
            void toAxisAngle(Vec3& axis, float& angle) const;
            
            Quat operator*(const Quat& q) const;
            Vec3 operator*(const Vec3& v) const;
            Quat operator*(float f) const;
            Quat operator/(float f) const;
            
            Quat conjugate() const;
            float magnitude() const;
            Quat norm() const;
            Quat inverse() const;
            
            bool equals(const Quat& q, float eps = Math::EPSILON) const;
            
            const static Quat IDENTITY;
            
            std::string toString();
            
            bool pyEquals( const Quat& q );
            
            boost::python::object pyToAxisAngle();
            
            static boost::python::object defineClass();
    };
    
}

#endif
