#ifndef PLANE_H
#define PLANE_H

#include "Vec3.h"
#include <vector>
#include "ggeMath.h"

namespace gge
{
    
    class Plane;
    
    typedef std::vector<Plane> PlaneList;
    
    class Plane
    {
        public:
            Vec3 normal;
            float dist;
            
            Plane();
            Plane(const Vec3& _normal, float _dist);
            Plane(const Vec3& point, const Vec3& _normal);
            Plane(const Plane& p);
            
            Plane& operator=(const Plane& p);
            
            float distToPoint(const Vec3& v) const;
            Vec3 projectPoint(const Vec3& v) const;
            bool classify(const Vec3& v) const;
            
            bool equals(const Plane& p, float eps = Math::EPSILON) const;
    };
    
}

#endif
