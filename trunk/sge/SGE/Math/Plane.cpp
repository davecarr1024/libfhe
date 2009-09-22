#include "Plane.h"

namespace SGE
{
    Plane::Plane() :
        dist(0)
    {
    }
    
    Plane::Plane(const Vec3& _normal, float _dist) :
        normal(_normal),
        dist(_dist)
    {
    }
    
    Plane::Plane(const Vec3& point, const Vec3& _normal) :
        normal(_normal)
    {
        dist = -normal.dot(point);
    }
    
    Plane::Plane(const Plane& p) :
        normal(p.normal),
        dist(p.dist)
    {
    }
    
    Plane& Plane::operator=(const Plane& p)
    {
        normal = p.normal;
        dist = p.dist;
        return *this;
    }
    
    float Plane::distToPoint(const Vec3& v) const
    {
        return (normal.dot(v) + dist) / normal.length();
    }
    
    Vec3 Plane::projectPoint(const Vec3& v) const
    {
        return v + normal.norm() * -distToPoint(v);
    }
    
    bool Plane::classify(const Vec3& v) const
    {
        return distToPoint(v) >= 0;
    }
    
    bool Plane::equals(const Plane& p, float eps) const
    {
        return normal.equals(p.normal,eps) && Math::equal(dist,p.dist,eps);
    }
}
