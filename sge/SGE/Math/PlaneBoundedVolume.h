#ifndef PLANE_BOUNDED_VOLUME_H
#define PLANE_BOUNDED_VOLUME_H

#include "Plane.h"
#include "SGEMath.h"

namespace SGE
{
    
    class PlaneBoundedVolume
    {
        public:
            PlaneList planes;
            
            PlaneBoundedVolume();
            PlaneBoundedVolume(const PlaneList& _planes);
            PlaneBoundedVolume(const PlaneBoundedVolume& pbv);
            
            PlaneBoundedVolume& operator=(const PlaneBoundedVolume& pbv);
            
            bool contains(const Vec3& v) const;
            bool contains(const Vec3List& vl) const;
            
            bool overlaps(const Vec3List& vl) const;
            
            bool equals(const PlaneBoundedVolume& pbv, float eps = Math::EPSILON) const;
    };
    
}

#endif
