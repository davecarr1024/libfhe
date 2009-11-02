#include "PlaneBoundedVolume.h"
#include "Vec3.h"

namespace gge
{
    
    PlaneBoundedVolume::PlaneBoundedVolume()
    {
    }
    
    PlaneBoundedVolume::PlaneBoundedVolume(const PlaneList& _planes) :
        planes(_planes)
    {
    }
    
    PlaneBoundedVolume::PlaneBoundedVolume(const PlaneBoundedVolume& pbv) :
        planes(pbv.planes)
    {
    }
    
    PlaneBoundedVolume& PlaneBoundedVolume::operator=(const PlaneBoundedVolume& pbv)
    {
        planes = pbv.planes;
    }
    
    bool PlaneBoundedVolume::contains(const Vec3& v) const
    {
        for (PlaneList::const_iterator i = planes.begin(); i != planes.end(); ++i)
            if (!i->classify(v))
                return false;
        return true;
    }
    
    bool PlaneBoundedVolume::contains(const Vec3List& vl) const
    {
        for (Vec3List::const_iterator i = vl.begin(); i != vl.end(); ++i)
            if (!contains(*i))
                return false;
        return true;
    }
    
    bool PlaneBoundedVolume::overlaps(const Vec3List& vl) const
    {
        for (Vec3List::const_iterator i = vl.begin(); i != vl.end(); ++i)
            if (contains(*i))
                return true;
        return false;
    }

    bool PlaneBoundedVolume::equals(const PlaneBoundedVolume& pbv, float eps) const
    {
        for (int i = 0; i < planes.size(); ++i)
            if (!const_cast<PlaneBoundedVolume*>(this)->planes[i].equals(const_cast<PlaneBoundedVolume&>(pbv).planes[i]))
                return false;
        return true;
    }
}
