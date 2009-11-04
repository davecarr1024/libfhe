#include "AlignedBox.h"
#include "Mat4.h"
#include "ggeMath.h"

namespace gge
{
    
    AlignedBox::AlignedBox()
    {
    }
    
    AlignedBox::AlignedBox(const Vec3& _min, const Vec3& _max) :
        min(_min),
        max(_max)
    {
    }
    
    AlignedBox::AlignedBox(const Vec3List& vl)
    {
        if (!vl.empty())
        {
            min = max = vl[0];
            for (int i = 1; i < vl.size(); ++i)
                expand(vl[i]);
        }
    }
    
    AlignedBox::AlignedBox(const AlignedBox& ab, const Mat4& relativeTransform)
    {
        Vec3List vl;
        ab.getCorners(vl);
        relativeTransform.transform(vl);
        
        min = max = vl[0];
        for (int i = 1; i < vl.size(); ++i)
            expand(vl[i]);
    }
    
    AlignedBox::AlignedBox(const AlignedBox& ab) :
        min(ab.min),
        max(ab.max)
    {
    }
    
    AlignedBox& AlignedBox::operator=(const AlignedBox& ab)
    {
        min = ab.min;
        max = ab.max;
        return *this;
    }
    
    void AlignedBox::expand(const Vec3& v)
    {
        if (v.x < min.x) min.x = v.x;
        if (v.y < min.y) min.y = v.y;
        if (v.z < min.z) min.z = v.z;
        if (v.x > max.x) max.x = v.x;
        if (v.y > max.y) max.y = v.y;
        if (v.z > max.z) max.z = v.z;
    }
    
    void AlignedBox::expand(const AlignedBox& ab, const Mat4& relativeTransform)
    {
        Vec3List corners;
        ab.getCorners(corners);
        relativeTransform.transform(corners);
        for (Vec3List::iterator i = corners.begin(); i != corners.end(); ++i)
            expand(*i);
    }
    
    bool AlignedBox::contains(const Vec3& v) const
    {
        return v.x >= min.x &&
               v.y >= min.y &&
               v.z >= min.z &&
               v.x <= max.x &&
               v.y <= max.y &&
               v.z <= max.z;
    }
    
    bool AlignedBox::overlaps(const AlignedBox& ab, const Mat4& relativeTransform) const
    {
        Mat4 obTransform = relativeTransform * Mat4::translation(ab.getCenter());
        return obTest(getSize(),obTransform,ab.getSize());
    }
    
    //http://www.nuclex.org/articles/static-intersection-test-between-aabb-and-obb
    bool AlignedBox::obTest(const Vec3& abExtents, const Mat4& obTransform, const Vec3& obExtents) const
    {
        float ra, rb, t;
        int i;
        
        for (i = 0; i < 3; ++i)
        {
            ra = abExtents[i];
            rb = obExtents.x * Math::fabs(obTransform.get(i,0)) +
                 obExtents.y * Math::fabs(obTransform.get(i,1)) +
                 obExtents.z * Math::fabs(obTransform.get(i,2));
            t = Math::fabs(obTransform.get(3,i));
            
            if (t > ra + rb)
                return false;
        }
        
        for (i = 0; i < 3; ++i)
        {
            ra = abExtents.x * Math::fabs(obTransform.get(0,i)) +
                 abExtents.y * Math::fabs(obTransform.get(1,i)) +
                 abExtents.z * Math::fabs(obTransform.get(2,i));
            rb = obExtents[i];
            t = Math::fabs(obTransform.get(0,3) * obTransform.get(0,i) +
                      obTransform.get(1,3) * obTransform.get(1,i) +
                      obTransform.get(2,3) * obTransform.get(2,i));
                      
            if (t > ra + rb)
                return false;
        }
        
        //A0 x B0
        ra = abExtents.y * Math::fabs(obTransform.get(0,2)) +
             abExtents.z * Math::fabs(obTransform.get(0,1));
        rb = obExtents.y * Math::fabs(obTransform.get(2,0)) +
             obExtents.z * Math::fabs(obTransform.get(1,0));
        t = Math::fabs(obTransform.get(2,3) * obTransform.get(0,1) - obTransform.get(1,3) * obTransform.get(0,2));
        if (t > ra + rb)
            return false;
        
        //A0 x B1
        ra = abExtents.y * Math::fabs(obTransform.get(1,2)) +
             abExtents.z * Math::fabs(obTransform.get(1,1));
        rb = obExtents.x * Math::fabs(obTransform.get(2,0)) +
             obExtents.z * Math::fabs(obTransform.get(0,0));
        t = Math::fabs(obTransform.get(2,3) * obTransform.get(1,1) - obTransform.get(1,3) * obTransform.get(1,2));
        if (t > ra + rb)
            return false;
        
        //A0 x B2
        ra = abExtents.x * Math::fabs(obTransform.get(1,0)) +
             abExtents.z * Math::fabs(obTransform.get(2,1));
        rb = obExtents.x * Math::fabs(obTransform.get(1,0)) +
             obExtents.y * Math::fabs(obTransform.get(0,0));
        t = Math::fabs(obTransform.get(2,3) * obTransform.get(2,1) - obTransform.get(1,3) * obTransform.get(2,2));
        if (t > ra + rb)
            return false;
        
        //A1 x B0
        ra = abExtents.x * Math::fabs(obTransform.get(0,2)) +
             abExtents.z * Math::fabs(obTransform.get(0,0));
        rb = obExtents.y * Math::fabs(obTransform.get(2,1)) +
             obExtents.z * Math::fabs(obTransform.get(1,1));
        t = Math::fabs(obTransform.get(0,3) * obTransform.get(0,2) - obTransform.get(2,3) * obTransform.get(0,0));
        if (t > ra + rb)
            return false;
        
        //A1 x B1
        ra = abExtents.x * Math::fabs(obTransform.get(1,2)) +
             abExtents.z * Math::fabs(obTransform.get(1,0));
        rb = obExtents.x * Math::fabs(obTransform.get(2,1)) +
             obExtents.z * Math::fabs(obTransform.get(0,1));
        t = Math::fabs(obTransform.get(0,3) * obTransform.get(1,2) - obTransform.get(2,3) * obTransform.get(1,0));
        if (t > ra + rb)
            return false;
        
        //A1 x B2
        ra = abExtents.x * Math::fabs(obTransform.get(2,2)) +
             abExtents.z * Math::fabs(obTransform.get(2,0));
        rb = obExtents.x * Math::fabs(obTransform.get(1,1)) +
             obExtents.y * Math::fabs(obTransform.get(0,1));
        t = Math::fabs(obTransform.get(0,3) * obTransform.get(2,2) - obTransform.get(2,3) * obTransform.get(2,0));
        if (t > ra + rb)
            return false;
        
        //A2 x B0
        ra = abExtents.x * Math::fabs(obTransform.get(0,1)) +
             abExtents.y * Math::fabs(obTransform.get(0,0));
        rb = obExtents.y * Math::fabs(obTransform.get(2,2)) +
             obExtents.z * Math::fabs(obTransform.get(1,2));
        t = Math::fabs(obTransform.get(1,3) * obTransform.get(0,0) - obTransform.get(0,3) * obTransform.get(0,1));
        if (t > ra + rb)
            return false;
        
        //A2 x B1
        ra = abExtents.x * Math::fabs(obTransform.get(1,1)) +
             abExtents.y * Math::fabs(obTransform.get(1,0));
        rb = obExtents.x * Math::fabs(obTransform.get(2,2)) +
             obExtents.z * Math::fabs(obTransform.get(0,2));
        t = Math::fabs(obTransform.get(1,3) * obTransform.get(1,0) - obTransform.get(0,3) * obTransform.get(1,1));
        if (t > ra + rb)
            return false;
        
        //A2 x B2
        ra = abExtents.x * Math::fabs(obTransform.get(2,1)) +
             abExtents.y * Math::fabs(obTransform.get(2,0));
        rb = obExtents.x * Math::fabs(obTransform.get(1,2)) +
             obExtents.y * Math::fabs(obTransform.get(0,2));
        t = Math::fabs(obTransform.get(1,3) * obTransform.get(2,0) - obTransform.get(0,3) * obTransform.get(2,1));
        if (t > ra + rb)
            return false;
        
        return true;
    }
    
    void AlignedBox::getCorners(Vec3List& vl) const
    {
        Vec3 size = getSize();
        int dx, dy, dz;
        for (dx = 0; dx <= 1; ++dx)
            for (dy = 0; dy <= 1; ++dy)
                for (dz = 0; dz <= 1; ++dz)
                    vl.push_back(Vec3(min.x + size.x * dx, min.y + size.y * dy, min.z + size.z * dz));
    }
    
    Vec3 AlignedBox::getSize() const
    {
        return max - min;
    }
    
    Vec3 AlignedBox::getCenter() const
    {
        return (max + min) / 2.0;
    }
}
