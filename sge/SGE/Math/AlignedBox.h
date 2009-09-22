#ifndef ALIGNED_BOX_H
#define ALIGNED_BOX_H

#include "Vec3.h"

#include <vector>
#include <iostream>

namespace SGE
{
    class Mat4;
    class AlignedBox;
    
    typedef std::vector< const AlignedBox& > AlignedBoxList;
    
    class AlignedBox
    {
        private:
            bool obTest(const Vec3& abExtents, const Mat4& obTransform, const Vec3& obExtents) const;
        
        public:
            
            Vec3 min, max;
            
            AlignedBox();
            AlignedBox(const Vec3& _min, const Vec3& max);
            AlignedBox(const Vec3List& vl);
            AlignedBox(const AlignedBox& ab, const Mat4& relativeTransform);
            AlignedBox(const AlignedBox& ab);
            
            AlignedBox& operator=(const AlignedBox& ab);
            
            void expand(const Vec3& v);
            void expand(const AlignedBox& ab, const Mat4& relativeTransform);
            
            bool contains(const Vec3& v) const;
            
            bool overlaps(const AlignedBox& ab, const Mat4& relativeTransform) const;
            
            void getCorners(Vec3List& vl) const;
            Vec3 getSize() const;
            Vec3 getCenter() const;
            
            friend std::ostream& operator<<(std::ostream& os, const AlignedBox& ab);
    };
    
}

#endif
