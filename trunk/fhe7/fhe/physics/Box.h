#ifndef FHE_PHYSICS_BODY_H
#define FHE_PHYSICS_BODY_H

#include <fhe/physics/Body.h>
#include <fhe/core/Vec.h>

namespace fhe
{
    namespace physics
    {
        
        class Box
        {
            public:
//                 Vec3 size;
                
                btCollisionShape* makeShape();
        };
        
    }
}

#endif
