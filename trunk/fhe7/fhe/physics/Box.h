#ifndef FHE_PHYSICS_BOX_H
#define FHE_PHYSICS_BOX_H

#include <fhe/physics/Body.h>
#include <fhe/core/Vec.h>

namespace fhe
{
    namespace physics
    {
        
        class Box : public Body
        {
            public:
                Vec3 size;
                
                btCollisionShape* makeShape();
        };
        
    }
}

#endif
