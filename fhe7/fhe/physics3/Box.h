#ifndef FHE_PHYSICS3_BOX_H
#define FHE_PHYSICS3_BOX_H

#include <fhe/physics3/Body.h>
#include <fhe/core/Vec.h>

namespace fhe
{
    namespace physics3
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
