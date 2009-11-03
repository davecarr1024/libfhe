#ifndef PLANE_BODY_H
#define PLANE_BODY_H

#include "Body.h"

namespace gge
{
    namespace Physics
    {
        
        class Plane : public Body
        {
            public:
                btCollisionShape* makeShape();
        };
        
    }
}

#endif
