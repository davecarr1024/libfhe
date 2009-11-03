#ifndef CUBE_BODY_H
#define CUBE_BODY_H

#include "Body.h"

namespace gge
{
    namespace Physics
    {
    
        class Cube : public Body
        {
            public:
                btCollisionShape* makeShape();
        };
    }
}

#endif
