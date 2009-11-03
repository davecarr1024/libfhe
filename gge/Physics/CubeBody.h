#ifndef CUBE_BODY_H
#define CUBE_BODY_H

#include "Body.h"

namespace gge
{
    
    class CubeBody : public Body
    {
        public:
            btCollisionShape* makeShape();
    };
    
}

#endif
