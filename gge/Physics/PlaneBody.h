#ifndef PLANE_BODY_H
#define PLANE_BODY_H

#include "Body.h"

namespace gge
{
    
    class PlaneBody : public Body
    {
        public:
            btCollisionShape* makeShape();
    };
    
}

#endif
