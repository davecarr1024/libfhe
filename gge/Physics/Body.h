#ifndef BODY_H
#define BODY_H

#include "btBulletDynamicsCommon.h"
#include <gge/Aspect.h>

namespace gge
{
    namespace Physics
    {
    
        class Body : public Aspect
        {
            public:
                Body();
                
                void on_attach();
                void on_detach();
                
                virtual btCollisionShape* makeShape();
        };

    }
}

#endif
