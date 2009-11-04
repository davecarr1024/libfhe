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
                
                Var on_attach( const Var& arg );
                Var on_detach( const Var& arg );
                
                virtual btCollisionShape* makeShape();
        };

    }
}

#endif
