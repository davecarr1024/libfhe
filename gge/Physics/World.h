#ifndef WORLD_H
#define WORLD_H

#include "btBulletDynamicsCommon.h"

#include <gge/Aspect.h>

namespace gge
{
    namespace Physics
    {
        
        class World : public Aspect
        {
            private:
                btAxisSweep3* m_broadphase;
                btDefaultCollisionConfiguration* m_collisionConfiguration;
                btCollisionDispatcher* m_dispatcher;
                btSequentialImpulseConstraintSolver* m_solver;
                btDiscreteDynamicsWorld* m_dynamicsWorld;
                float m_lastTick;
                
            public:
                World();
                ~World();
                
                Var msg_update( const Var& arg );
                
                Var get_dynamicsWorld( const Var& arg );
        };
        
    }
}

#endif
