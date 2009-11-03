#ifndef BODY_MOTION_STATE_H
#define BODY_MOTION_STATE_H

#include "btBulletDynamicsCommon.h"
#include <gge/Entity.h>

namespace gge
{
    namespace Physics
    {
    
        class BodyMotionState : public btDefaultMotionState 
        {
            private:
                Entity* m_entity;
                
            public:
                BodyMotionState( Entity* entity, btTransform transform );
                
                virtual void getWorldTransform( btTransform& transform ) const;
                virtual void setWorldTransform( const btTransform& transform );
        };
        
    }
}

#endif
