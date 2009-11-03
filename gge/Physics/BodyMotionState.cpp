#include "BodyMotionState.h"
#include "BulletUtil.h"

namespace gge
{
    namespace Physics
    {
        
        BodyMotionState::BodyMotionState( Entity* entity, btTransform transform ) :
            btDefaultMotionState(transform),
            m_entity(entity)
        {
        }
        
        void BodyMotionState::getWorldTransform( btTransform& transform ) const
        {
            btDefaultMotionState::getWorldTransform(transform);
        }
        
        void BodyMotionState::setWorldTransform( const btTransform& transform )
        {
            btDefaultMotionState::setWorldTransform(transform);
            if ( m_entity )
            {
                m_entity->setVar<Quat>("rot",BulletUtil::toQuat(m_graphicsWorldTrans.getRotation()));
                m_entity->setVar<Vec3>("pos",BulletUtil::toVec(m_graphicsWorldTrans.getOrigin()));
            }
        }

    }
}
