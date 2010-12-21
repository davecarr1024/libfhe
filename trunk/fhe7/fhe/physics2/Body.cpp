#include <fhe/physics2/Body.h>
#include <fhe/physics2/World.h>

namespace fhe
{
    namespace physics2
    {
        
        FHE_NODE( Body );
        FHE_DEP( Body, sim, SpatialNode2d );
        FHE_DEP( Body, core, IOnAttach );
        FHE_FUNC( Body, onAttach );
        FHE_FUNC( Body, onDetach );
        FHE_FUNC( Body, getPosition );
        FHE_FUNC( Body, setPosition );
        FHE_FUNC( Body, getRotation );
        FHE_FUNC( Body, setRotation );
        
        Body::Body() :
            m_world( 0 ),
            m_body( 0 )
        {
            m_def.type = b2_dynamicBody;
        }
        
        Body::~Body()
        {
        }
        
        b2Body* Body::getBody()
        {
            return m_body;
        }
        
        void Body::onAttach()
        {
            FHE_ASSERT_MSG( ancestorCall( &World::getWorld, m_world ) && m_world,
                            "unable to get world" );
            
            m_body = m_world->CreateBody( &m_def );
            FHE_ASSERT( m_body );
        }
        
        void Body::onDetach()
        {
            m_world->DestroyBody( m_body );
            m_body = 0;
            m_world = 0;
        }
        
        Vec2d Body::getPosition()
        {
            return World::convert( m_body ? m_body->GetPosition() : m_def.position );
        }
        
        void Body::setPosition( Vec2d pos )
        {
            if ( m_body )
            {
                m_body->SetTransform( World::convert( pos ), World::convert( getRotation() ) );
            }
            else
            {
                m_def.position = World::convert( pos );
            }
        }
        
        Rot2 Body::getRotation()
        {
            return World::convert( m_body ? m_body->GetAngle() : m_def.angle );
        }
        
        void Body::setRotation( Rot2 rot )
        {
            if ( m_body )
            {
                m_body->SetTransform( World::convert( getPosition() ), World::convert( rot ) );
            }
            else
            {
                m_def.angle = World::convert( rot );
            }
        }
    }
}
