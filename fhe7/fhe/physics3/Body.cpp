#include <fhe/physics3/Body.h>
#include <fhe/physics3/World.h>

namespace fhe
{
    namespace physics3
    {
        
        FHE_NODE( Body );
        FHE_DEP( Body, sim, SpatialNode3d );
        FHE_DEP( Body, core, IOnAttach );
        FHE_FUNC( Body, setPosition );
        FHE_FUNC( Body, getPosition );
        FHE_FUNC( Body, setRotation );
        FHE_FUNC( Body, getRotation );
        FHE_FUNC( Body, setMass );
        FHE_FUNC( Body, getMass );
        
        Body::Body() :
            m_rigidBody( 0 ),
            m_collisionShape( 0 ),
            m_dynamicsWorld( 0 ),
            m_mass( 1 )
        {
        }
        
        Body::~Body()
        {
        }
        
        void Body::onAttach()
        {
            Vec3d pos = getPosition();
            Rot3 rot = getRotation();
            double mass = getMass();
            btVector3 inertia;
            
            FHE_ASSERT_MSG( ancestorCall( &World::getDynamicsWorld, m_dynamicsWorld ) && m_dynamicsWorld,
                            "unable to get dynamics world" );
                            
            m_collisionShape = makeShape();
            FHE_ASSERT_MSG( m_collisionShape, "unable to make shape" );
            
            btDefaultMotionState* motionState = 
                new btDefaultMotionState( btTransform( World::convert( rot ), World::convert( pos ) ) );
            m_collisionShape->calculateLocalInertia( mass, inertia );
            btRigidBody::btRigidBodyConstructionInfo rbci( mass, motionState, m_collisionShape, inertia );
            m_rigidBody = new btRigidBody( rbci );
            m_dynamicsWorld->addRigidBody( m_rigidBody );
        }
        
        void Body::onDetach()
        {
            Super::setPosition( getPosition() );
            Super::setRotation( getRotation() );
            m_dynamicsWorld->removeRigidBody( m_rigidBody );
            delete m_rigidBody;
            m_rigidBody = 0;
            m_dynamicsWorld = 0;
        }
        
        void Body::setPosition( Vec3d pos )
        {
            if ( m_rigidBody )
            {
                m_rigidBody->setCenterOfMassTransform( btTransform( World::convert( getRotation() ), World::convert( pos ) ) );
            }
            else
            {
                Super::setPosition( pos );
            }
        }
        
        Vec3d Body::getPosition()
        {
            if ( m_rigidBody )
            {
                return World::convert( m_rigidBody->getCenterOfMassTransform().getOrigin() );
            }
            else
            {
                return Super::getPosition();
            }
        }
        
        void Body::setRotation( Rot3 rot )
        {
            if ( m_rigidBody )
            {
                m_rigidBody->setCenterOfMassTransform( btTransform( World::convert( rot ), World::convert( getPosition() ) ) );
            }
            else
            {
                Super::setRotation( rot );
            }
        }
        
        Rot3 Body::getRotation()
        {
            if ( m_rigidBody )
            {
                return World::convert( m_rigidBody->getCenterOfMassTransform().getRotation() );
            }
            else
            {
                return Super::getRotation();
            }
        }
        
        void Body::setMass( double mass )
        {
            m_mass = mass;
            if ( m_rigidBody )
            {
                btVector3 inertia;
                m_collisionShape->calculateLocalInertia( mass, inertia );
                m_rigidBody->setMassProps( mass, inertia );
            }
        }
        
        double Body::getMass()
        {
            return m_mass;
        }
        
        btCollisionShape* Body::makeShape()
        {
            return 0;
        }
        
    }
}
