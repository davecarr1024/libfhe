#ifndef FHE_PHYSICS_BODY_H
#define FHE_PHYSICS_BODY_H

#include <sim/SpatialNode.h>
#include <fhe/IOnAttach.h>
#include <btBulletDynamicsCommon.h>

namespace fhe
{
    namespace physics
    {
        class Body : public sim::SpatialNode3, public IOnAttach
        {
            typedef sim::SpatialNode3 Super;
            
            private:
                btRigidBody* m_rigidBody;
                btCollisionShape* m_collisionShape;
                btDiscreteDynamicsWorld* m_dynamicsWorld;
                double m_mass;
                
            public:
                Body();
                virtual ~Body();
                
                virtual void onAttach();
                virtual void onDetach();
                
                virtual void setPosition( Vec3 pos );
                virtual Vec3 getPosition();
                virtual void setRotation( Rot3 rot );
                virtual Rot3 getRotation();
                virtual void setMass( double mass );
                virtual double getMass();
                
                virtual btCollisionShape* makeShape();
        };
    }
}

#endif
