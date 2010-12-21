#ifndef FHE_PHYSICS3_BODY_H
#define FHE_PHYSICS3_BODY_H

#include <fhe/sim/SpatialNode.h>
#include <fhe/core/IOnAttach.h>
#include <btBulletDynamicsCommon.h>

namespace fhe
{
    namespace physics3
    {
        class Body : public sim::SpatialNode3d, public IOnAttach
        {
            typedef sim::SpatialNode3d Super;
            
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
                
                virtual void setPosition( Vec3d pos );
                virtual Vec3d getPosition();
                virtual void setRotation( Rot3 rot );
                virtual Rot3 getRotation();
                virtual void setMass( double mass );
                virtual double getMass();
                
                virtual btCollisionShape* makeShape();
        };
    }
}

#endif
