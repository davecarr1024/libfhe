#ifndef FHE_PHYSICS3_WORLD_H
#define FHE_PHYSICS3_WORLD_H

#include <fhe/core/Node.h>
#include <fhe/sim/IUpdate.h>
#include <btBulletDynamicsCommon.h>

namespace fhe
{
    namespace physics3
    {
        
        class World : public Node, public sim::IUpdate
        {
            private:
                btAxisSweep3* m_broadphase;
                btDefaultCollisionConfiguration* m_collisionConfiguration;
                btCollisionDispatcher* m_dispatcher;
                btSequentialImpulseConstraintSolver* m_solver;
                btDiscreteDynamicsWorld* m_dynamicsWorld;
                double m_lastUpdate;
                
            public:
                double fps;
                
                World();
                virtual ~World();
                
                virtual void update( double time, double dtime );
                
                void setGravity( const Vec3d& gravity );
                
                btDiscreteDynamicsWorld* getDynamicsWorld();
                
                static btVector3 convert( const Vec3d& v );
                static Vec3d convert( const btVector3& v );
                static btQuaternion convert( const Rot3d& r );
                static Rot3d convert( const btQuaternion& r );
        };
        
    }
}

#endif
