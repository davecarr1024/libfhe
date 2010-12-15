#ifndef FHE_PHYSICS_WORLD_H
#define FHE_PHYSICS_WORLD_H

#include <fhe/Node.h>
#include <sim/IUpdate.h>
#include <btBulletDynamicsCommon.h>

namespace fhe
{
    namespace physics
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
                
                btDiscreteDynamicsWorld* getDynamicsWorld();
                
                static btVector3 convert( const Vec3& v );
                static Vec3 convert( const btVector3& v );
                static btQuaternion convert( const Rot3& r );
                static Rot3 convert( const btQuaternion& r );
        };
        
    }
}

#endif
