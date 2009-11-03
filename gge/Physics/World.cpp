#include "World.h"

namespace gge
{
    namespace Physics
    {
        GGE_ASPECT(World);
        
        World::World() :
            m_broadphase(0),
            m_collisionConfiguration(0),
            m_dispatcher(0),
            m_solver(0),
            m_dynamicsWorld(0),
            m_lastTick(0)
        {
            int maxProxies = 1024;
            btVector3 min(-10000,-10000,-10000), max(10000,10000,10000);
            m_broadphase = new btAxisSweep3(min,max,maxProxies);
            m_collisionConfiguration = new btDefaultCollisionConfiguration();
            m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
            m_solver = new btSequentialImpulseConstraintSolver();
            m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,
                                                        m_broadphase,
                                                        m_solver,
                                                        m_collisionConfiguration);
                                                        
            addFunc("msg_update",&World::msg_update,this);
            addFunc("get_dynamicsWorld",&World::get_dynamicsWorld,this);
        }
        
        World::~World()
        {
            if ( m_dynamicsWorld )
            {
                delete m_broadphase;
                delete m_collisionConfiguration;
                delete m_dispatcher;
                delete m_solver;
    //             delete m_dynamicsWorld;
                m_dynamicsWorld = 0;
            }
        }
        
        void World::msg_update( VarMap args )
        {
            if ( m_dynamicsWorld )
            {
                float time = args.getVar<float>("time",0),
                    dtime = 1.0 / getEntity()->getVar("fps",100);
                if ( time - m_lastTick > dtime )
                {
                    m_lastTick += dtime;
                    m_dynamicsWorld->stepSimulation(dtime);
                }
            }
        }
        
        Var World::get_dynamicsWorld()
        {
            return Var::build<btDiscreteDynamicsWorld*>(m_dynamicsWorld);
        }

    }
}
