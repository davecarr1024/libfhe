#include <physics/World.h>

namespace fhe
{
    namespace physics
    {
        
        FHE_NODE( World );
        FHE_EXT_DEP( World, sim, IUpdate );
        FHE_FUNC( World, update );
        FHE_FUNC( World, getDynamicsWorld );
        FHE_VAR( World, fps );
        
        World::World() :
            m_broadphase( 0 ),
            m_collisionConfiguration( 0 ),
            m_dispatcher( 0 ),
            m_solver( 0 ),
            m_dynamicsWorld( 0 ),
            m_lastUpdate( -1 ),
            fps( 100 )
        {
            m_broadphase = new btAxisSweep3( btVector3( -1e5, -1e5, -1e5 ), btVector3( 1e5, 1e5, 1e5 ), 1024 );
            m_collisionConfiguration = new btDefaultCollisionConfiguration();
            m_dispatcher = new btCollisionDispatcher( m_collisionConfiguration );
            m_solver = new btSequentialImpulseConstraintSolver();
            m_dynamicsWorld = new btDiscreteDynamicsWorld( m_dispatcher,
                                                           m_broadphase,
                                                           m_solver,
                                                           m_collisionConfiguration );
        }
        
        World::~World()
        {
            if ( m_dynamicsWorld )
            {
                delete m_dynamicsWorld;
                delete m_solver;
                delete m_dispatcher;
                delete m_collisionConfiguration;
                delete m_broadphase;
                m_dynamicsWorld = 0;
            }
        }
        
        void World::update( double time, double dtime )
        {
            if ( m_lastUpdate < 0 )
            {
                m_lastUpdate = time;
            }
            
            double d = 1.0 / fps;
            while ( time - m_lastUpdate > d )
            {
                m_lastUpdate += d;
                m_dynamicsWorld->stepSimulation( d );
            }
        }
        
        btDiscreteDynamicsWorld* World::getDynamicsWorld()
        {
            return m_dynamicsWorld;
        }
        
        btVector3 World::convert( const Vec3& v )
        {
            return btVector3( v.x, v.y, v.z );
        }
        
        Vec3 World::convert( const btVector3& v )
        {
            return Vec3( v.x(), v.y(), v.z() );
        }
        
        btQuaternion World::convert( const Rot3& r )
        {
            return btQuaternion( r.x, r.y, r.z, r.w );
        }
        
        Rot3 World::convert( const btQuaternion& r )
        {
            return Rot3( r.w(), r.x(), r.y(), r.z() );
        }
    }
}
