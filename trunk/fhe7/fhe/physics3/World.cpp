#include <fhe/physics3/World.h>

namespace fhe
{
    namespace physics3
    {
        
        FHE_NODE( World );
        FHE_DEP( World, sim, IUpdate );
        FHE_FUNC( World, update );
        FHE_FUNC( World, getDynamicsWorld );
        FHE_FUNC( World, setGravity );
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
            setGravity( Vec3d( 0, 0, -10 ) );
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
        
        void World::setGravity( const Vec3d& gravity )
        {
            m_dynamicsWorld->setGravity( convert( gravity ) );
        }
        
        btDiscreteDynamicsWorld* World::getDynamicsWorld()
        {
            return m_dynamicsWorld;
        }
        
        btVector3 World::convert( const Vec3d& v )
        {
            return btVector3( v.x, v.y, v.z );
        }
        
        Vec3d World::convert( const btVector3& v )
        {
            return Vec3d( v.x(), v.y(), v.z() );
        }
        
        btQuaternion World::convert( const Rot3d& r )
        {
            return btQuaternion( r.x, r.y, r.z, r.w );
        }
        
        Rot3d World::convert( const btQuaternion& r )
        {
            return Rot3d( r.w(), r.x(), r.y(), r.z() );
        }
    }
}
