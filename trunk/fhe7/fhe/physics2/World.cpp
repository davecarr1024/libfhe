#include <fhe/physics2/World.h>

namespace fhe
{
    namespace physics2
    {
        FHE_NODE( World );
        FHE_DEP( World, sim, IUpdate );
        FHE_VAR( World, fps );
        FHE_FUNC( World, update );
        FHE_FUNC( World, getWorld );
        
        World::World() :
            m_world( b2Vec2( 0, -10 ), true ),
            m_lastUpdate( -1 ),
            fps( 100 )
        {
        }
        
        World::~World()
        {
        }
        
        b2World* World::getWorld()
        {
            return &m_world;
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
                m_world.Step( d, 6, 2 );
            }
        }
        
        b2Vec2 World::convert( const Vec2d& v )
        {
            return b2Vec2( v.x, v.y );
        }
        
        Vec2d World::convert( const b2Vec2& v )
        {
            return Vec2d( v.x, v.y );
        }
        
        Rot2 World::convert( double r )
        {
            return Rot2::fromRadians( r );
        }
        
        double World::convert( const Rot2& r )
        {
            return r.radians();
        }
        
    }
}
