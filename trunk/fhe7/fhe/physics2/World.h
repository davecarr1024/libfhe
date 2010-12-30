#ifndef FHE_PHYSICS2_WORLD_H
#define FHE_PHYSICS2_WORLD_H

#include <fhe/core/Node.h>
#include <fhe/core/Vec.h>
#include <fhe/core/Rot.h>
#include <fhe/sim/IUpdate.h>
#include <Box2D/Box2D.h>

namespace fhe
{
    namespace physics2
    {
        
        class World : public Node, public sim::IUpdate
        {
            private:
                b2World m_world;
                double m_lastUpdate;
            
            public:
                double fps;
                
                World();
                virtual ~World();
                
                virtual void update( double time, double dtime );
                
                b2World* getWorld();
                
                static b2Vec2 convert( const Vec2d& v );
                static Vec2d convert( const b2Vec2& v );
                static Rot2d convert( double r );
                static double convert( const Rot2d& r );
        };
        
    }
}

#endif
