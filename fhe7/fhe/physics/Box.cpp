#include <fhe/physics/Box.h>
#include <fhe/physics/World.h>

namespace fhe
{
    namespace physics
    {
        
        FHE_NODE( Box );
        FHE_DEP( Box, physics, Body );
        FHE_VAR( Box, size );
        
        btCollisionShape* Box::makeShape()
        {
            return new btBoxShape( World::convert( size ) );
        }
        
    }
}
