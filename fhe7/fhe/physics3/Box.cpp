#include <fhe/physics3/Box.h>
#include <fhe/physics3/World.h>

namespace fhe
{
    namespace physics3
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
