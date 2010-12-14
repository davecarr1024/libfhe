#include <sim/SpatialNode.h>
#include <sim/Sim.h>
#include <sim/IUpdate.h>

namespace fhe
{
    namespace sim
    {
        FHE_MOD( sim );
        
        FHE_NODE( SpatialNode2 );
        FHE_VAR( SpatialNode2, position );
        FHE_VAR( SpatialNode2, rotation );
        FHE_FUNC( SpatialNode2, localTransform );
        FHE_FUNC( SpatialNode2, globalTransform );
        
        FHE_NODE( SpatialNode3 );
        FHE_VAR( SpatialNode3, position );
        FHE_VAR( SpatialNode3, rotation );
        FHE_FUNC( SpatialNode3, localTransform );
        FHE_FUNC( SpatialNode3, globalTransform );
        
        FHE_NODE( Sim )
        FHE_FUNC( Sim, time )
        FHE_FUNC( Sim, run )
        FHE_FUNC( Sim, shutdown )
        
        FHE_INODE( IUpdate )
        FHE_FUNC( IUpdate, update );
        
        FHE_END_MOD
    }
}
